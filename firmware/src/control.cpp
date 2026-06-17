// =============================================================================
// control.cpp - mode/fault state machine implementation
//
// Mode is a SELECTED state (s.mode), advanced ONLY by explicit events:
//   - cycleMode(): local button short-press cycles OFF -> AUTO -> MANUAL -> OFF
//   - setMode():   a remote (Home Assistant) command sets it
// Last writer wins (main applies remote then local each tick). evaluate() never
// rewrites s.mode, so the selected mode survives a fault (shown in HA, not lost).
//
// Latching fault policy (default safe, documented in README):
//   Most faults LATCH and force the load OFF. A latched fault clears only when
//   BOTH (a) every fault condition currently reads healthy continuously for
//   FAULT_RECOVERY_HOLD_MS, AND (b) an explicit operator acknowledge is given
//   (mode set to OFF, OR a button long-press / HA Fault Reset).
//
//   EXCEPTION (clear-on-recovery, no ack): FAULT_PV_INVALID and FAULT_SENSOR_STALE
//   auto-clear once their condition is healthy again for the healthy-hold window.
//   The PV surplus is an environmental signal that can momentarily rail, and a
//   (typically network/MQTT) feed can briefly stop and resume; requiring a human
//   to re-arm after every such transient would defeat unattended operation. Both
//   still force the load OFF while active, and the AUTO ON path re-validates the
//   live surplus before energizing, so a transient cannot energize the load.
//
//   A long-press / Fault Reset acknowledge is an operator OVERRIDE that clears
//   ALL latched bits once the healthy-hold has elapsed -- including manual-clear-
//   only bits such as FAULT_INTERNAL.
// =============================================================================

#include "control.h"
#include "config.h"

namespace control {

namespace {

// Runtime-adjustable AUTO thresholds (W), seeded from the config defaults.
// Remote control (HA) may retune them via setThresholds(); the setter enforces
// off < on so the hysteresis can never be inverted by a bad command.
uint32_t g_onW  = SURPLUS_ON_THRESHOLD_W;
uint32_t g_offW = SURPLUS_OFF_THRESHOLD_W;

// Compute the set of fault conditions present THIS tick (not yet latched).
uint16_t detectFaults(const Inputs& in) {
  uint16_t f = FAULT_NONE;

  if (!in.surplusValid) f |= FAULT_PV_INVALID;

  switch (in.tempStatus) {
    case temperature::Status::OPEN:         f |= FAULT_NTC_OPEN;  break;
    case temperature::Status::SHORT:        f |= FAULT_NTC_SHORT; break;
    case temperature::Status::OUT_OF_RANGE: f |= FAULT_NTC_RANGE; break;
    case temperature::Status::OK:
      // Overtemp is only trusted when the NTC reads healthy; an unhealthy NTC
      // faults on its own above, so it can never silently mask an overtemp.
      if (in.tempC > TEMPERATURE_LIMIT_C) f |= FAULT_OVERTEMP;
      break;
  }

  // Sensor heartbeat: if either sample path has gone stale (update() no longer
  // called, ADC wedged, MQTT surplus feed stopped, etc.) the checks above keep
  // consuming a frozen value. Treat lost freshness as a missing-sensor-value
  // fault. PV and temperature have independent windows (PV may be a slow network
  // feed; temperature is a fast local sample).
  if (in.pvSampleAgeMs > PV_STALE_MS || in.tempSampleAgeMs > TEMP_STALE_MS) {
    f |= FAULT_SENSOR_STALE;
  }

  return f;
}

// True when a given latched fault's underlying condition currently reads healthy.
// Overtemp uses a clear margin to avoid chatter at the limit.
bool conditionHealthy(uint16_t bit, const Inputs& in) {
  switch (bit) {
    case FAULT_PV_INVALID:    return in.surplusValid;
    case FAULT_NTC_OPEN:      return in.tempStatus != temperature::Status::OPEN;
    case FAULT_NTC_SHORT:     return in.tempStatus != temperature::Status::SHORT;
    case FAULT_NTC_RANGE:     return in.tempStatus != temperature::Status::OUT_OF_RANGE;
    case FAULT_OVERTEMP:      return in.tempStatus == temperature::Status::OK &&
                                     in.tempC < (TEMPERATURE_LIMIT_C - OVERTEMP_CLEAR_MARGIN_C);
    case FAULT_SENSOR_STALE:  return in.pvSampleAgeMs <= PV_STALE_MS &&
                                     in.tempSampleAgeMs <= TEMP_STALE_MS;
    // An internal invariant violation has no self-measurable "healthy" reading;
    // it stays asserted until an operator long-press override clears it. The
    // recovery gate treats it specially (see allRecoverableHealthy / evaluate).
    case FAULT_INTERNAL:      return false;
    default:                  return true;
  }
}

// True when bit is a "manual-clear-only" fault with no self-measurable recovery.
// These are cleared by an operator long-press override, not by clear-on-recovery.
bool isManualClearOnly(uint16_t bit) {
  return bit == FAULT_INTERNAL;
}

// Do all SELF-RECOVERABLE latched faults read healthy right now? Manual-clear-
// only bits (e.g. FAULT_INTERNAL) are excluded here so the healthy-hold timer
// can still advance; clearing them additionally requires the operator override.
bool allRecoverableHealthy(uint16_t faults, const Inputs& in) {
  for (uint16_t bit = 1; bit != 0; bit <<= 1) {
    if ((faults & bit) && !isManualClearOnly(bit) && !conditionHealthy(bit, in)) {
      return false;
    }
  }
  return true;
}

// Faults that auto-clear once their condition reads healthy again, WITHOUT an
// operator acknowledge. These are environmental / link conditions (not hardware
// faults that warrant inspection): a railed/missing surplus and a stopped-then-
// resumed sample feed. Essential for an unattended HA-fed controller so a WiFi or
// broker hiccup does not require a human to press Fault Reset afterwards. The load
// stays OFF while they are active regardless.
bool isAutoClearOnRecovery(uint16_t bit) {
  return bit == FAULT_PV_INVALID || bit == FAULT_SENSOR_STALE;
}

// AUTO hysteresis sub-FSM. Updates pending timers and the committed autoLoadOn
// sub-state. Delay timers measure CONTINUOUS hold (reset when the condition
// lapses). Returns the desired AUTO load level (before the dwell gate). The
// dwell gate in commit() may temporarily hold the physical load OFF while
// autoLoadOn==true; this is benign and self-correcting (a real surplus deficit
// flips autoLoadOn back via the OFF delay).
bool autoStep(State& s, const Inputs& in, uint32_t nowMs) {
  if (!s.autoLoadOn) {
    // Currently OFF: look for sustained surplus above the ON threshold.
    if (in.surplusW > g_onW) {
      if (!s.onPending) {
        s.onPending = true;
        s.onCandidateSinceMs = nowMs;
      }
      if ((uint32_t)(nowMs - s.onCandidateSinceMs) >= SWITCH_ON_DELAY_MS) {
        s.autoLoadOn = true;
        s.onPending = false;
      }
    } else {
      s.onPending = false;  // condition lapsed -> reset continuous-hold timer
    }
  } else {
    // autoLoadOn is latched true. Two cases:
    //  (1) the load is NOT yet physically committed (dwell gate still holding).
    //      The ON decision is not yet realized, so the spec's "switch ON only if
    //      surplus > on_threshold" must still hold AT the energizing instant. If
    //      surplus has fallen out of the ON region (into the dead-band or below),
    //      drop the latch immediately so we never energize on a stale crossing.
    //  (2) the load IS committed ON: normal OFF hysteresis (sustained surplus
    //      below the OFF threshold for the OFF delay).
    if (!s.committedLoad) {
      if (in.surplusW <= g_onW) {
        s.autoLoadOn = false;  // ON condition no longer live -> abandon the pending ON
        s.offPending = false;
      }
    } else if (in.surplusW < g_offW) {
      if (!s.offPending) {
        s.offPending = true;
        s.offCandidateSinceMs = nowMs;
      }
      if ((uint32_t)(nowMs - s.offCandidateSinceMs) >= SWITCH_OFF_DELAY_MS) {
        s.autoLoadOn = false;
        s.offPending = false;
      }
    } else {
      s.offPending = false;
    }
  }
  return s.autoLoadOn;
}

// Reset AUTO sub-state (called on entering AUTO, leaving AUTO, or on fault).
void resetAuto(State& s) {
  s.onPending = false;
  s.offPending = false;
  s.autoLoadOn = false;
}

}  // namespace

void init(State& s, uint32_t nowMs) {
  s.mode = Mode::OFF;
  s.manualOn = false;
  s.faults = FAULT_NONE;
  s.lastLoadChangeMs = nowMs;
  s.onCandidateSinceMs = nowMs;
  s.offCandidateSinceMs = nowMs;
  s.onPending = false;
  s.offPending = false;
  s.autoLoadOn = false;
  s.committedLoad = false;
  s.healthySinceMs = nowMs;
  s.wasHealthy = true;
}

namespace {

// Commit a desired load level into the decision. OFF transitions are always
// immediate. ON transitions are gated by MINIMUM_DWELL_TIME_MS ONLY when
// dwellGated is true (AUTO anti-cycling). MANUAL/OFF/FAULT pass dwellGated=false
// so a local manual override and all forced-off actions are immediate -- the
// dwell is an AUTO-only concern per spec, never something that ignores an
// operator's button press for up to 30 s.
void commit(State& s, Decision& d, bool desired, bool dwellGated, uint32_t nowMs) {
  if (desired && !s.committedLoad && dwellGated) {
    // Turning ON under AUTO: honor minimum dwell since the last change.
    if ((uint32_t)(nowMs - s.lastLoadChangeMs) < MINIMUM_DWELL_TIME_MS) {
      d.desiredLoad = false;  // hold OFF until dwell elapses; retried each tick
      return;
    }
  }
  d.desiredLoad = desired;
  if (desired != s.committedLoad) {
    s.committedLoad = desired;
    s.lastLoadChangeMs = nowMs;
  }
}

}  // namespace

Decision evaluate(State& s, const Inputs& in, uint32_t nowMs) {
  Decision d;
  d.desiredLoad = false;
  d.activeMode = s.mode;   // the selected mode (never clobbered, even on fault)
  d.faultMask = FAULT_NONE;
  d.reason = "init";

  // --- 1) Fault detection + latch ------------------------------------------
  uint16_t live = detectFaults(in);
  s.faults |= live;  // latch new faults

  // --- 2) Recovery gate -----------------------------------------------------
  // Track how long all SELF-RECOVERABLE latched conditions have been healthy.
  //  - FAULT_PV_INVALID / FAULT_SENSOR_STALE are clear-on-recovery (no ack).
  //  - All other self-recoverable faults clear on healthy-hold + operator ack
  //    (mode OFF, button long-press, or HA Fault Reset).
  //  - Manual-clear-only faults (FAULT_INTERNAL) clear only via a long-press
  //    override, and only once the recoverable faults are also healthy.
  if (s.faults != FAULT_NONE) {
    bool healthyNow = allRecoverableHealthy(s.faults, in);
    if (healthyNow) {
      if (!s.wasHealthy) {
        s.healthySinceMs = nowMs;  // rising edge of "all recoverable healthy"
        s.wasHealthy = true;
      }
      bool holdElapsed = (uint32_t)(nowMs - s.healthySinceMs) >= FAULT_RECOVERY_HOLD_MS;
      bool ack = in.buttonLong || (s.mode == Mode::OFF);
      bool longPressOverride = in.buttonLong;  // long-press is a full operator override

      uint16_t clearable = 0;
      if (holdElapsed) {
        for (uint16_t bit = 1; bit != 0; bit <<= 1) {
          if (!(s.faults & bit)) continue;
          if (isAutoClearOnRecovery(bit)) {
            clearable |= bit;                    // environmental/link: no ack needed
          } else if (ack && !isManualClearOnly(bit)) {
            clearable |= bit;                    // self-recoverable: needs operator ack
          } else if (longPressOverride && isManualClearOnly(bit)) {
            clearable |= bit;                    // internal: long-press override only
          }
        }
      }

      if (clearable) {
        s.faults &= ~clearable;
        if (s.faults == FAULT_NONE) {
          s.manualOn = false;  // require a fresh affirmative action after a full clear
          resetAuto(s);
        }
      }
    } else {
      s.wasHealthy = false;
    }
  } else {
    s.wasHealthy = true;
  }

  // --- 3) FAULT priority ----------------------------------------------------
  if (s.faults != FAULT_NONE) {
    resetAuto(s);                       // AUTO timers do not run while faulted
    d.faultMask = s.faults;
    d.reason = "fault";
    commit(s, d, false, false, nowMs);  // forced OFF, immediate (no dwell)
    return d;                           // FAULT cannot be overridden by any mode
  }

  // --- 4) OFF priority ------------------------------------------------------
  if (s.mode == Mode::OFF) {
    d.reason = "mode OFF";
    commit(s, d, false, false, nowMs);  // OFF is immediate
    return d;
  }

  // --- 5) MANUAL priority ---------------------------------------------------
  bool desired = false;
  bool dwellGated = false;  // dwell anti-cycling applies to AUTO only
  if (s.mode == Mode::MANUAL) {
    desired = s.manualOn;   // manualOn is set externally (local long-press / HA switch)
    d.reason = s.manualOn ? "manual on" : "manual off";
    // dwellGated stays false: a manual command takes effect immediately.
  }
  // --- 6) AUTO priority -----------------------------------------------------
  else if (s.mode == Mode::AUTO) {
    desired = autoStep(s, in, nowMs);
    dwellGated = true;  // AUTO ON transitions honor MINIMUM_DWELL_TIME_MS
    d.reason = desired ? "auto surplus" : "auto deficit";
  } else {
    // Genuinely unreachable: s.mode is only ever OFF/MANUAL/AUTO (set via
    // setMode/cycleMode). Reaching here means s.mode holds an impossible value
    // -> latch an internal state error and force OFF this tick.
    s.faults |= FAULT_INTERNAL;
    d.faultMask = s.faults;
    d.reason = "internal state error";
    commit(s, d, false, false, nowMs);
    return d;
  }

  // Dwell gate: only AUTO ON transitions are delayed; OFF is always immediate.
  commit(s, d, desired, dwellGated, nowMs);
  return d;
}

// --- Mode / manual control (called by main from local button + remote) -------
void setMode(State& s, Mode m) {
  if (m == s.mode || m == Mode::INVALID) return;
  s.mode = m;
  s.manualOn = false;  // any mode change starts with the manual latch OFF (safe)
  resetAuto(s);        // and AUTO timers reset on entry
}

void cycleMode(State& s) {
  Mode next;
  switch (s.mode) {
    case Mode::OFF:    next = Mode::AUTO;   break;
    case Mode::AUTO:   next = Mode::MANUAL; break;
    case Mode::MANUAL: next = Mode::OFF;    break;
    default:           next = Mode::OFF;    break;
  }
  setMode(s, next);
}

void toggleManual(State& s) {
  s.manualOn = !s.manualOn;  // local long-press in MANUAL; effective only in MANUAL
}

void setThresholds(uint32_t onW, uint32_t offW) {
  // Reject any command that would break the hysteresis invariant (off < on).
  // A malformed remote command therefore cannot create a single-threshold
  // oscillation; the previous valid thresholds simply stay in effect.
  if (offW < onW) {
    g_onW = onW;
    g_offW = offW;
  }
}

void getThresholds(uint32_t& onW, uint32_t& offW) {
  onW = g_onW;
  offW = g_offW;
}

void setManualOn(State& s, bool on) {
  s.manualOn = on;
}

}  // namespace control
