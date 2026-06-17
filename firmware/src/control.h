#pragma once

// =============================================================================
// control.h - the brain: mode/fault state machine
//
// Pure decision logic. The selected mode lives in State and changes only via
// cycleMode()/setMode(); evaluate() consumes sampled inputs (surplus, temperature,
// sample ages, fault-ack) and emits a Decision (desired load, active mode, fault
// mask). It performs NO direct GPIO/ADC access; main applies the Decision through
// the single hal::setLoad() choke-point.
//
// Strict descending priority: FAULT > OFF > MANUAL > AUTO.
// =============================================================================

#include <Arduino.h>
#include "temperature.h"

namespace control {

enum class Mode : uint8_t { OFF, MANUAL, AUTO, INVALID };

// FaultCode is a bitmask so multiple faults can latch simultaneously and be
// reported individually over serial.
enum Fault : uint16_t {
  FAULT_NONE             = 0,
  FAULT_PV_INVALID       = 1u << 0,  // PV reading railed / out of plausible window
  FAULT_NTC_OPEN         = 1u << 1,  // NTC open circuit (Rntc -> inf)
  FAULT_NTC_SHORT        = 1u << 2,  // NTC short circuit (Rntc -> 0)
  FAULT_NTC_RANGE        = 1u << 3,  // NTC reading out of valid temperature range
  FAULT_OVERTEMP         = 1u << 4,  // temperature over limit
  FAULT_MODE_SELECTOR    = 1u << 5,  // reserved (legacy hardware selector; unused with cycle-button UI)
  FAULT_SENSOR_STALE     = 1u << 6,  // a sensor sample path stopped updating (heartbeat lost)
  FAULT_INTERNAL         = 1u << 7,  // self-consistency / invariant violation
};

// Inputs gathered by main once per control tick.
struct Inputs {
  bool                buttonLong;    // fault-acknowledge request this tick (long-press / HA reset)
  uint32_t            surplusW;      // filtered PV surplus in watts
  bool                surplusValid;  // PV reading within plausible window
  float               tempC;         // filtered NTC temperature
  temperature::Status tempStatus;    // NTC health classification
  uint32_t            pvSampleAgeMs;   // age of the last PV sample
  uint32_t            tempSampleAgeMs; // age of the last NTC sample
};

// Output produced by evaluate(); main applies desiredLoad via hal::setLoad().
struct Decision {
  bool        desiredLoad;
  Mode        activeMode;
  uint16_t    faultMask;
  const char* reason;  // human-readable cause for the load command (for logging)
};

// Persistent controller state, owned by main and passed to evaluate() each tick.
struct State {
  Mode     mode;
  bool     manualOn;             // MANUAL latch toggled by the button
  uint16_t faults;               // latched fault bitmask
  uint32_t lastLoadChangeMs;     // for minimum-dwell enforcement (ON transitions)
  uint32_t onCandidateSinceMs;   // AUTO: when surplus first crossed above ON threshold
  uint32_t offCandidateSinceMs;  // AUTO: when surplus first crossed below OFF threshold
  bool     onPending;            // AUTO: ON delay currently counting
  bool     offPending;           // AUTO: OFF delay currently counting
  bool     autoLoadOn;           // AUTO: committed sub-state (load on/off within AUTO)
  bool     committedLoad;        // last load level this controller actually committed
  uint32_t healthySinceMs;       // when ALL fault conditions last became healthy
  bool     wasHealthy;           // tracks healthy-edge for the recovery hold timer
};

// Initialize to the safe default (OFF, no faults, load off).
void init(State& s, uint32_t nowMs);

// Master tick: applies fault detection + latch + recovery, then strict priority.
Decision evaluate(State& s, const Inputs& in, uint32_t nowMs);

// --- Mode + manual control (set by main from the local button AND remote) ----
// cycleMode advances OFF -> AUTO -> MANUAL -> OFF (local short-press). setMode
// sets an explicit mode (remote command). Both reset the manual latch and AUTO
// timers on entry. toggleManual flips the MANUAL load latch (local long-press);
// setManualOn sets it (remote switch). FAULT and OFF always win in evaluate().
void cycleMode(State& s);
void setMode(State& s, Mode m);
void toggleManual(State& s);

// --- Runtime-adjustable AUTO thresholds (e.g. from Home Assistant) -----------
// Seeded from config defaults. setThresholds is validated and IGNORED unless
// offW < onW (hysteresis must hold), so a bad remote command can never invert
// the hysteresis. getThresholds reports the values in effect (for telemetry).
void setThresholds(uint32_t onW, uint32_t offW);
void getThresholds(uint32_t& onW, uint32_t& offW);

// Directly set the MANUAL load latch (used by a remote MANUAL switch). Takes
// effect only while the active mode is MANUAL; FAULT and OFF still force the
// load off with higher priority.
void setManualOn(State& s, bool on);

}  // namespace control
