// =============================================================================
// main.cpp - composition root and non-blocking super-loop
//
// setup() establishes the boot fail-safe (GPIO26 OFF) BEFORE anything else,
// then initializes modules in safe order. loop() runs one sample -> evaluate
// -> actuate -> telemetry pass per iteration, fully non-blocking (millis()-
// based timing, no delay() in steady state).
//
// The load is driven ONLY through hal::setLoad() from exactly one call site
// here, fed by the Decision returned from control::evaluate().
// =============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "hal.h"
#include "temperature.h"
#include "control.h"
#include "net.h"
#include "oled.h"

namespace {

control::State g_state;

uint32_t g_lastControlMs = 0;
uint32_t g_lastStatusMs  = 0;

// Overflow-safe periodic gate: returns true and advances `last` when due.
bool due(uint32_t& last, uint32_t interval, uint32_t nowMs) {
  if ((uint32_t)(nowMs - last) >= interval) {
    last = nowMs;
    return true;
  }
  return false;
}

void updateLeds(const control::Decision& d, uint32_t nowMs) {
  hal::setLed(hal::Led::Load, hal::loadState());

  // Fault LED: blink while a fault is latched (signals "needs operator clear").
  bool faulted = (d.faultMask != control::FAULT_NONE);
  hal::setLed(hal::Led::Fault, faulted ? hal::blinkPhase(nowMs) : false);

  // WiFi LED: solid when MQTT is up, blink when WiFi-but-no-broker, off when down.
  switch (net::linkState()) {
    case net::LinkState::ONLINE:    hal::setLed(hal::Led::WiFi, true);                  break;
    case net::LinkState::WIFI_ONLY: hal::setLed(hal::Led::WiFi, hal::blinkPhase(nowMs)); break;
    case net::LinkState::OFFLINE:   hal::setLed(hal::Led::WiFi, false);                 break;
  }
}

#if ENABLE_SERIAL_DEBUG
const char* modeName(control::Mode m) {
  switch (m) {
    case control::Mode::OFF:     return "OFF";
    case control::Mode::MANUAL:  return "MANUAL";
    case control::Mode::AUTO:    return "AUTO";
    case control::Mode::INVALID: return "INVALID";
  }
  return "?";
}

const char* tempStatusName(temperature::Status s) {
  switch (s) {
    case temperature::Status::OK:           return "OK";
    case temperature::Status::OPEN:         return "OPEN";
    case temperature::Status::SHORT:        return "SHORT";
    case temperature::Status::OUT_OF_RANGE: return "RANGE";
  }
  return "?";
}

void printStatus(const control::Decision& d, uint32_t surplusW,
                 bool surplusValid, float tempC, temperature::Status tempStatus,
                 uint32_t nowMs) {
  Serial.printf(
      "[%lu] mode=%s load=%s surplus=%luW(%s) temp=%.1fC(%s) faults=0x%02X up=%lus\n",
      static_cast<unsigned long>(nowMs),
      modeName(d.activeMode),
      hal::loadState() ? "ON" : "OFF",
      static_cast<unsigned long>(surplusW),
      surplusValid ? "ok" : "BAD",
      tempC,
      tempStatusName(tempStatus),
      static_cast<unsigned>(d.faultMask),
      static_cast<unsigned long>(nowMs / 1000u));
}
#endif  // ENABLE_SERIAL_DEBUG

}  // namespace

void setup() {
  // INVARIANT #1: force GPIO26 OFF before Serial, config, sensors, anything.
  hal::initFailSafe();

  hal::init();

  // Pre-seed sensor filters so the first evaluate() sees settled values and
  // cannot read a transient that spuriously faults OR (worse) permits ON.
  hal::seedSurplusFilter();
  temperature::init();

  uint32_t now = millis();
  control::init(g_state, now);

  net::begin();
  oled::begin();

  // Arm the Task Watchdog LAST, once init is complete. If loop() ever stalls
  // longer than WATCHDOG_TIMEOUT_S the chip panics and resets; on reboot
  // hal::initFailSafe() drives GPIO26 LOW first thing, so a hang -> load OFF.
  // This is the stall-to-safe backstop: a wedged loop must never leave a
  // high-power contactor energized with no thermal/PV supervision running.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Arduino-ESP32 3.x: esp_task_wdt_init takes a config struct.
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = WATCHDOG_TIMEOUT_S * 1000u;
  wdtConfig.idle_core_mask = 0;
  wdtConfig.trigger_panic = true;
  esp_task_wdt_init(&wdtConfig);
#else
  // Arduino-ESP32 2.x: (timeout_seconds, panic_on_timeout).
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);  // subscribe the loop task (the Arduino loopTask)

#if ENABLE_SERIAL_DEBUG
  Serial.println(F("[BOOT] init complete; entering control loop"));
#endif
}

void loop() {
  uint32_t now = millis();

  // Feed the Task Watchdog every iteration. As long as loop() keeps cycling the
  // chip is healthy; a stall starves the WDT and forces a reset-to-safe.
  esp_task_wdt_reset();

  // Service optional features (no-ops in the default build).
  net::loop(now);

  // Sensors sample themselves on their own cadence (SAMPLE_INTERVAL_MS).
  hal::updateSurplus(now);
  temperature::update(now);

  // Poll the button every loop so debounce/long-press timing is accurate.
  hal::ButtonEvent btn = hal::pollButton(now);

  // Run the control state machine on the control tick cadence. The button edge
  // is latched into static accumulators so a press between ticks is not lost.
  static bool s_pressPending = false;
  static bool s_longPending = false;
  if (btn.pressedEdge) s_pressPending = true;
  if (btn.longPress)   s_longPending = true;

  static control::Decision s_lastDecision{false, control::Mode::OFF,
                                          control::FAULT_NONE, "boot"};

  if (due(g_lastControlMs, CONTROL_TICK_MS, now)) {
    control::Inputs in;

    // --- Mode source -------------------------------------------------------
#if MODE_SOURCE_REMOTE
    // HA-driven mode. A broker link lost longer than LINK_LOSS_SAFE_MS collapses
    // the effective mode to OFF: a stale remote command must not keep the load on.
    static uint32_t s_linkDownSinceMs = 0;
    if (net::linkUp()) {
      s_linkDownSinceMs = 0;
    } else if (s_linkDownSinceMs == 0) {
      s_linkDownSinceMs = now;
    }
    bool linkLost = !net::linkUp() && s_linkDownSinceMs != 0 &&
                    (uint32_t)(now - s_linkDownSinceMs) >= LINK_LOSS_SAFE_MS;
    control::Mode eff = linkLost ? control::Mode::OFF : net::remoteMode();
    // Express the resolved mode in the selector-bit convention control decodes.
    in.aClosed = (eff == control::Mode::MANUAL);
    in.bClosed = (eff == control::Mode::AUTO);
    // Apply remote tunables/commands (control validates + enforces priority).
    control::setThresholds(net::remoteThresholdOnW(), net::remoteThresholdOffW());
    control::setManualOn(g_state, net::remoteManual());
    in.buttonPressed = false;
    in.buttonLong    = net::consumeFaultReset();  // HA "Fault Reset" button acks faults
    (void)s_pressPending;
    (void)s_longPending;
#else
    hal::ModeRaw mode = hal::readModeInputs();
    in.aClosed       = mode.aClosed;
    in.bClosed       = mode.bClosed;
    in.buttonPressed = s_pressPending;
    in.buttonLong    = s_longPending;
#endif
    s_pressPending = false;
    s_longPending = false;

    // --- PV surplus source -------------------------------------------------
#if SURPLUS_SOURCE_MQTT
    in.surplusW      = net::surplusW();             // clamped >=0 (import -> 0 W)
    in.surplusValid  = net::surplusValid();
    in.pvSampleAgeMs = (uint32_t)(now - net::surplusLastSampleMs());
#else
    in.surplusW      = hal::surplusW();
    in.surplusValid  = hal::surplusValid();
    in.pvSampleAgeMs = (uint32_t)(now - hal::surplusLastSampleMs());
#endif

    // --- Temperature (always the local NTC) --------------------------------
    in.tempC           = temperature::degC();
    in.tempStatus      = temperature::status();
    in.tempSampleAgeMs = (uint32_t)(now - temperature::lastSampleMs());

    control::Decision d = control::evaluate(g_state, in, now);

    // SINGLE steady-state call site that drives the load.
    hal::setLoad(d.desiredLoad, d.reason);

    s_lastDecision = d;
    updateLeds(d, now);
    oled::render(g_state, d, in.surplusW, in.tempC);
  } else {
    // Keep LED animations (fault blink) smooth between control ticks.
    updateLeds(s_lastDecision, now);
  }

#if ENABLE_WIFI
  // Telemetry to Home Assistant on its own cadence (no-op while not connected).
  static uint32_t s_lastMqttPubMs = 0;
  if (due(s_lastMqttPubMs, MQTT_STATE_PUBLISH_MS, now)) {
#if SURPLUS_SOURCE_MQTT
    long     surplusRaw  = net::surplusRawW();
    uint32_t surplusUsed = net::surplusW();
#else
    long     surplusRaw  = (long)hal::surplusW();
    uint32_t surplusUsed = hal::surplusW();
#endif
    net::publishState(s_lastDecision, hal::loadState(), surplusRaw, surplusUsed,
                      temperature::degC(), temperature::status(), now / 1000u);
  }
#endif

#if ENABLE_SERIAL_DEBUG
  if (due(g_lastStatusMs, STATUS_INTERVAL_MS, now)) {
#if SURPLUS_SOURCE_MQTT
    uint32_t surplusDisp   = net::surplusW();      // show the source control actually uses
    bool     surplusOkDisp = net::surplusValid();
#else
    uint32_t surplusDisp   = hal::surplusW();
    bool     surplusOkDisp = hal::surplusValid();
#endif
    printStatus(s_lastDecision, surplusDisp, surplusOkDisp,
                temperature::degC(), temperature::status(), now);
  }
#endif
}
