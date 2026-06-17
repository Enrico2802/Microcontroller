// =============================================================================
// hal.cpp - Hardware Abstraction Layer implementation
// =============================================================================

#include "hal.h"
#include "config.h"

namespace hal {

namespace {

// --- Load choke-point state --------------------------------------------------
bool g_loadState = false;  // authoritative software mirror of the contactor command

// --- Button debounce state ---------------------------------------------------
bool     g_btnStable      = false;  // debounced "pressed" state (active-low -> pressed == LOW)
bool     g_btnLastRaw     = false;  // last raw sample
uint32_t g_btnLastChange  = 0;      // millis of last raw change
uint32_t g_btnPressStart  = 0;      // millis when the current stable press began
bool     g_btnLongFired   = false;  // long-press already reported for this hold

// --- PV surplus filter state -------------------------------------------------
float    g_pvEmaMv     = 0.0f;
bool     g_pvSeeded    = false;
uint32_t g_pvLastSampleMs = 0;

bool isPvPlausible(float mv) {
  // A near-zero reading is a legitimate 0 W surplus (see config notes), NOT a
  // fault. Only a hard floor (impossible for a healthy front-end) or the high
  // rail (shorted) are implausible.
  return mv >= static_cast<float>(PV_RAIL_FLOOR_MV) && mv <= static_cast<float>(PV_RAIL_HIGH_MV);
}

}  // namespace

// -----------------------------------------------------------------------------
// Boot / init
// -----------------------------------------------------------------------------
void initFailSafe() {
  // The very first hardware action of the whole program. Nothing may energize
  // the load before this establishes a defined OFF output.
  pinMode(PIN_LOAD, OUTPUT);
  digitalWrite(PIN_LOAD, LOW);
  g_loadState = false;
}

void init() {
#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("[BOOT] ESP32-32S PV surplus load controller v1"));
  Serial.println(F("[BOOT] GPIO26 forced OFF; initializing HAL"));
#endif

  // Re-assert load OFF defensively (initFailSafe already did it).
  digitalWrite(PIN_LOAD, LOW);
  g_loadState = false;

  pinMode(PIN_LED_LOAD, OUTPUT);
  pinMode(PIN_LED_WIFI, OUTPUT);
  pinMode(PIN_LED_FAULT, OUTPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  digitalWrite(PIN_LED_LOAD, LOW);
  digitalWrite(PIN_LED_WIFI, LOW);
  digitalWrite(PIN_LED_FAULT, LOW);
  digitalWrite(PIN_LED_MODE, LOW);

  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_MODE_A, INPUT_PULLUP);
  pinMode(PIN_MODE_B, INPUT_PULLUP);

  // ADC1 channels: 12-bit, 11dB attenuation for the ~0..3.3V range.
  // analogReadMilliVolts() applies per-chip eFuse calibration on top of this.
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PV, ADC_11db);
  analogSetPinAttenuation(PIN_NTC, ADC_11db);
}

// -----------------------------------------------------------------------------
// Load choke-point (ONLY writer of GPIO26)
// -----------------------------------------------------------------------------
void setLoad(bool on, const char* reason) {
  if (on == g_loadState) {
    return;  // idempotent: no pin write, no log when unchanged
  }
  g_loadState = on;
  digitalWrite(PIN_LOAD, on ? HIGH : LOW);
#if ENABLE_SERIAL_DEBUG
  Serial.printf("[%lu] LOAD %s (%s)\n",
                static_cast<unsigned long>(millis()),
                on ? "ON" : "OFF",
                reason ? reason : "");
#else
  (void)reason;
#endif
}

bool loadState() {
  return g_loadState;
}

// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------
void setLed(Led led, bool on) {
  int pin = PIN_LED_LOAD;
  switch (led) {
    case Led::Load:  pin = PIN_LED_LOAD;  break;
    case Led::WiFi:  pin = PIN_LED_WIFI;  break;
    case Led::Fault: pin = PIN_LED_FAULT; break;
    case Led::Mode:  pin = PIN_LED_MODE;  break;
  }
  digitalWrite(pin, on ? HIGH : LOW);
}

bool blinkPhase(uint32_t nowMs) {
  return (nowMs / 500u) % 2u == 0u;  // ~1 Hz, 50% duty
}

// -----------------------------------------------------------------------------
// Inputs
// -----------------------------------------------------------------------------
ModeRaw readModeInputs() {
  // Active-low with INPUT_PULLUP: a closed contact reads LOW.
  ModeRaw m;
  m.aClosed = (digitalRead(PIN_MODE_A) == LOW);
  m.bClosed = (digitalRead(PIN_MODE_B) == LOW);
  return m;
}

ButtonEvent pollButton(uint32_t nowMs) {
  ButtonEvent ev{false, false};

  // Raw "pressed" is active-low.
  bool raw = (digitalRead(PIN_BTN) == LOW);

  if (raw != g_btnLastRaw) {
    g_btnLastRaw = raw;
    g_btnLastChange = nowMs;
  }

  // Accept the raw level once it has been stable for the debounce window.
  if ((uint32_t)(nowMs - g_btnLastChange) >= DEBOUNCE_MS && raw != g_btnStable) {
    g_btnStable = raw;
    if (g_btnStable) {
      // Press down: start timing. The action is decided on release / long-hold,
      // so a long press does NOT also fire a short press.
      g_btnPressStart = nowMs;
      g_btnLongFired = false;
    } else if (!g_btnLongFired) {
      // Released before the long threshold -> it was a short press.
      ev.shortPress = true;
    }
  }

  // Long-press fires once while the button is still held past the threshold.
  if (g_btnStable && !g_btnLongFired &&
      (uint32_t)(nowMs - g_btnPressStart) >= FAULT_ACK_LONGPRESS_MS) {
    ev.longPress = true;
    g_btnLongFired = true;
  }

  return ev;
}

// -----------------------------------------------------------------------------
// Analog
// -----------------------------------------------------------------------------
uint16_t readMilliVoltsAvg(int pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_OVERSAMPLE; ++i) {
    sum += analogReadMilliVolts(pin);
  }
  return static_cast<uint16_t>(sum / ADC_OVERSAMPLE);
}

void seedSurplusFilter() {
  g_pvEmaMv = static_cast<float>(readMilliVoltsAvg(PIN_PV));
  g_pvSeeded = true;
  g_pvLastSampleMs = millis();  // mark the seed as a fresh sample for staleness tracking
}

void updateSurplus(uint32_t nowMs) {
  if (g_pvSeeded && (uint32_t)(nowMs - g_pvLastSampleMs) < SAMPLE_INTERVAL_MS) {
    return;
  }
  g_pvLastSampleMs = nowMs;

  float sample = static_cast<float>(readMilliVoltsAvg(PIN_PV));
  if (!g_pvSeeded) {
    g_pvEmaMv = sample;
    g_pvSeeded = true;
  } else {
    g_pvEmaMv += EMA_ALPHA * (sample - g_pvEmaMv);
  }
}

uint16_t surplusMilliVolts() {
  return static_cast<uint16_t>(g_pvEmaMv + 0.5f);
}

bool surplusValid() {
  return isPvPlausible(g_pvEmaMv);
}

uint32_t surplusLastSampleMs() {
  return g_pvLastSampleMs;
}

uint32_t surplusW() {
  // A legitimate low/zero reading clamps to 0 W (the clamp below handles it).
  float w = (g_pvEmaMv / ADC_VREF_MV) * static_cast<float>(PV_SURPLUS_FULL_SCALE_W);
  if (w < 0.0f) w = 0.0f;
  if (w > static_cast<float>(PV_SURPLUS_FULL_SCALE_W)) w = static_cast<float>(PV_SURPLUS_FULL_SCALE_W);
  return static_cast<uint32_t>(w + 0.5f);
}

}  // namespace hal
