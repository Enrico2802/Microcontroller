#pragma once

// =============================================================================
// hal.h - Hardware Abstraction Layer
//
// Owns ALL direct GPIO/ADC access so control.cpp stays hardware-agnostic.
// Contains the single GPIO26 choke-point (hal::setLoad), boot fail-safe,
// LED control, non-blocking debounced button edges, mode-input reads, and
// calibrated/oversampled/filtered analog reads for PV surplus and the NTC node.
//
// No control policy lives here. This layer is pure mechanism.
// =============================================================================

#include <Arduino.h>

namespace hal {

// Raw mode-selector input (active-low pins; "closed" == pin pulled to GND).
struct ModeRaw {
  bool aClosed;
  bool bClosed;
};

// Result of the non-blocking button debounce state machine, evaluated per tick.
struct ButtonEvent {
  bool shortPress;   // true once on RELEASE of a press shorter than FAULT_ACK_LONGPRESS_MS
  bool longPress;    // true once when a held press crosses FAULT_ACK_LONGPRESS_MS
};

enum class Led : uint8_t { Load, WiFi, Fault, Mode };

// --- Boot / init -------------------------------------------------------------

// MUST be the very first thing called in setup(): forces GPIO26 to a defined
// OFF state (pinMode OUTPUT, digitalWrite LOW) before Serial or any other init.
void initFailSafe();

// Configures Serial, LEDs, button/mode pins, and ADC. Re-asserts load OFF.
void init();

// --- Load choke-point (the ONLY writer of GPIO26) ----------------------------

// THE ONLY place digitalWrite(PIN_LOAD, ...) is ever called.
// Idempotent: writes the pin and logs only when the logical state changes.
void setLoad(bool on, const char* reason);

// Last commanded logical contactor state (software mirror).
bool loadState();

// --- LEDs --------------------------------------------------------------------

void setLed(Led led, bool on);

// Non-blocking ~1 Hz blink helper for the fault LED (returns current phase).
bool blinkPhase(uint32_t nowMs);

// --- Inputs ------------------------------------------------------------------

ModeRaw readModeInputs();

// Non-blocking debounce; call every loop with millis(). Reports edge events.
ButtonEvent pollButton(uint32_t nowMs);

// --- Analog ------------------------------------------------------------------

// Oversampled calibrated-mV read (analogReadMilliVolts), used by both PV and NTC.
uint16_t readMilliVoltsAvg(int pin);

// Pre-seed the PV EMA filter so the first control evaluate() sees a settled value.
void seedSurplusFilter();

// Sample PV input on cadence: oversample -> EMA. Call regularly.
void updateSurplus(uint32_t nowMs);

// Filtered surplus in watts (clamped to [0, full scale]).
uint32_t surplusW();

// Raw filtered PV node voltage in mV (before W mapping); used for plausibility.
uint16_t surplusMilliVolts();

// True when the latest filtered PV reading is within the plausible (non-railed) window.
bool surplusValid();

// millis() of the last successful PV sample. Used by control for staleness/
// heartbeat supervision (a wedged sample path -> stale -> FAULT_INTERNAL).
uint32_t surplusLastSampleMs();

}  // namespace hal
