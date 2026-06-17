#pragma once

// =============================================================================
// temperature.h - NTC signal chain
//
// Samples the NTC divider node (GPIO35 via hal), converts V -> R -> degC using
// the Beta equation, smooths with an EMA, and classifies sensor health
// (OK / OPEN / SHORT / OUT_OF_RANGE). It reports status + temperature only;
// it does NOT decide faults or touch the load. That policy lives in control.cpp.
// =============================================================================

#include <Arduino.h>

namespace temperature {

enum class Status : uint8_t {
  OK,            // valid reading within sensor range
  OPEN,          // node voltage ~0 -> Rntc -> infinity (broken/unplugged)
  SHORT,         // node voltage ~Vsupply -> Rntc -> 0 (shorted)
  OUT_OF_RANGE   // computed temperature outside the configured sensor range
};

// Pre-seed the EMA so the first control evaluate() sees a settled value.
void init();

// Sample on cadence (SAMPLE_INTERVAL_MS): read node mV -> classify -> convert -> EMA.
void update(uint32_t nowMs);

// Cached classification of the last update().
Status status();

// Last filtered temperature in Celsius (only meaningful when status() == OK).
float degC();

// millis() of the last sample taken by update(). Used by control for staleness/
// heartbeat supervision (a wedged sample path -> stale -> FAULT_INTERNAL).
uint32_t lastSampleMs();

}  // namespace temperature
