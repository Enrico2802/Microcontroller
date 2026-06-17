// =============================================================================
// temperature.cpp - NTC signal chain implementation
//
// Divider topology: 3V3 -- NTC -- node(GPIO35) -- Rfixed -- GND
//   V_node = Vsupply * Rfixed / (Rntc + Rfixed)
//   Rntc   = Rfixed * (Vsupply / V_node - 1)
// As temperature rises, Rntc falls, so V_node RISES (HIGH = hot, LOW = cold/open).
// =============================================================================

#include "temperature.h"
#include "config.h"
#include "hal.h"

#include <math.h>

namespace temperature {

namespace {

Status   g_status      = Status::OPEN;  // default-deny: assume worst until a good read
float    g_tempC       = 0.0f;
float    g_emaC        = 0.0f;
bool     g_seeded      = false;
uint32_t g_lastSampleMs = 0;

// Convert a node voltage (mV) to temperature in Celsius via the Beta equation.
// Caller must have already excluded open/short, so mv is well inside the rails.
float mvToCelsius(float mv) {
  // Rntc from the divider. mv is guaranteed > 0 here (open already excluded).
  float rntc = NTC_R_FIXED * (NTC_VSUPPLY_MV / mv - 1.0f);
  if (rntc <= 0.0f) {
    return NAN;  // non-physical; treated as OUT_OF_RANGE by caller
  }
  // 1/T = 1/T0 + (1/Beta) * ln(Rntc / R0)
  float invT = 1.0f / NTC_T0_K + (1.0f / NTC_BETA) * logf(rntc / NTC_R0);
  float kelvin = 1.0f / invT;
  return kelvin - 273.15f;
}

}  // namespace

void init() {
  // Seed the filter from a few raw classifications so the first evaluate() is settled.
  g_lastSampleMs = millis();  // a sample WAS taken; keep the freshness clock honest
  uint16_t mv = hal::readMilliVoltsAvg(PIN_NTC);
  if (mv > NTC_OPEN_MV && mv < NTC_SHORT_MV) {
    float c = mvToCelsius(static_cast<float>(mv));
    if (isfinite(c)) {
      g_emaC = c;
      g_tempC = c;
      g_seeded = true;
      g_status = Status::OK;
      return;
    }
  }
  // Sensor not healthy at boot: leave unseeded; update() will retry and classify.
  g_seeded = false;
  g_status = (mv <= NTC_OPEN_MV) ? Status::OPEN
           : (mv >= NTC_SHORT_MV) ? Status::SHORT
           : Status::OUT_OF_RANGE;
}

void update(uint32_t nowMs) {
  if (g_seeded && (uint32_t)(nowMs - g_lastSampleMs) < SAMPLE_INTERVAL_MS) {
    return;
  }
  g_lastSampleMs = nowMs;

  uint16_t mv = hal::readMilliVoltsAvg(PIN_NTC);

  // Guard rails BEFORE any math to avoid divide-by-zero / log-of-negative.
  if (mv <= NTC_OPEN_MV) {
    g_status = Status::OPEN;
    return;
  }
  if (mv >= NTC_SHORT_MV) {
    g_status = Status::SHORT;
    return;
  }

  float c = mvToCelsius(static_cast<float>(mv));
  if (!isfinite(c) || c < NTC_TEMP_MIN_C || c > NTC_TEMP_MAX_C) {
    g_status = Status::OUT_OF_RANGE;
    return;
  }

  // Healthy sample: smooth and report.
  if (!g_seeded) {
    g_emaC = c;
    g_seeded = true;
  } else {
    g_emaC += EMA_ALPHA * (c - g_emaC);
  }
  g_tempC = g_emaC;
  g_status = Status::OK;
}

Status status() {
  return g_status;
}

float degC() {
  return g_tempC;
}

uint32_t lastSampleMs() {
  return g_lastSampleMs;
}

}  // namespace temperature
