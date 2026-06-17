#pragma once

// =============================================================================
// oled.h - OPTIONAL I2C OLED display seam (v1 STUB)
//
// Compiled out by default (ENABLE_OLED 0). When disabled, these resolve to
// inline no-ops. The OLED is strictly READ-ONLY with respect to control: it
// renders state but never influences the load decision.
//
// v1 does NOT implement an OLED driver; the seam keeps main.cpp clean for a
// later version (would use PIN_OLED_SDA / PIN_OLED_SCL).
// =============================================================================

#include "config.h"
#include "control.h"

namespace oled {

#if ENABLE_OLED

void begin();
void render(const control::State& s, const control::Decision& d,
            uint32_t surplusW, float tempC);

#else  // ENABLE_OLED == 0 : inline no-op stubs

inline void begin() {}
inline void render(const control::State& /*s*/, const control::Decision& /*d*/,
                   uint32_t /*surplusW*/, float /*tempC*/) {}

#endif  // ENABLE_OLED

}  // namespace oled
