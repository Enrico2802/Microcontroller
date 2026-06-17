#pragma once

// =============================================================================
// config.h - single source of truth for the ESP32-32S PV surplus load controller
//
// Contains ONLY constants: pin map, feature flags, control tunables, ADC/sensor
// tunables, and fault policy. No logic lives here. Changing v1 behavior should
// require editing only this file.
//
// HARDWARE BOUNDARY: GPIO26 drives a low-voltage SSR-input driver stage ONLY.
// The ESP32 never switches mains. Chain: GPIO26 -> LV driver -> Ailao GSR2-1-10DA
// SSR input (3-32V DC) -> SSR output switches 230V coil of Heschen CT1-25
// contactor -> contactor switches the load.
// =============================================================================

#include <Arduino.h>

// -----------------------------------------------------------------------------
// FEATURE FLAGS (v1 must compile and run with all optional features OFF)
// -----------------------------------------------------------------------------
// Guarded so a build-time override (e.g. `-D ENABLE_OLED=1`) wins instead of
// being clobbered by an unconditional #define (which would also warn).
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1          // WiFi + MQTT (Home Assistant). Set 0 (or -DENABLE_WIFI=0) for a pure-local ADC build.
#endif
#ifndef ENABLE_OLED
#define ENABLE_OLED 0          // Optional: I2C OLED display seam. Read-only, never affects control.
#endif
#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 1  // Periodic serial telemetry + transition logging.
#endif

// -----------------------------------------------------------------------------
// PIN MAP (exact, see project spec). Never use GPIO6-11. GPIO34-39 input-only.
// -----------------------------------------------------------------------------
constexpr int PIN_LOAD      = 26;  // SSR/contactor driver output (digital out, default LOW/OFF at boot)
constexpr int PIN_BTN       = 25;  // Mode/multifunction button: short=cycle mode, long=manual toggle / fault ack
constexpr int PIN_MODE_A    = 32;  // Reserved (free GPIO; e.g. a future dedicated manual button)
constexpr int PIN_MODE_B    = 33;  // Reserved (free GPIO)
constexpr int PIN_PV        = 34;  // PV surplus analog input (ADC1, input-only, no pull-up)
constexpr int PIN_NTC       = 35;  // NTC temperature analog input (ADC1, input-only, no pull-up)
constexpr int PIN_OLED_SDA  = 21;  // OLED SDA (only used if ENABLE_OLED)
constexpr int PIN_OLED_SCL  = 22;  // OLED SCL (only used if ENABLE_OLED)
constexpr int PIN_LED_LOAD  = 16;  // Load status LED (digital out)
constexpr int PIN_LED_WIFI  = 17;  // WiFi status LED (digital out)
constexpr int PIN_LED_FAULT = 18;  // Fault status LED (digital out)
constexpr int PIN_LED_MODE  = 27;  // Mode LED: off=OFF, blink=AUTO, solid=MANUAL (digital out)

// -----------------------------------------------------------------------------
// CONTROL TUNABLES (AUTO mode hysteresis / delays / dwell, overtemp limit)
// -----------------------------------------------------------------------------
constexpr uint32_t SURPLUS_ON_THRESHOLD_W  = 1200;   // Switch ON above this surplus (W)
constexpr uint32_t SURPLUS_OFF_THRESHOLD_W = 800;    // Switch OFF below this surplus (W)
constexpr uint32_t SWITCH_ON_DELAY_MS      = 10000;  // Surplus must hold > ON threshold this long
constexpr uint32_t SWITCH_OFF_DELAY_MS     = 5000;   // Surplus must hold < OFF threshold this long
constexpr uint32_t MINIMUM_DWELL_TIME_MS   = 30000;  // Min time between load ON transitions (anti-cycling)
constexpr float    TEMPERATURE_LIMIT_C     = 70.0f;  // Overtemperature fault threshold (C)
constexpr float    OVERTEMP_CLEAR_MARGIN_C = 5.0f;   // Overtemp recovery hysteresis (clears below limit-margin)

// -----------------------------------------------------------------------------
// PV SURPLUS SCALING (GPIO34): 0..ADC_VREF_MV maps linearly to 0..full-scale W
// -----------------------------------------------------------------------------
constexpr uint32_t PV_SURPLUS_FULL_SCALE_W = 3680;   // Watts at full-scale (230V * 16A reference)
constexpr float    ADC_VREF_MV             = 3300.0f; // ADC/divider full-scale in mV (nominal; see README)
// IMPORTANT: a genuine 0 W surplus (night / clouds / no export) is a NORMAL
// operating point and maps to ~0 mV. It must NOT be treated as a fault, or an
// unattended AUTO controller would latch off every evening. We therefore only
// fault on a *hard* low floor (well below any legitimate reading, e.g. a
// negative-biased / broken front-end) and on the high rail (shorted). Normal
// low readings between the floor and the high rail are clamped to 0 W.
constexpr uint16_t PV_RAIL_FLOOR_MV        = 2;      // Below this: hardware fault (impossible legit reading) -> PV invalid
constexpr uint16_t PV_RAIL_HIGH_MV         = 3270;   // Above this: railed high / shorted -> PV invalid

// -----------------------------------------------------------------------------
// NTC TUNABLES (GPIO35): divider 3V3 -- NTC -- node(GPIO35) -- Rfixed -- GND
// So V_node RISES as temperature rises (Rntc falls). HIGH node = hot, LOW = cold/open.
// Rntc = Rfixed * (Vsupply/V_node - 1);  1/T = 1/T0 + (1/Beta)*ln(Rntc/R0)
// -----------------------------------------------------------------------------
constexpr float    NTC_R_FIXED    = 10000.0f;  // Fixed divider resistor (ohms), node--Rfixed--GND
constexpr float    NTC_BETA       = 3950.0f;   // NTC Beta coefficient (B25/50)
constexpr float    NTC_R0         = 10000.0f;  // NTC nominal resistance at T0 (ohms)
constexpr float    NTC_T0_K       = 298.15f;   // Reference temperature (25 C) in Kelvin
constexpr float    NTC_VSUPPLY_MV = 3300.0f;   // Divider supply rail (mV); see README accuracy note
constexpr uint16_t NTC_OPEN_MV    = 100;       // V_node <= this => Rntc->inf => OPEN (broken/unplugged)
constexpr uint16_t NTC_SHORT_MV   = 3200;      // V_node >= this => Rntc->0   => SHORT
constexpr float    NTC_TEMP_MIN_C = -50.0f;    // Sensor valid range floor (else OUT_OF_RANGE)
constexpr float    NTC_TEMP_MAX_C = 90.0f;     // Sensor valid range ceiling

// -----------------------------------------------------------------------------
// ADC / SAMPLING / FILTER TUNABLES
// -----------------------------------------------------------------------------
constexpr uint8_t  ADC_OVERSAMPLE       = 16;   // Calibrated-mV reads averaged per measurement
constexpr float    EMA_ALPHA            = 0.20f; // Exponential moving-average smoothing factor
constexpr uint32_t SAMPLE_INTERVAL_MS   = 50;   // Sensor sampling cadence (decoupled from control tick)
constexpr uint32_t CONTROL_TICK_MS      = 100;  // State machine evaluation cadence
constexpr uint32_t STATUS_INTERVAL_MS   = 2000; // Periodic serial telemetry cadence

// -----------------------------------------------------------------------------
// BUTTON / FAULT POLICY TUNABLES
// -----------------------------------------------------------------------------
constexpr uint32_t DEBOUNCE_MS              = 30;    // Non-blocking button debounce window
constexpr uint32_t FAULT_ACK_LONGPRESS_MS  = 3000;  // Button hold to acknowledge/clear latched faults
constexpr uint32_t FAULT_RECOVERY_HOLD_MS  = 5000;  // Conditions must read healthy this long before clear is allowed

// -----------------------------------------------------------------------------
// SUPERVISION TUNABLES (sensor freshness + hardware watchdog)
// -----------------------------------------------------------------------------
// If a sensor stops producing fresh samples (sample path wedged, ADC stuck on a
// plausible mid-scale value, update() no longer called, or the MQTT surplus feed
// stops arriving), raise FAULT_SENSOR_STALE -> load OFF. The temperature path is
// local and sampled fast, so its window is tight. The PV path may come from the
// network (HA over MQTT), which updates far more slowly, so it gets its own,
// much wider window (see PV_STALE_MS below).
constexpr uint32_t TEMP_STALE_MS = SAMPLE_INTERVAL_MS * 10;  // 500 ms with defaults (local NTC)

#if ENABLE_WIFI
// PV surplus is delivered by Home Assistant over MQTT. Tolerate several missed
// updates before declaring the feed dead (a steady surplus may only be published
// every ~10 s). If no fresh value arrives within this window -> stale -> load OFF.
constexpr uint32_t PV_STALE_MS = 90000;   // 90 s
#else
// PV surplus is the local ADC on GPIO34, sampled fast.
constexpr uint32_t PV_STALE_MS = SAMPLE_INTERVAL_MS * 10;  // 500 ms
#endif

// ESP32 Task Watchdog: if loop() stalls longer than this, the chip panics and
// resets; the boot fail-safe then drives GPIO26 LOW, so a hang -> load OFF.
// Comfortably larger than worst-case loop time, far below any thermal interval.
constexpr uint32_t WATCHDOG_TIMEOUT_S = 5;

// -----------------------------------------------------------------------------
// PV SURPLUS SOURCE. With WiFi enabled the surplus comes from Home Assistant
// (MQTT). With WiFi disabled it is the local ADC on GPIO34 (pure standalone).
// The MODE is NOT compile-time bound to a source: the local button AND Home
// Assistant can both set it at runtime (last change wins) -- see main.cpp.
// SAFETY: regardless of source, FAULT and a forced OFF always win in control.cpp.
// -----------------------------------------------------------------------------
#if ENABLE_WIFI
#define SURPLUS_SOURCE_MQTT 1   // surplus value arrives from HA over MQTT
#else
#define SURPLUS_SOURCE_MQTT 0   // surplus value read from the local ADC (GPIO34)
#endif

// Optional: when the MQTT surplus feed goes stale, fall back to the local ADC
// (GPIO34) instead of faulting to OFF. Default OFF -- only enable once a real
// local surplus sensor (or test poti) is wired to GPIO34, otherwise a floating
// pin would feed garbage to AUTO. (Issues.txt #5.)
#ifndef ENABLE_LOCAL_SURPLUS_FALLBACK
#define ENABLE_LOCAL_SURPLUS_FALLBACK 0
#endif

// -----------------------------------------------------------------------------
// NETWORKING (MQTT / Home Assistant). Connection SECRETS (SSID, broker, topic)
// live in secrets.h (git-ignored). These are non-secret behavior tunables.
// -----------------------------------------------------------------------------
#if ENABLE_WIFI
#define HA_DISCOVERY_PREFIX "homeassistant"  // HA MQTT discovery base topic (default)
#define MQTT_DEVICE_PREFIX  "pvctrl"         // base for this device's own topics

constexpr uint32_t WIFI_RETRY_MS         = 15000;  // re-issue WiFi.begin() if not connected this long
constexpr uint32_t MQTT_RECONNECT_MS     = 5000;   // broker reconnect attempt cadence
constexpr uint16_t MQTT_KEEPALIVE_S      = 15;     // MQTT keepalive (broker marks us dead at ~1.5x)
constexpr uint32_t MQTT_STATE_PUBLISH_MS = 5000;   // telemetry publish cadence
constexpr uint16_t MQTT_BUFFER_BYTES     = 1024;   // must fit the largest discovery payload

// Plausibility window for the received feed-in power (W). The sensor is signed:
// positive = PV export/surplus, negative = grid import. Values outside this band
// (or non-numeric like "unavailable") are treated as invalid -> load OFF.
constexpr float SURPLUS_PLAUSIBLE_MIN_W = -100000.0f;
constexpr float SURPLUS_PLAUSIBLE_MAX_W =  100000.0f;
#endif  // ENABLE_WIFI

// -----------------------------------------------------------------------------
// Compile-time invariants. Catch mis-tuned config before it can ship.
// -----------------------------------------------------------------------------
static_assert(SURPLUS_OFF_THRESHOLD_W < SURPLUS_ON_THRESHOLD_W,
              "Hysteresis requires SURPLUS_OFF_THRESHOLD_W < SURPLUS_ON_THRESHOLD_W");
static_assert(SWITCH_ON_DELAY_MS > 0 && SWITCH_OFF_DELAY_MS > 0,
              "Switch delays must be non-zero");
static_assert(PV_RAIL_FLOOR_MV < PV_RAIL_HIGH_MV, "PV rail window invalid");
static_assert(NTC_OPEN_MV < NTC_SHORT_MV, "NTC open/short thresholds inverted");
static_assert(EMA_ALPHA > 0.0f && EMA_ALPHA <= 1.0f, "EMA_ALPHA must be in (0,1]");
