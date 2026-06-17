#pragma once

// =============================================================================
// net.h - WiFi + MQTT (Home Assistant) integration.
//
// When ENABLE_WIFI == 0 every entry point resolves to an inline no-op / safe
// default, so main.cpp calls them unconditionally and the firmware links with
// zero networking code (pure-local ADC build).
//
// SAFETY CONTRACT (enforced in control.cpp, not here):
//   - Remote commands may only REQUEST a mode/threshold/manual change.
//   - FAULT and a forced OFF always override any remote request.
//   - A lost broker link collapses the effective mode to OFF (see main.cpp /
//     LINK_LOSS_SAFE_MS) and the surplus feed going stale forces the load OFF.
//   The network layer can never re-energize the load past a fault.
// =============================================================================

#include "config.h"
#include "control.h"  // control::Mode, control::Decision; pulls temperature::Status too

namespace net {

// Connection state, used by main to drive the WiFi status LED.
enum class LinkState : uint8_t { OFFLINE, WIFI_ONLY, ONLINE };

#if ENABLE_WIFI

// Start WiFi (station) + configure the MQTT client. Non-blocking.
void begin();

// Service WiFi/MQTT connect + reconnect + inbound messages. Call every loop().
void loop(uint32_t nowMs);

// True when the MQTT broker connection is up (WiFi associated AND MQTT session).
bool connected();

// Same as connected(); named for the link-loss safety check in main.
bool linkUp();

// Coarse link state for the status LED (OFFLINE / WIFI_ONLY / ONLINE).
LinkState linkState();

// --- PV surplus received over MQTT (mirrors the hal:: surplus accessors) -----
// surplusW(): clamped to [0, ...] for the control logic (negative feed-in =
//             grid import = no surplus = 0 W).
// surplusRawW(): signed raw feed-in power for telemetry/debug.
// surplusValid(): a numeric, plausible value has been received.
// surplusLastSampleMs(): millis() of the last valid numeric value (0 if never).
uint32_t surplusW();
long     surplusRawW();
bool     surplusValid();
uint32_t surplusLastSampleMs();

// --- Remote requests from Home Assistant (levels; main applies them) ---------
control::Mode remoteMode();           // last commanded mode (defaults OFF)
bool          remoteManual();         // last commanded MANUAL load switch state
uint32_t      remoteThresholdOnW();   // last commanded AUTO ON threshold (W)
uint32_t      remoteThresholdOffW();  // last commanded AUTO OFF threshold (W)
bool          consumeFaultReset();    // true once if a remote fault-reset was requested

// Consume-once inbound HA commands (so they do not override local changes every
// tick; main applies remote first, then local, so a local button press wins ties).
bool          takeModeCommand(control::Mode& out);  // true if a new mode command arrived
bool          takeManualCommand(bool& out);         // true if a new manual command arrived
bool          takeCmdDirty();                       // true once after ANY inbound command (state echo)

// Publish current controller state (telemetry) to HA. No-op if not connected.
void publishState(const control::Decision& d, bool loadOn, long surplusRawW,
                  uint32_t surplusUsedW, float tempC,
                  temperature::Status tempStatus, uint32_t uptimeS);

#else  // ENABLE_WIFI == 0 : inline no-op stubs / safe defaults

inline void begin() {}
inline void loop(uint32_t /*nowMs*/) {}
inline bool connected() { return false; }
inline bool linkUp() { return false; }
inline LinkState linkState() { return LinkState::OFFLINE; }
inline uint32_t surplusW() { return 0; }
inline long     surplusRawW() { return 0; }
inline bool     surplusValid() { return false; }
inline uint32_t surplusLastSampleMs() { return 0; }
inline control::Mode remoteMode() { return control::Mode::OFF; }
inline bool          remoteManual() { return false; }
inline uint32_t      remoteThresholdOnW() { return SURPLUS_ON_THRESHOLD_W; }
inline uint32_t      remoteThresholdOffW() { return SURPLUS_OFF_THRESHOLD_W; }
inline bool          consumeFaultReset() { return false; }
inline bool          takeModeCommand(control::Mode& /*out*/) { return false; }
inline bool          takeManualCommand(bool& /*out*/) { return false; }
inline bool          takeCmdDirty() { return false; }
inline void publishState(const control::Decision& /*d*/, bool /*loadOn*/,
                         long /*surplusRawW*/, uint32_t /*surplusUsedW*/,
                         float /*tempC*/, temperature::Status /*tempStatus*/,
                         uint32_t /*uptimeS*/) {}

#endif  // ENABLE_WIFI

}  // namespace net
