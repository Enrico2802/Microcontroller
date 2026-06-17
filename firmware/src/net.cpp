// =============================================================================
// net.cpp - WiFi + MQTT (Home Assistant) integration implementation.
//
// Entire translation unit compiles to nothing when ENABLE_WIFI == 0.
//
// Responsibilities:
//   - Non-blocking WiFi station connect + reconnect.
//   - Non-blocking MQTT connect (with Last-Will availability) + reconnect.
//   - Subscribe to the PV surplus feed and to HA command topics.
//   - Publish HA MQTT-Discovery configs so entities appear automatically.
//   - Publish periodic telemetry (one retained JSON state message).
//
// It NEVER touches the load or makes control decisions. It only exposes received
// values + remote requests; main applies them, and control enforces priority.
// =============================================================================

#include "net.h"

#if ENABLE_WIFI

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>  // strcasecmp

#include "secrets.h"

namespace net {
namespace {

WiFiClient   g_wifiClient;
PubSubClient g_mqtt(g_wifiClient);

// --- Identity + topics (built once the MAC is known) -------------------------
char g_id[8]   = "000000";  // last 3 MAC bytes, hex
char g_nodeId[24];          // "pvctrl_25f7fc" (unique id / HA device id)
char g_base[40];            // "pvctrl/25f7fc"
char g_avail[56];           // availability topic
char g_state[56];           // telemetry (JSON) topic
char g_tMode[64];
char g_tManual[64];
char g_tThOn[64];
char g_tThOff[64];
char g_tFaultReset[72];

// --- Connection timing -------------------------------------------------------
uint32_t g_lastWifiBeginMs = 0;
uint32_t g_lastMqttTryMs   = 0;

// --- PV surplus feed ---------------------------------------------------------
float    g_surplusW       = 0.0f;   // signed feed-in power (W): + export, - import
bool     g_surplusNumeric = false;  // last payload parsed to a plausible number
bool     g_surplusGotOne  = false;  // at least one valid value ever received
uint32_t g_surplusLastMs  = 0;      // millis() of the last valid value

// --- Remote requests (levels) ------------------------------------------------
control::Mode g_remoteMode  = control::Mode::OFF;
bool          g_remoteManual = false;
uint32_t      g_remoteOnW   = SURPLUS_ON_THRESHOLD_W;
uint32_t      g_remoteOffW  = SURPLUS_OFF_THRESHOLD_W;
bool          g_faultResetPending = false;

// --- Small helpers -----------------------------------------------------------
const char* modeName(control::Mode m) {
  switch (m) {
    case control::Mode::MANUAL: return "MANUAL";
    case control::Mode::AUTO:   return "AUTO";
    case control::Mode::OFF:    return "OFF";
    default:                    return "OFF";
  }
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

// Parse a full numeric payload (rejects "unavailable"/"unknown"/garbage).
bool parseFloat(const char* s, float& out) {
  char* end = nullptr;
  double v = strtod(s, &end);
  if (end == s) return false;  // no leading number
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
  if (*end != '\0') return false;  // trailing non-numeric content
  if (!isfinite(v)) return false;
  out = static_cast<float>(v);
  return true;
}

bool parseWatts(const char* s, uint32_t& out) {
  float f;
  if (!parseFloat(s, f)) return false;
  if (f < 0.0f) f = 0.0f;
  if (f > 1000000.0f) f = 1000000.0f;
  out = static_cast<uint32_t>(f + 0.5f);
  return true;
}

bool truthy(const char* s) {
  return strcasecmp(s, "ON") == 0 || strcasecmp(s, "true") == 0 || strcmp(s, "1") == 0;
}

void buildTopics() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(g_id, sizeof g_id, "%02x%02x%02x", mac[3], mac[4], mac[5]);
  snprintf(g_nodeId, sizeof g_nodeId, "%s_%s", MQTT_DEVICE_PREFIX, g_id);
  snprintf(g_base, sizeof g_base, "%s/%s", MQTT_DEVICE_PREFIX, g_id);
  snprintf(g_avail, sizeof g_avail, "%s/availability", g_base);
  snprintf(g_state, sizeof g_state, "%s/state", g_base);
  snprintf(g_tMode, sizeof g_tMode, "%s/mode/set", g_base);
  snprintf(g_tManual, sizeof g_tManual, "%s/manual/set", g_base);
  snprintf(g_tThOn, sizeof g_tThOn, "%s/threshold_on/set", g_base);
  snprintf(g_tThOff, sizeof g_tThOff, "%s/threshold_off/set", g_base);
  snprintf(g_tFaultReset, sizeof g_tFaultReset, "%s/fault_reset/set", g_base);
}

// --- Inbound message handling ------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int len) {
  char buf[96];
  unsigned int n = len < sizeof buf - 1 ? len : sizeof buf - 1;
  memcpy(buf, payload, n);
  buf[n] = '\0';

  if (strcmp(topic, MQTT_SURPLUS_TOPIC) == 0) {
    float v;
    if (parseFloat(buf, v) && v >= SURPLUS_PLAUSIBLE_MIN_W && v <= SURPLUS_PLAUSIBLE_MAX_W) {
      g_surplusW = v;
      g_surplusNumeric = true;
      g_surplusGotOne = true;
      g_surplusLastMs = millis();
    } else {
      // "unavailable"/"unknown"/out-of-range -> invalid -> control forces load OFF.
      g_surplusNumeric = false;
    }
  } else if (strcmp(topic, g_tMode) == 0) {
    if      (strcasecmp(buf, "OFF") == 0)    g_remoteMode = control::Mode::OFF;
    else if (strcasecmp(buf, "MANUAL") == 0) g_remoteMode = control::Mode::MANUAL;
    else if (strcasecmp(buf, "AUTO") == 0)   g_remoteMode = control::Mode::AUTO;
  } else if (strcmp(topic, g_tManual) == 0) {
    g_remoteManual = truthy(buf);
  } else if (strcmp(topic, g_tThOn) == 0) {
    uint32_t w;
    if (parseWatts(buf, w)) g_remoteOnW = w;
  } else if (strcmp(topic, g_tThOff) == 0) {
    uint32_t w;
    if (parseWatts(buf, w)) g_remoteOffW = w;
  } else if (strcmp(topic, g_tFaultReset) == 0) {
    g_faultResetPending = true;
  }
}

// --- HA MQTT Discovery -------------------------------------------------------
void publishConfig(const char* component, const char* objId, const char* cfg) {
  char topic[112];
  snprintf(topic, sizeof topic, "%s/%s/%s/%s/config",
           HA_DISCOVERY_PREFIX, component, g_nodeId, objId);
  g_mqtt.publish(topic, cfg, true);  // retained
}

void publishDiscovery() {
  // Common availability + device block appended to every entity config.
  char dev[256];
  snprintf(dev, sizeof dev,
           "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
           "\"dev\":{\"ids\":[\"%s\"],\"name\":\"PV Surplus Controller\","
           "\"mdl\":\"ESP32-32S\",\"mf\":\"DIY\"}",
           g_avail, g_nodeId);

  char cfg[768];

  // binary_sensor: load on/off
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Load\",\"uniq_id\":\"%s_load\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.load }}\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
    "\"dev_cla\":\"power\",%s}", g_nodeId, g_state, dev);
  publishConfig("binary_sensor", "load", cfg);

  // binary_sensor: problem (any latched fault)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Problem\",\"uniq_id\":\"%s_problem\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ 'ON' if value_json.fault|int > 0 else 'OFF' }}\","
    "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"problem\",%s}",
    g_nodeId, g_state, dev);
  publishConfig("binary_sensor", "problem", cfg);

  // sensor: temperature
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Temperature\",\"uniq_id\":\"%s_temp\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.temp_c }}\",\"unit_of_meas\":\"\xC2\xB0""C\","
    "\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\",%s}",
    g_nodeId, g_state, dev);
  publishConfig("sensor", "temp", cfg);

  // sensor: surplus used (clamped, control's view)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Surplus (used)\",\"uniq_id\":\"%s_surplus_used\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.surplus_used_w }}\",\"unit_of_meas\":\"W\","
    "\"dev_cla\":\"power\",\"stat_cla\":\"measurement\",%s}",
    g_nodeId, g_state, dev);
  publishConfig("sensor", "surplus_used", cfg);

  // sensor: fault code (hex)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Fault Code\",\"uniq_id\":\"%s_fault\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.fault_hex }}\",\"ic\":\"mdi:alert-circle\","
    "\"ent_cat\":\"diagnostic\",%s}", g_nodeId, g_state, dev);
  publishConfig("sensor", "fault", cfg);

  // sensor: uptime
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Uptime\",\"uniq_id\":\"%s_uptime\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.uptime_s }}\",\"unit_of_meas\":\"s\","
    "\"dev_cla\":\"duration\",\"stat_cla\":\"total_increasing\","
    "\"ent_cat\":\"diagnostic\",%s}", g_nodeId, g_state, dev);
  publishConfig("sensor", "uptime", cfg);

  // select: mode (OFF/MANUAL/AUTO)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Mode\",\"uniq_id\":\"%s_mode\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.mode }}\",\"cmd_t\":\"%s\","
    "\"options\":[\"OFF\",\"MANUAL\",\"AUTO\"],%s}",
    g_nodeId, g_state, g_tMode, dev);
  publishConfig("select", "mode", cfg);

  // switch: manual load request (effective only in MANUAL)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Manual Load\",\"uniq_id\":\"%s_manual\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.manual }}\",\"cmd_t\":\"%s\","
    "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",%s}",
    g_nodeId, g_state, g_tManual, dev);
  publishConfig("switch", "manual", cfg);

  // number: AUTO ON threshold
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Threshold ON\",\"uniq_id\":\"%s_th_on\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.th_on_w }}\",\"cmd_t\":\"%s\","
    "\"min\":0,\"max\":20000,\"step\":50,\"unit_of_meas\":\"W\",\"mode\":\"box\","
    "\"ent_cat\":\"config\",%s}", g_nodeId, g_state, g_tThOn, dev);
  publishConfig("number", "th_on", cfg);

  // number: AUTO OFF threshold
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Threshold OFF\",\"uniq_id\":\"%s_th_off\",\"stat_t\":\"%s\","
    "\"val_tpl\":\"{{ value_json.th_off_w }}\",\"cmd_t\":\"%s\","
    "\"min\":0,\"max\":20000,\"step\":50,\"unit_of_meas\":\"W\",\"mode\":\"box\","
    "\"ent_cat\":\"config\",%s}", g_nodeId, g_state, g_tThOff, dev);
  publishConfig("number", "th_off", cfg);

  // button: fault reset (operator acknowledge)
  snprintf(cfg, sizeof cfg,
    "{\"name\":\"Fault Reset\",\"uniq_id\":\"%s_fault_reset\",\"cmd_t\":\"%s\","
    "\"ic\":\"mdi:restart\",\"ent_cat\":\"config\",%s}",
    g_nodeId, g_tFaultReset, dev);
  publishConfig("button", "fault_reset", cfg);
}

bool mqttConnect() {
  const char* user = (MQTT_USER[0] != '\0') ? MQTT_USER : nullptr;
  const char* pass = (MQTT_PASS[0] != '\0') ? MQTT_PASS : nullptr;
  // Last Will: broker publishes "offline" (retained) to availability if we drop.
  if (!g_mqtt.connect(g_nodeId, user, pass, g_avail, 0, true, "offline")) {
    return false;
  }
  g_mqtt.publish(g_avail, "online", true);
  g_mqtt.subscribe(MQTT_SURPLUS_TOPIC);
  g_mqtt.subscribe(g_tMode);
  g_mqtt.subscribe(g_tManual);
  g_mqtt.subscribe(g_tThOn);
  g_mqtt.subscribe(g_tThOff);
  g_mqtt.subscribe(g_tFaultReset);
  publishDiscovery();
#if ENABLE_SERIAL_DEBUG
  Serial.printf("[NET] MQTT connected as %s; discovery published\n", g_nodeId);
#endif
  return true;
}

}  // namespace

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void begin() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // keep the radio responsive for an always-on controller
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("pv-controller");
  buildTopics();
  g_mqtt.setServer(MQTT_HOST, MQTT_PORT);
  g_mqtt.setBufferSize(MQTT_BUFFER_BYTES);
  g_mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  g_mqtt.setCallback(onMessage);

#if ENABLE_SERIAL_DEBUG
  // One-shot 2.4 GHz scan to confirm the target SSID is visible (the ESP32 radio
  // only sees 2.4 GHz, so this also reveals a 5 GHz-only / band-steering problem).
  int n = WiFi.scanNetworks();
  Serial.printf("[NET] scan: %d networks; target=\"%s\" (len=%u)\n",
                n, WIFI_SSID, (unsigned)strlen(WIFI_SSID));
  bool found = false;
  for (int i = 0; i < n; ++i) {
    String s = WiFi.SSID(i);
    bool hit = (s == WIFI_SSID);
    found |= hit;
    Serial.printf("  %c \"%s\" (len=%u) ch=%2d rssi=%d enc=%d\n", hit ? '*' : ' ',
                  s.c_str(), (unsigned)s.length(), WiFi.channel(i), WiFi.RSSI(i),
                  (int)WiFi.encryptionType(i));
  }
  Serial.printf("[NET] target SSID %sfound on 2.4 GHz\n", found ? "" : "NOT ");
  WiFi.scanDelete();
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  g_lastWifiBeginMs = millis();
#if ENABLE_SERIAL_DEBUG
  Serial.printf("[NET] WiFi connecting to \"%s\"\n", WIFI_SSID);
#endif
}

void loop(uint32_t nowMs) {
  static bool s_wifiWasUp = false;
  static uint32_t s_lastWifiLogMs = 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (s_wifiWasUp) {
      s_wifiWasUp = false;
#if ENABLE_SERIAL_DEBUG
      Serial.println(F("[NET] WiFi lost"));
#endif
    }
#if ENABLE_SERIAL_DEBUG
    if ((uint32_t)(nowMs - s_lastWifiLogMs) >= 3000) {
      s_lastWifiLogMs = nowMs;
      // status: 0=idle 1=no-ssid 3=connected 4=connect-failed(auth) 6=disconnected
      Serial.printf("[NET] WiFi status=%d\n", WiFi.status());
    }
#endif
    if ((uint32_t)(nowMs - g_lastWifiBeginMs) >= WIFI_RETRY_MS) {
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      g_lastWifiBeginMs = nowMs;
    }
    return;
  }

  if (!s_wifiWasUp) {
    s_wifiWasUp = true;
#if ENABLE_SERIAL_DEBUG
    Serial.print(F("[NET] WiFi connected, IP="));
    Serial.println(WiFi.localIP());
#endif
  }

  if (!g_mqtt.connected()) {
    if ((uint32_t)(nowMs - g_lastMqttTryMs) >= MQTT_RECONNECT_MS) {
      g_lastMqttTryMs = nowMs;
      if (!mqttConnect()) {
#if ENABLE_SERIAL_DEBUG
        // PubSubClient state(): 4=bad credentials, 5=unauthorized, -2=TCP/connect
        // failed (broker unreachable / wrong host:port), -4=timeout.
        Serial.printf("[NET] MQTT connect failed (state=%d) -> %s:%d\n",
                      g_mqtt.state(), MQTT_HOST, MQTT_PORT);
#endif
      }
    }
    return;
  }
  g_mqtt.loop();
}

bool connected() {
  return WiFi.status() == WL_CONNECTED && g_mqtt.connected();
}

bool linkUp() {
  return connected();
}

LinkState linkState() {
  if (WiFi.status() != WL_CONNECTED) return LinkState::OFFLINE;
  return g_mqtt.connected() ? LinkState::ONLINE : LinkState::WIFI_ONLY;
}

uint32_t surplusW() {
  if (!surplusValid()) return 0;
  float v = g_surplusW;
  if (v < 0.0f) v = 0.0f;  // grid import = no surplus
  if (v > 1000000.0f) v = 1000000.0f;
  return static_cast<uint32_t>(v + 0.5f);
}

long surplusRawW() {
  return static_cast<long>(g_surplusW + (g_surplusW >= 0.0f ? 0.5f : -0.5f));
}

bool surplusValid() {
  return g_surplusGotOne && g_surplusNumeric &&
         g_surplusW >= SURPLUS_PLAUSIBLE_MIN_W && g_surplusW <= SURPLUS_PLAUSIBLE_MAX_W;
}

uint32_t surplusLastSampleMs() {
  return g_surplusLastMs;
}

control::Mode remoteMode()       { return g_remoteMode; }
bool          remoteManual()     { return g_remoteManual; }
uint32_t      remoteThresholdOnW()  { return g_remoteOnW; }
uint32_t      remoteThresholdOffW() { return g_remoteOffW; }

bool consumeFaultReset() {
  if (g_faultResetPending) {
    g_faultResetPending = false;
    return true;
  }
  return false;
}

void publishState(const control::Decision& d, bool loadOn, long surplusRaw,
                  uint32_t surplusUsedW, float tempC,
                  temperature::Status tempStatus, uint32_t uptimeS) {
  if (!g_mqtt.connected()) return;
  uint32_t onW, offW;
  control::getThresholds(onW, offW);
  char json[420];
  snprintf(json, sizeof json,
    "{\"mode\":\"%s\",\"load\":\"%s\",\"manual\":\"%s\",\"temp_c\":%.1f,"
    "\"temp_status\":\"%s\",\"surplus_w\":%ld,\"surplus_used_w\":%lu,"
    "\"th_on_w\":%lu,\"th_off_w\":%lu,\"fault\":%u,\"fault_hex\":\"0x%02X\","
    "\"uptime_s\":%lu,\"link\":\"online\"}",
    modeName(d.activeMode), loadOn ? "ON" : "OFF", g_remoteManual ? "ON" : "OFF",
    tempC, tempStatusName(tempStatus), surplusRaw, (unsigned long)surplusUsedW,
    (unsigned long)onW, (unsigned long)offW, (unsigned)d.faultMask,
    (unsigned)d.faultMask, (unsigned long)uptimeS);
  g_mqtt.publish(g_state, json, true);  // retained: HA shows last state immediately
}

}  // namespace net

#endif  // ENABLE_WIFI
