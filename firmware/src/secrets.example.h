#pragma once

// =============================================================================
// secrets.example.h - TEMPLATE for connection secrets.
//
// Copy this file to "secrets.h" (same folder) and fill in your values.
// secrets.h is git-ignored so your credentials never get committed.
//
//   cp firmware/src/secrets.example.h firmware/src/secrets.h
//
// Only used when ENABLE_WIFI = 1.
// =============================================================================

#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

// Home Assistant MQTT broker (e.g. the Mosquitto add-on). Host as IP or name.
#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt-user"      // use "" for an anonymous broker
#define MQTT_PASS "mqtt-password"  // use "" for an anonymous broker

// Topic the ESP32 SUBSCRIBES to for the PV surplus (signed feed-in power, in W:
// positive = export/surplus, negative = grid import). A non-numeric payload such
// as "unavailable" is treated as a fault -> load OFF.
//
// Your HA sensor (sensor.qcells_inverter_h34c15j6s19024_feed_in_power) is NOT on
// MQTT by itself. See firmware/README.md for the small HA automation that
// re-publishes it to this topic on change AND on a fixed interval (so the
// controller's staleness watchdog stays fed).
#define MQTT_SURPLUS_TOPIC "pvctrl/surplus_w"
