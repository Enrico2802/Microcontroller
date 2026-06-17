# ESP32 PV Surplus Load Controller

This project describes an ESP32-based load controller that can automatically switch high-power consumers depending on the available photovoltaic surplus. The main control logic runs locally on the ESP32, so the controller remains operational even without Home Assistant, Node-RED, or an internet connection.

Home Assistant, Node-RED, and MQTT are intended as optional interfaces for visualization, monitoring, and parameter adjustment. The actual switching decision is made locally on the device.

## Project Goal

Households with a photovoltaic system often have surplus electrical energy available for limited periods of time. This prototype is intended to use that surplus by automatically activating suitable consumers, for example:

- heating elements
- other large resistive loads
- miners or comparable high-power consumers

Since these loads cannot be switched directly by a microcontroller, the ESP32 only controls a safe low-voltage switching path. The actual load is switched by an external relay, solid-state relay, or contactor.

## Key Features

- local control logic on the ESP32
- operating modes `OFF`, `MANUAL`, and `AUTO`
- physical user interface using buttons, switches, or a rotary encoder
- local status indication using LEDs and optionally an OLED display
- switching output for a relay, SSR, or contactor
- hysteresis and safety delays to prevent rapid toggling
- optional WiFi integration with Home Assistant, Node-RED, or MQTT
- manual override directly on the device
- safe fallback behavior: load off on faults or missing control signal

## System Overview

The ESP32 performs the complete local control loop:

1. Measure or receive a PV surplus value
2. Evaluate the operating mode and local user input
3. Decide whether the load should be switched based on thresholds, hysteresis, and safety delays
4. Drive a relay, SSR, or contactor through a low-voltage driver stage
5. Indicate the current state locally
6. Optionally transmit status data via WiFi

External home automation is therefore not part of the safety-critical core function. It can display values, adjust parameters, or log data, but it is not required for basic operation.

## Planned Hardware

- ESP32 development board
- buttons, selector switch, or rotary encoder for local input
- LEDs for power, WiFi status, and load state
- optional OLED display for mode, surplus value, and switching status
- driver stage using a transistor, optocoupler, or relay module
- Ailao `GSR2-1-10DA` single-phase solid-state relay with integrated heat sink for switching the contactor coil
- Heschen `CT1-25` 4-pole AC contactor for the actual load circuit
- optional sensor or data interface for PV surplus values
- optional temperature measurement on the driver stage or load

## Planned Switching Element

For the basic prototype, the ESP32 does not switch the load directly. It controls a low-voltage input stage which drives an SSR. The SSR then switches the AC coil of a larger contactor. The contactor performs the actual load switching.

Planned control chain:

```text
ESP32 GPIO26
  -> low-voltage driver stage
  -> Ailao GSR2-1-10DA SSR input
  -> SSR output switches 230 V AC contactor coil
  -> Heschen CT1-25 contactor switches the load
```

The planned intermediate switching element is an Ailao `GSR2-1-10DA` single-phase solid-state relay with an integrated heat sink.

Relevant product data:

- input/control voltage: 3-32 V DC
- output/load voltage: 24-480 V AC
- maximum load current: 10 A
- switching type: normally open solid-state relay
- intended use in this project: switching the 230 V AC coil of the contactor

The planned main switching element is a Heschen `CT1-25` household AC contactor.

Relevant product data:

- coil voltage: 220/240 V AC
- rated contact voltage/current: 220/240 V AC, 25 A
- contact type: 4NO, 4-pole
- mounting: 35 mm DIN rail
- approximate size: 82 x 65 x 35 mm
- net weight: 202 g

The SSR input can be controlled by the ESP32-side low-voltage circuit, but the ESP32 GPIO should not be assumed to drive the SSR directly without verification. For the prototype, GPIO26 should drive the SSR input through a suitable transistor or driver stage.

The SSR is used only for the contactor coil circuit. The high-power load must be switched by the contactor contacts, not by the SSR. Because the selected contactor has a 220/240 V AC coil, the SSR output side is still a mains-voltage circuit and must be wired accordingly.

## Operating Modes

| Mode | Behavior |
| --- | --- |
| `OFF` | The load remains switched off. This mode has priority over automatic control and external commands. |
| `MANUAL` | The load can be switched directly through the local user interface. |
| `AUTO` | The load is switched automatically depending on the available PV surplus. |

In automatic mode, the load should only be activated after the surplus exceeds a switch-on threshold and a safety delay has elapsed. It is deactivated when the surplus falls below a lower switch-off threshold. This hysteresis prevents rapid switching around a single threshold.

## Home Assistant Integration

The integration with Home Assistant or Node-RED is designed as an addition, not as a requirement. Possible functions include:

- displaying operating mode, load state, and surplus value
- logging switching events
- adjusting thresholds
- showing fault states
- optionally providing a simulated or externally measured surplus value

If WiFi, Home Assistant, Node-RED, or MQTT fails, the local controller should continue to operate as long as the surplus value is available locally.

## Safety Concept

This project is a proof of concept and not a certified product for mains-voltage installations.

Important boundaries:

- The ESP32 must never switch mains voltage or load current directly.
- Work on mains voltage and 400 V three-phase systems must only be carried out professionally and with suitable protective measures.
- The microcontroller only controls a galvanically isolated low-voltage switching path.
- The SSR is not a mechanical isolator. Its off-state leakage current and heat dissipation must be considered.
- The SSR switches the 230 V AC contactor coil only, not the final high-power load.
- The contactor coil circuit and load circuit must be fused, mounted, cooled, and enclosed according to the actual mains-voltage safety requirements.
- SSR leakage current must not be able to hold the contactor coil energized unintentionally.
- If the ESP32 supply fails or sensor values become invalid, the load should be switched off.
- The complete high-power electrical installation is outside the scope of this prototype concept.

## Development Stages

### Basic Prototype

The basic prototype focuses on:

- local operation
- local switching logic
- safe low-voltage control of an external switching element
- optional WiFi integration
- surplus value from an external source or simulated input

### Final Expansion Stage

The planned final version should measure and decide fully independently. For this purpose, three measurement channels are planned, comparable to a three-phase energy meter such as a Shelly Pro 3EM.

For each phase, the controller should acquire:

- voltage
- current via current transformer
- active power
- apparent power
- reactive power
- direction of power flow, meaning grid import or grid export

With these measurements, the controller can calculate the PV surplus by itself and no longer depends on a smart meter, inverter API, MQTT, or home automation system.

An optional battery or UPS supply for the low-voltage section is also planned. This allows the ESP32 to keep running during short supply interruptions, indicate fault states, and resume operation in a defined way after mains power returns.

## Verification and Test Plan

The planned verification sequence builds up the system step by step.

### 1. Low-Voltage Switching Path

- verify the ESP32 GPIO output at approximately 3.3 V
- actuate the driver stage and the `GSR2-1-10DA` SSR input
- switch the `CT1-25` contactor coil through the SSR
- verify that the contactor pulls in and drops out reliably
- monitor SSR and contactor temperature during repeated switching and steady-state operation
- verify galvanic isolation between ESP32 and AC side

### 2. Three-Phase Loads

- actuate a contactor coil through the verified switching path
- switch a three-phase resistive load
- perform repeated switching cycles under load
- verify behavior when the ESP32 supply fails

### 3. Surplus-Driven Automatic Control

- acquire an external or simulated surplus value
- verify the switch-on threshold
- verify the switch-off threshold and hysteresis
- test manual override
- detect sensor faults and switch the load off safely
- verify network-independent operation

### 4. Temperature Monitoring

During longer load tests, the temperature of the driver stage and, if applicable, the connected load should be monitored. If a configurable temperature limit is exceeded, the firmware should switch the load off.

The planned sensor for this task is an InLine temperature sensor with a 1 m cable and 2-pin header connector. It is an NTC thermistor with a nominal resistance of 10 kOhm at 25 °C and a specified temperature range of -50 °C to +90 °C.

Because the sensor is a passive NTC resistor, it must be connected to the ESP32 through a voltage divider, not directly as a digital sensor. The divider output can be read by an ADC1 input such as GPIO35. The firmware then converts the ADC voltage into a resistance value and estimates the temperature using the NTC characteristic or a calibration table.

For the prototype, the temperature monitoring should verify:

- correct ADC reading from the NTC voltage divider
- plausible temperature conversion around room temperature
- load shutdown when the configured temperature limit is exceeded
- local and optional Home Assistant fault indication after overtemperature shutdown

### Optional: Standalone Operation

Standalone operation with integrated three-phase measurement is considered an optional expansion if enough time remains. This would include:

- verifying measurement accuracy for each phase
- detecting the direction of power flow
- calculating the total three-phase surplus
- making switching decisions using only internal measurements
- testing operation without WiFi, Home Assistant, or external interfaces
- testing battery or UPS operation of the low-voltage section

## Expected Result

The final prototype should demonstrate that an ESP32 can autonomously control a high-power consumer based on available PV surplus. The device should remain locally operable, require external systems only optionally, and switch to a safe state in case of faults.

## Source

This README was derived from the concept document `Microcontroler_Esp32_englisch_final.pdf`.
