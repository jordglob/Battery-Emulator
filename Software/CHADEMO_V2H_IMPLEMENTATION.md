# CHAdeMO V2H (Vehicle-to-Home) Implementation Guide

## Overview

This document describes the CHAdeMO V2H bidirectional discharge implementation for Battery-Emulator. This module enables an Electric Vehicle (EV) battery to discharge power through a CHAdeMO connector to a home inverter system (V2H - Vehicle-to-Home).

**WARNING: This involves HIGH VOLTAGE DC systems (300-450V). Improper installation or use can result in DEATH, FIRE, or EQUIPMENT DAMAGE. Only qualified personnel should install and operate this system.**

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [GPIO Wiring Guide](#gpio-wiring-guide)
4. [CHAdeMO Protocol](#chademo-protocol)
5. [State Machine](#state-machine)
6. [Configuration](#configuration)
7. [Safety Considerations](#safety-considerations)
8. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

### System Components

```
┌─────────────────┐     CHAdeMO      ┌─────────────────┐
│                 │    Connector     │                 │
│   EV Battery    │◄───────────────►│  Battery-Emulator│
│   (Vehicle)     │   CAN + Power    │    (ESP32)      │
│                 │                  │                 │
└─────────────────┘                  └────────┬────────┘
                                              │
                                              │ CAN Bus (Pylon/BYD/etc)
                                              │
                                     ┌────────▼────────┐
                                     │                 │
                                     │  Home Inverter  │
                                     │   (Deye, etc)   │
                                     │                 │
                                     └─────────────────┘
```

### Data Flow

1. **Vehicle → EVSE (this system)**
   - CAN frames: 0x100, 0x101, 0x102 (charge limits, estimates, session)
   - CAN frames: 0x200, 0x201 (V2H discharge limits/estimates)
   - GPIO: Pin 4 (j) - permission signal

2. **EVSE → Vehicle**
   - CAN frames: 0x108, 0x109 (EVSE capabilities, status)
   - CAN frames: 0x208, 0x209 (V2H discharge capabilities/estimates)
   - GPIO: Pin 2 (d1) - EVSE ready, Pin 10 (k) - charge enable

3. **EVSE → Inverter**
   - Pylon/BYD/SMA CAN protocol
   - Mapped battery parameters (voltage, current, SOC, limits)

---

## Hardware Requirements

### Essential Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| ESP32 Board | Waveshare ESP32-S3 or compatible | Main controller |
| CAN Transceiver | MCP2515 or native ESP32 CAN | CHAdeMO communication |
| Second CAN | For inverter communication | Inverter protocol |
| DC Contactors | 500V+ rated, appropriate current | Power switching |
| Precharge Relay | With 50-100Ω resistor | Soft-start for capacitors |
| Voltage Sensor | For actual voltage measurement | Safety verification |
| Current Sensor | Hall effect or shunt | Power metering |
| CHAdeMO Connector | Type 4 / CHAdeMO inlet | Vehicle interface |

### Optional but Recommended

| Component | Purpose |
|-----------|---------|
| ISA Shunt | Precision current/voltage measurement |
| Isolation Monitor | Ground fault detection |
| Emergency Stop | Physical safety disconnect |
| Fuses | Overcurrent protection |

---

## GPIO Wiring Guide

### CHAdeMO Connector Pinout

The CHAdeMO connector has these key pins for V2H operation:

| Pin | Signal | Direction | Purpose |
|-----|--------|-----------|---------|
| 1 | GND | - | Signal ground |
| 2 | d1 | EVSE → EV | EVSE ready signal |
| 4 | j | EV → EVSE | EV charge/discharge permission |
| 7 | PP | EV → EVSE | Proximity/plug detection |
| 10 | k | EVSE → EV | Charge/discharge enable |
| 8 | CAN-H | Bidirectional | CAN High |
| 9 | CAN-L | Bidirectional | CAN Low |
| DC+ | - | Power | Positive DC terminal |
| DC- | - | Power | Negative DC terminal |

### Default GPIO Mapping (Waveshare ESP32-S3)

```
┌──────────────────────────────────────────────────────────┐
│                    ESP32-S3 GPIO Mapping                  │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  CHAdeMO Pin 2 (d1)  ────────► GPIO 4  (OUTPUT)         │
│  CHAdeMO Pin 4 (j)   ◄──────── GPIO 5  (INPUT)          │
│  CHAdeMO Pin 7 (PP)  ◄──────── GPIO 6  (INPUT)          │
│  CHAdeMO Pin 10 (k)  ────────► GPIO 7  (OUTPUT)         │
│  Connector Lock      ────────► GPIO 15 (OUTPUT)         │
│  Precharge Relay     ────────► GPIO 16 (OUTPUT)         │
│  Main Contactor      ────────► GPIO 17 (OUTPUT)         │
│                                                          │
│  CAN TX (CHAdeMO)    ────────► Hardware CAN TX          │
│  CAN RX (CHAdeMO)    ◄──────── Hardware CAN RX          │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Customizing GPIO Pins

To use different GPIO pins, define them in your `platformio.ini`:

```ini
build_flags =
  -DCHADEMO_V2H_PIN_2_DEFAULT=GPIO_NUM_4
  -DCHADEMO_V2H_PIN_4_DEFAULT=GPIO_NUM_5
  -DCHADEMO_V2H_PIN_7_DEFAULT=GPIO_NUM_6
  -DCHADEMO_V2H_PIN_10_DEFAULT=GPIO_NUM_7
  -DCHADEMO_V2H_LOCK_DEFAULT=GPIO_NUM_15
  -DCHADEMO_V2H_PRECHARGE_DEFAULT=GPIO_NUM_16
  -DCHADEMO_V2H_POSITIVE_CONTACTOR_DEFAULT=GPIO_NUM_17
```

Or implement them in your hardware HAL class.

### Wiring Diagram

```
                          ┌─────────────────────┐
                          │     CHAdeMO         │
                          │     Connector       │
                          │                     │
ESP32 GPIO4 ──[10kΩ]──────┤ Pin 2 (d1)         │
                          │                     │
ESP32 GPIO5 ◄──[10kΩ]─────┤ Pin 4 (j)          │
                          │                     │
ESP32 GPIO6 ◄──[10kΩ]─────┤ Pin 7 (PP)         │
                          │                     │
ESP32 GPIO7 ──[10kΩ]──────┤ Pin 10 (k)         │
                          │                     │
                          │ Pin 8/9 (CAN-H/L)───┼──► MCP2515/ESP32 CAN
                          │                     │
                          │ DC+/DC- ────────────┼──► Power Circuit
                          └─────────────────────┘
```

**Note:** 10kΩ resistors are for signal conditioning. Actual implementation may require optocouplers for isolation.

---

## CHAdeMO Protocol

### CAN Message Summary

#### Vehicle → EVSE (Received)

| ID | Name | Interval | Purpose |
|----|------|----------|---------|
| 0x100 | Vehicle Charge Limits | 100ms | Min/max voltage, min current |
| 0x101 | Vehicle Charge Estimate | 100ms | Timing info, battery capacity |
| 0x102 | Vehicle Session | 100ms | **CRITICAL:** Voltage/current request, SOC, V2H flag |
| 0x200 | Vehicle Discharge Limits | 100ms | V2H: Max discharge current, min voltage |
| 0x201 | Vehicle Discharge Estimate | 100ms | V2H: Available energy, time estimate |
| 0x110 | Vehicle Dynamic Control | 100ms | CHAdeMO 2.0: Dynamic control support |

#### EVSE → Vehicle (Transmitted)

| ID | Name | Interval | Purpose |
|----|------|----------|---------|
| 0x108 | EVSE Capabilities | 100ms | Available voltage/current, threshold |
| 0x109 | EVSE Status | 100ms | **CRITICAL:** Setpoints, V2H compatible flag, status |
| 0x208 | EVSE Discharge Caps | 100ms | V2H: Present discharge current, limits |
| 0x209 | EVSE Discharge Estimate | 100ms | V2H: Sequence number, time estimate |
| 0x118 | EVSE Dynamic Control | 100ms | CHAdeMO 2.0: Dynamic control response |

### Critical V2H Bits

**In 0x102 (Vehicle Session) Byte 5:**
- Bit 7: `StatusVehicleDischargeCompatible` - **Vehicle supports V2H!**
- Bit 0: `StatusVehicleChargingEnabled` - Vehicle ready for power transfer

**In 0x109 (EVSE Status) Byte 4:**
- Bit 0: `discharge_compatible` - **EVSE supports V2H!**

Both bits must be set for V2H discharge to occur.

### Timing Requirements

| Requirement | Specification |
|-------------|---------------|
| CAN Transmission | Every 100ms during session |
| Init Timeout | 5 seconds for first vehicle CAN |
| Negotiation Timeout | 10 seconds max |
| Stop Sequence | Current < 5A, Voltage < 20V before disconnect |

---

## State Machine

### State Diagram

```
                    ┌─────────────┐
                    │   V2H_IDLE  │◄────────────────────┐
                    └──────┬──────┘                     │
                           │ Plug inserted              │
                           ▼                            │
                    ┌─────────────┐                     │
                    │ V2H_CONNECTED│                    │
                    └──────┬──────┘                     │
                           │ Pin 2 HIGH                 │
                           ▼                            │
                    ┌─────────────┐                     │
                    │  V2H_INIT   │ ◄── Timeout ──┐     │
                    └──────┬──────┘               │     │
                           │ Vehicle CAN rx       │     │
                           ▼                      │     │
                    ┌─────────────┐               │     │
                    │V2H_NEGOTIATE│───────────────┘     │
                    └──────┬──────┘                     │
                           │ Pin 4 (j) HIGH             │
                           ▼                            │
                    ┌─────────────┐                     │
                    │V2H_EV_ALLOW │                     │
                    └──────┬──────┘                     │
                           │ Lock connector             │
                           ▼                            │
                    ┌─────────────┐                     │
                    │V2H_EVSE_PREP│                     │
                    └──────┬──────┘                     │
                           │ Pin 10 HIGH                │
                           ▼                            │
                    ┌─────────────┐                     │
                    │V2H_EVSE_START│                    │
                    └──────┬──────┘                     │
                           │ Allow contactors           │
                           ▼                            │
                    ┌─────────────┐                     │
                    │V2H_CONTACTORS│                    │
                    └──────┬──────┘                     │
                           │ Contactors closed          │
                           ▼                            │
                    ┌─────────────┐        ┌────────┐   │
              ┌────►│V2H_POWERFLOW│───────►│V2H_STOP├───┘
              │     └─────────────┘        └────────┘
              │            │
              │            │ Error/Fault
              │            ▼
              │     ┌─────────────┐
              └─────┤ V2H_FAULT   │  (Power cycle required)
                    └─────────────┘
```

### State Descriptions

| State | Description | Entry Conditions | Exit Conditions |
|-------|-------------|------------------|-----------------|
| IDLE | Waiting for plug | System startup | Plug detected |
| CONNECTED | Plug inserted | Proximity signal | CAN communication starts |
| INIT | Awaiting vehicle | Pin 2 asserted | First vehicle CAN received |
| NEGOTIATE | Capability exchange | Vehicle CAN flowing | Vehicle grants permission |
| EV_ALLOWED | Permission granted | Pin 4 (j) HIGH | Connector locked |
| EVSE_PREPARE | Pre-power checks | Lock engaged | Vehicle ready signal |
| EVSE_START | Starting session | Insulation OK | Contactors commanded |
| CONTACTORS | Closing contactors | Start commanded | Contactors verified closed |
| POWERFLOW | Active discharge | Contactors closed | Stop requested or limits hit |
| STOP | Controlled shutdown | Normal termination | Safe state reached |
| FAULT | Error condition | Any fault detected | Power cycle only |

---

## Configuration

### Selecting CHAdeMO V2H Mode

In the Battery-Emulator web interface or configuration:

1. Select Battery Type: **"CHAdeMO V2H Discharge Mode"**
2. Select Inverter Protocol: Your inverter type (e.g., Pylon CAN, BYD CAN)

### Build Configuration

Add to your `platformio.ini` environment:

```ini
[env:your_board]
build_flags =
  ; ... other flags ...
  ; Optional: Override default GPIO pins
  -DCHADEMO_V2H_PIN_2_DEFAULT=GPIO_NUM_4
  ; Optional: Adjust power limits
  -DCHADEMO_V2H_MAX_POWER_W=10000
  -DCHADEMO_V2H_MAX_OUTPUT_CURRENT_A=32
```

### Runtime Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Max Output Voltage | 410V | Maximum voltage EVSE can handle |
| Max Output Current | 32A | Maximum discharge current |
| Max Power | 10kW | Power limit (W) |
| Min Voltage | 200V | Minimum operating voltage |
| Threshold Tolerance | 2% | Voltage threshold margin |

---

## Safety Considerations

### CRITICAL REQUIREMENTS

1. **Precharge Circuit**
   - MANDATORY before closing main contactor
   - Typical: 50-100Ω resistor with relay
   - Protects against inrush current to capacitors

2. **DC-Rated Contactors**
   - Must be rated for DC voltage (not AC contactors!)
   - Minimum 500V DC rating recommended
   - Appropriate current rating for your application

3. **Isolation Monitoring**
   - Strongly recommended for HV safety
   - Detect ground faults before they become dangerous

4. **Emergency Stop**
   - Physical e-stop that disconnects HV power
   - Accessible and clearly marked

5. **Fusing**
   - HV DC fuses rated for your system
   - Both positive and negative lines

### Fault Handling

The system will enter FAULT state (requires power cycle) on:
- CAN communication timeout (>2 seconds)
- Vehicle fault flags set
- Voltage/current out of range
- Inconsistent permission signals

### DO NOT

- Operate without proper precharge circuit
- Use AC-rated contactors for DC applications
- Bypass safety interlocks
- Operate with damaged cables or connectors
- Leave system unattended during initial testing

---

## Troubleshooting

### Common Issues

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Stuck in INIT | No vehicle CAN | Check CAN wiring, termination |
| Stuck in NEGOTIATE | V2H not supported | Verify vehicle supports V2H |
| FAULT immediately | GPIO conflict | Check pin allocations in HAL |
| No power flow | Contactors not closing | Verify contactor wiring, precharge |
| CAN errors | Wrong speed | Ensure 500kbps CAN speed |

### Debug Logging

Enable debug output via serial monitor to see state transitions and CAN traffic:

```
CHAdeMO V2H: State 2 -> 3
CHAdeMO V2H: Plug inserted
CHAdeMO V2H: State 3 -> 4
...
```

### CAN Message Verification

Use the web interface CAN log or a CAN analyzer to verify:
1. Vehicle is sending 0x100, 0x101, 0x102 (and 0x200, 0x201 for V2H)
2. EVSE is sending 0x108, 0x109 (and 0x208, 0x209 for V2H)
3. V2H compatible bits are set in both 0x102.5.7 and 0x109.4.0

---

## Appendix: CAN Frame Details

### H102 (Vehicle Session) - 0x102

| Byte | Bit | Field | Description |
|------|-----|-------|-------------|
| 0 | - | Protocol Number | CHAdeMO version |
| 1-2 | - | Target Voltage | Requested voltage (V) |
| 3 | - | Current Request | Requested current (A) |
| 4 | 0 | FaultOverVoltage | Over voltage fault |
| 4 | 1 | FaultUnderVoltage | Under voltage fault |
| 4 | 2 | FaultCurrentDev | Current deviation fault |
| 4 | 3 | FaultHighTemp | High temperature fault |
| 4 | 4 | FaultVoltageDev | Voltage deviation fault |
| 5 | 0 | ChargingEnabled | Vehicle ready for power |
| 5 | 1 | ShifterPosition | 1 = not in park |
| 5 | 2 | ChargingError | Vehicle charging error |
| 5 | 3 | VehicleStatus | General vehicle status |
| 5 | 4 | NormalStop | Normal stop requested |
| 5 | 7 | **DischargeCompat** | **V2H COMPATIBLE!** |
| 6 | - | StateOfCharge | SOC (%) |

### H109 (EVSE Status) - 0x109

| Byte | Bit | Field | Description |
|------|-----|-------|-------------|
| 0 | - | Protocol Number | CHAdeMO version |
| 1-2 | - | Setpoint Voltage | Output voltage (V) |
| 3 | - | Setpoint Current | Output current (A) |
| 4 | 0 | **DischargeCompat** | **V2H COMPATIBLE!** |
| 5 | 0 | EVSEStatus | EVSE ready |
| 5 | 1 | EVSEError | EVSE error |
| 5 | 2 | ConnectorLocked | Lock engaged |
| 5 | 3 | BatteryIncompat | Incompatible battery |
| 5 | 4 | ChgDischError | Charge/discharge error |
| 5 | 5 | ChgDischStop | Stop control (1=stop) |
| 6 | - | RemainingTime10s | Time remaining (10s) |
| 7 | - | RemainingTime1min | Time remaining (1min) |

---

## References

- CHAdeMO Protocol Specification v2.0
- IEEE 2030.1.1 Standard for DC Quick Chargers
- Battery-Emulator Documentation: https://github.com/dalathegreat/Battery-Emulator

---

*This document is part of the Battery-Emulator project. Use at your own risk.*
*Always prioritize safety when working with high voltage systems.*
