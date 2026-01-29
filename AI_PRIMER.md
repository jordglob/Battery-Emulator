# AI Assistant Primer - Kraftkranen/Battery-Emulator

This document provides context for AI assistants working on this codebase.

## Project Overview

**Kraftkranen v2.0** is a CHAdeMO V2H (Vehicle-to-Home) controller built on top of [dalathegreat's Battery-Emulator](https://github.com/dalathegreat/Battery-Emulator).

**Purpose:** Extract DC power from a CHAdeMO vehicle (e.g., Nissan Leaf) and feed it to a Deye inverter for home use.

**Approach:** "Man-in-the-Middle" - the ESP32 acts as a CHAdeMO EVSE (charger) that negotiates V2H discharge with the vehicle.

## Repository Structure

```
Battery-Emulator/
├── Software/
│   ├── Software.cpp              # Main entry point
│   ├── src/
│   │   ├── battery/              # Battery protocol implementations
│   │   │   ├── Battery.h         # Base class, BatteryType enum
│   │   │   ├── CanBattery.h      # CAN battery base class
│   │   │   ├── BATTERIES.h/cpp   # Factory and includes
│   │   │   ├── CHADEMO-V2H-BATTERY.h/cpp  # ★ KRAFTKRANEN V2H
│   │   │   └── CHADEMO-BATTERY.h/cpp      # Original CHAdeMO (charging)
│   │   │
│   │   ├── inverter/             # Inverter protocol implementations
│   │   │   ├── INVERTERS.h/cpp   # Factory
│   │   │   └── PYLON-CAN.cpp     # Pylontech protocol (Deye compatible)
│   │   │
│   │   ├── devboard/
│   │   │   ├── hal/              # Hardware Abstraction Layer
│   │   │   │   ├── hal.h/cpp     # Base HAL class
│   │   │   │   └── hw_6ch_relay_s3.h  # ★ KRAFTKRANEN HAL
│   │   │   ├── webserver/        # Web UI
│   │   │   └── utils/            # Events, logging
│   │   │
│   │   ├── communication/
│   │   │   ├── can/              # CAN bus handling
│   │   │   └── contactorcontrol/ # Generic contactor control
│   │   │
│   │   └── datalayer/            # Shared data structures
│   │       ├── datalayer.h       # Main datalayer
│   │       └── datalayer_extended.h  # CHAdeMO extensions
│   │
│   └── CHADEMO_V2H_IMPLEMENTATION.md  # Protocol documentation
│
├── platformio.ini                # Build configuration
├── build.bat                     # Windows build script
├── KRAFTKRANEN_README.md         # Hardware documentation
├── VERSIONS.md                   # Changelog
└── AI_PRIMER.md                  # This file
```

## Key Files for Kraftkranen

### 1. HAL: `hw_6ch_relay_s3.h`
Defines GPIO mappings for Waveshare ESP32-S3-Relay-6CH:
- Relay 1-3: Nissan Junction Box contactors
- Relay 4-5: CHAdeMO signals
- GPIO 9: Vehicle Enable input
- GPIO 10: Current sensor ADC

### 2. V2H Protocol: `CHADEMO-V2H-BATTERY.cpp`
State machine for CHAdeMO V2H discharge:
- Direct relay control (not via comm_contactorcontrol.cpp)
- Kraftkranen precharge sequence
- ACS758 current sensor reading

### 3. Build: `platformio.ini`
Environment: `6ch_relay_s3`
- Board: esp32s3_flash_16MB
- Define: `HW_6CH_RELAY_S3`

## Architecture Patterns

### Battery Factory Pattern
```cpp
// Battery.h - Enum
enum class BatteryType {
  ChademoV2H = 49,  // Kraftkranen
  // ... others
};

// BATTERIES.cpp - Factory
Battery* create_battery(BatteryType type) {
  switch (type) {
    case BatteryType::ChademoV2H:
      return new ChademoV2HBattery();
  }
}
```

### HAL Pattern
```cpp
// hal.h - Base class with virtual methods
class Esp32Hal {
  virtual gpio_num_t POSITIVE_CONTACTOR_PIN() { return GPIO_NUM_NC; }
  // ...
};

// hw_6ch_relay_s3.h - Derived class
class Relay6chS3Hal : public Esp32Hal {
  gpio_num_t POSITIVE_CONTACTOR_PIN() override { return GPIO_NUM_4; }
};

// Usage
gpio_num_t pin = esp32hal->POSITIVE_CONTACTOR_PIN();
```

### Datalayer
Shared data structure between battery and inverter:
```cpp
datalayer.battery.status.voltage_dV      // Battery voltage in decivolts
datalayer.battery.status.current_dA      // Current in deciamps
datalayer.battery.status.real_soc        // SOC in 0.01% (pptt)
datalayer.system.status.contactors_engaged  // 0=open, 1=closed, 2=error, 3=precharging
```

## Kraftkranen-Specific Logic

### Relay Control
Kraftkranen controls relays **directly**, not via `comm_contactorcontrol.cpp`:
```cpp
// In CHADEMO-V2H-BATTERY.cpp
digitalWrite(relay_main_pos, HIGH);   // Relay 1
digitalWrite(relay_main_neg, HIGH);   // Relay 2
digitalWrite(relay_precharge, HIGH);  // Relay 3
digitalWrite(relay_d1, HIGH);         // Relay 4
digitalWrite(relay_lock, HIGH);       // Relay 5
```

### Precharge Sequence
```
1. Lock + d1 signal
2. Wait for vehicle CAN + Pin 4 HIGH
3. Main- + Precharge
4. Wait 3 seconds
5. Main+ on, Precharge off
```

### Current Sensing
ACS758 Hall sensor on ADC:
```cpp
uint16_t adcValue = analogRead(pin_current_sensor);
uint32_t sensorMv = (adcValue * 3300) / 4095;
int16_t currentA = (sensorMv - 2500) / 40;  // 40mV/A sensitivity
```

## Common Tasks

### Adding a New Feature
1. Check if it affects the state machine in `CHADEMO-V2H-BATTERY.cpp`
2. Update HAL if new GPIO needed in `hw_6ch_relay_s3.h`
3. Add to datalayer if data needs to be shared
4. Update `VERSIONS.md`

### Debugging
- Serial logging via `logging.println()` / `logging.printf()`
- Web UI at device IP address
- CAN messages logged when verbose

### Building
```bash
.\build.bat 6ch_relay_s3  # Windows
platformio run -e 6ch_relay_s3  # Cross-platform
```

## CHAdeMO Protocol Quick Reference

### CAN Messages (500kbps)
| ID | Direction | Description |
|----|-----------|-------------|
| 0x100 | Vehicle → EVSE | Charge limits |
| 0x101 | Vehicle → EVSE | Charge estimate |
| 0x102 | Vehicle → EVSE | Session status (includes V2H compatibility bit) |
| 0x108 | EVSE → Vehicle | EVSE capabilities |
| 0x109 | EVSE → Vehicle | EVSE status |
| 0x200 | Vehicle → EVSE | Discharge limits (V2H) |
| 0x201 | Vehicle → EVSE | Discharge estimate (V2H) |
| 0x208 | EVSE → Vehicle | Discharge parameters (V2H) |
| 0x209 | EVSE → Vehicle | Discharge status (V2H) |

### V2H Compatibility
- EVSE advertises discharge support in 0x109 byte 4 bit 0
- Vehicle confirms in 0x102 byte 5 bit 7

## Gotchas

1. **Pin names changed:** Old code used `pin_d1`, `pin_k`, etc. Kraftkranen uses `relay_d1`, `relay_main_pos`, etc.

2. **No Pin 10 (k):** Kraftkranen doesn't use CHAdeMO Pin 10 - only Pin 2 (d1) via Relay 4.

3. **Direct relay control:** Don't rely on `comm_contactorcontrol.cpp` for Kraftkranen - V2H code controls relays directly.

4. **Windows build:** Use `build.bat` to avoid MSys/Git Bash issues with PlatformIO.

5. **Voltage divider:** Vehicle Enable (Pin 4) is 12V - needs 10k/3.3k divider to GPIO 9.

## Owner Notes

- Private repo: `jordglob/Battery-Emulator`
- Upstream: `dalathegreat/Battery-Emulator`
- Target hardware: Deye inverter + Nissan Leaf (or similar CHAdeMO vehicle)
