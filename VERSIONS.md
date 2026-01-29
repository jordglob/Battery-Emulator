# Kraftkranen v2.0 - Version History

## Changelog

### v2.0.0 (2025-01-29)
**Major Release: Kraftkranen Hardware Adaptation**

#### New Features
- Full CHAdeMO V2H (Vehicle-to-Home) bidirectional discharge protocol
- Waveshare ESP32-S3-Relay-6CH board support
- Direct relay control for Nissan Leaf Junction Box contactors
- ACS758 Hall effect current sensor integration (ADC)
- Kraftkranen-specific precharge sequence for Deye inverter

#### Hardware Mapping
| Relay | GPIO | Function |
|-------|------|----------|
| 1 | GPIO 4 | Nissan Main+ Contactor |
| 2 | GPIO 5 | Nissan Main- Contactor |
| 3 | GPIO 6 | Precharge Relay |
| 4 | GPIO 7 | CHAdeMO Pin 2 (d1) |
| 5 | GPIO 15 | CHAdeMO Pin 7 (Lock) |
| 6 | GPIO 16 | Spare (Cooling Fan) |

#### Inputs
| GPIO | Function |
|------|----------|
| GPIO 9 | Vehicle Enable (Pin 4) via voltage divider |
| GPIO 10 | ACS758 Current Sensor (ADC) |
| GPIO 17/18 | CAN Bus TX/RX |

#### Files Changed
- `Software/src/devboard/hal/hw_6ch_relay_s3.h` - Kraftkranen HAL
- `Software/src/battery/CHADEMO-V2H-BATTERY.cpp` - V2H state machine
- `Software/src/battery/CHADEMO-V2H-BATTERY.h` - V2H header
- `Software/src/devboard/hal/hal.h` - Base HAL additions
- `platformio.ini` - Build environment
- `build.bat` - Windows build script

---

### v1.0.0 (2025-01-29)
**Initial CHAdeMO V2H Implementation**

#### Features
- CHAdeMO V2H state machine (11 states)
- CAN protocol for V2H discharge (0x100-0x209 messages)
- Discharge compatibility handshake
- Basic contactor control integration
- Datalayer mapping for inverter communication

#### State Machine
```
IDLE -> CONNECTED -> INIT -> NEGOTIATE -> EV_ALLOWED ->
EVSE_PREPARE -> EVSE_START -> EVSE_CONTACTORS -> POWERFLOW -> STOP
                                                          -> FAULT
```

---

## Build Information

**Platform:** ESP32-S3 (Espressif 32)
**Board:** esp32s3_flash_16MB
**Framework:** Arduino
**Flash:** ~31% used (2MB / 6.5MB)
**RAM:** ~25% used (82KB / 328KB)

### Build Command
```bash
# Windows PowerShell
.\build.bat 6ch_relay_s3

# Or directly with PlatformIO
python -m platformio run -e 6ch_relay_s3
```

---

## Upstream Compatibility

Based on: [dalathegreat/Battery-Emulator](https://github.com/dalathegreat/Battery-Emulator)

Kraftkranen additions are designed to be non-breaking:
- New battery type: `ChademoV2H = 49`
- New HAL: `HW_6CH_RELAY_S3`
- All existing battery profiles remain functional
