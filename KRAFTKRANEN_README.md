# Kraftkranen v2.0 - CHAdeMO V2H Controller

A universal Vehicle-to-Home (V2H) controller that extracts DC power from a CHAdeMO-equipped electric vehicle and feeds it to a Deye inverter using a "Man-in-the-Middle" approach.

## Overview

Kraftkranen acts as a CHAdeMO EVSE (charging station) that negotiates with the vehicle for **discharge** instead of charge. The vehicle thinks it's connected to a V2H-compatible charger and releases power through the CHAdeMO connector.

```
┌─────────────┐      CAN Bus       ┌──────────────────┐      DC Power      ┌─────────────┐
│   Vehicle   │◄──────────────────►│   Kraftkranen    │──────────────────►│    Deye     │
│  (Nissan)   │  CHAdeMO Protocol  │   ESP32-S3       │  300-400V DC       │  Inverter   │
└─────────────┘                    └──────────────────┘                    └─────────────┘
```

## Hardware

### Controller Board
**Waveshare ESP32-S3-Relay-6CH**
- ESP32-S3 with 16MB Flash, 8MB PSRAM
- 6x 12V relay outputs (10A each)
- Native CAN interface
- RS485 interface
- 12V power input via screw terminals

### Power Architecture
```
12V Adapter ─────► VSYS ─────► Board Power
                     │
                     └──► Relay COM (Common) ─────► 12V for all external loads
                                                    │
                                                    ├── Nissan Junction Box Coils
                                                    ├── CHAdeMO d1 Signal
                                                    └── CHAdeMO Lock Solenoid
```

### Contactors (Nissan Leaf Junction Box)
Repurposed from a Nissan Leaf battery pack:
- **Main+ Contactor:** Connects positive HV rail
- **Main- Contactor:** Connects negative HV rail
- **Precharge Relay:** Limits inrush current through resistor

## Wiring Diagram

### Relay Outputs (NO - Normally Open)
| Relay | GPIO | Wire Color | Destination |
|-------|------|------------|-------------|
| 1 | GPIO 4 | RED | Nissan Main+ Coil |
| 2 | GPIO 5 | BLACK | Nissan Main- Coil |
| 3 | GPIO 6 | YELLOW | Nissan Precharge Coil |
| 4 | GPIO 7 | GREEN | CHAdeMO Pin 2 (d1) |
| 5 | GPIO 15 | BLUE | CHAdeMO Pin 7 (Lock) |
| 6 | GPIO 16 | - | Spare (Cooling Fan) |

### Inputs
| Signal | GPIO | Circuit | CHAdeMO Pin |
|--------|------|---------|-------------|
| Vehicle Enable | GPIO 9 | 10k/3.3k voltage divider | Pin 4 (j) |
| Current Sensor | GPIO 10 | ACS758 analog output | - |
| CAN High | GPIO 18 | Direct | Pin 8 |
| CAN Low | GPIO 17 | Direct | Pin 9 |

### CHAdeMO Connector Pinout
```
Pin 1:  Ground Reference (common with ESP32 GND)
Pin 2:  d1 - EVSE Ready / Charger Start (OUTPUT via Relay 4)
Pin 4:  j - Vehicle Permission (INPUT via voltage divider)
Pin 7:  Lock Solenoid (OUTPUT via Relay 5)
Pin 8:  CAN-H
Pin 9:  CAN-L
```

## Precharge Sequence

Critical for protecting the Deye inverter's input capacitors from inrush current damage.

```
Step 1: HANDSHAKE
        ├── Engage Relay 5 (Lock connector)
        └── Engage Relay 4 (Send d1 "Charger Start" to vehicle)

Step 2: WAIT
        └── Wait for CAN response + Vehicle Enable (Pin 4) goes HIGH

Step 3: PRECHARGE
        ├── Engage Relay 2 (Main Negative)
        └── Engage Relay 3 (Precharge) ─── Current flows through resistor

Step 4: STABILIZE
        └── Wait 3 seconds for Deye capacitors to charge

Step 5: RUN
        ├── Engage Relay 1 (Main Positive)
        └── Disengage Relay 3 (Precharge) ─── Full current path established
```

### Shutdown Sequence (Reverse)
```
1. Wait for current to drop below 5A (arc protection)
2. Disengage Relay 1 (Main+)
3. Disengage Relay 3 (Precharge, if on)
4. Disengage Relay 2 (Main-)
5. Disengage Relay 4 (d1 signal)
6. Disengage Relay 5 (Unlock connector)
```

## Current Sensor

**ACS758-50A** Hall Effect Sensor
- Bidirectional current sensing
- Sensitivity: 40mV/A
- Zero current output: VCC/2 (2.5V @ 5V supply, scaled to 3.3V)
- Range: ±50A

```
Current (A) = (ADC_mV - 2500) / 40
```

## State Machine

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  ┌──────┐    ┌───────────┐    ┌──────┐    ┌───────────┐        │
│  │ IDLE │───►│ CONNECTED │───►│ INIT │───►│ NEGOTIATE │        │
│  └──────┘    └───────────┘    └──────┘    └───────────┘        │
│      ▲                                          │               │
│      │                                          ▼               │
│      │       ┌─────────────┐    ┌──────────────────┐           │
│      │       │ EVSE_PREPARE│◄───│   EV_ALLOWED     │           │
│      │       └─────────────┘    └──────────────────┘           │
│      │              │                                           │
│      │              ▼                                           │
│      │       ┌─────────────┐    ┌──────────────────┐           │
│      │       │ EVSE_START  │───►│ EVSE_CONTACTORS  │           │
│      │       └─────────────┘    └──────────────────┘           │
│      │                                   │                      │
│      │                                   ▼                      │
│      │       ┌──────┐           ┌──────────────┐               │
│      └───────│ STOP │◄──────────│  POWERFLOW   │               │
│              └──────┘           └──────────────┘               │
│                  │                                              │
│                  ▼                                              │
│              ┌───────┐                                          │
│              │ FAULT │ (requires power cycle)                   │
│              └───────┘                                          │
└──────────────────────────────────────────────────────────────────┘
```

## Building

### Prerequisites
- PlatformIO
- Python 3.x

### Build Command
```bash
# Windows (PowerShell)
cd Battery-Emulator
.\build.bat 6ch_relay_s3

# Linux/Mac
platformio run -e 6ch_relay_s3
```

### Flash
```bash
platformio run -e 6ch_relay_s3 --target upload
```

## Safety Warnings

**HIGH VOLTAGE DC (300-450V) - LETHAL**

1. Never work on energized circuits
2. Always verify zero voltage before touching
3. Use properly rated contactors (DC rated, >500V)
4. Install emergency stop capability
5. Use isolation monitoring
6. Qualified personnel only

## License

Based on [dalathegreat/Battery-Emulator](https://github.com/dalathegreat/Battery-Emulator)

Private development repository - not for public distribution.
