/**
 * @file CHADEMO-V2H-BATTERY.cpp
 * @brief CHAdeMO V2H (Vehicle-to-Home) Bidirectional Protocol Implementation
 *
 * Implementation of the CHAdeMO V2H discharge protocol for Battery-Emulator.
 * This module enables an EV battery to provide power to a home inverter.
 *
 * ============================================================================
 *                           TIMING REQUIREMENTS
 * ============================================================================
 * Per CHAdeMO specification:
 * - CAN messages must be sent every 100ms during normal operation
 * - During critical transitions, 10ms cycles may be required
 * - Handshake timeouts are typically 5-10 seconds
 * - Contactor operations must complete within specified timeframes
 * ============================================================================
 */

#include "CHADEMO-V2H-BATTERY.h"
#include "../communication/can/comm_can.h"
#include "../devboard/utils/events.h"

/* ============================================================================
 *                              CONSTRUCTOR
 * ============================================================================
 */
ChademoV2HBattery::ChademoV2HBattery()
  : CanBattery(can_config.battery, CAN_Speed::CAN_SPEED_500KBPS),
    currentState(V2H_IDLE),
    previousState(V2H_IDLE),
    stateEntryTime(0),
    lastCanRxTime(0),
    lastCanTxTime(0),
    plugInserted(false),
    vehicleCanReceived(false),
    vehicleCanInitialized(false),
    vehiclePermission(false),
    evsePermission(false),
    dischargeCompatible(true),
    contactorsReady(false),
    frameCounter(0),
    measuredVoltage(0),
    measuredCurrent(0)
{
  // Initialize Kraftkranen relay outputs from HAL
  relay_main_pos   = esp32hal->POSITIVE_CONTACTOR_PIN();  // Relay 1: Nissan Main+
  relay_main_neg   = esp32hal->NEGATIVE_CONTACTOR_PIN();  // Relay 2: Nissan Main-
  relay_precharge  = esp32hal->PRECHARGE_PIN();           // Relay 3: Precharge
  relay_d1         = esp32hal->CHADEMO_PIN_2();           // Relay 4: CHAdeMO Pin 2 (d1)
  relay_lock       = esp32hal->CHADEMO_LOCK();            // Relay 5: CHAdeMO Pin 7 Lock

  // Initialize Kraftkranen inputs from HAL
  pin_vehicle_enable = esp32hal->CHADEMO_PIN_4();         // Vehicle Enable input
  pin_current_sensor = esp32hal->CURRENT_SENSOR_PIN();    // ACS758 ADC input

  // Initialize CAN frames with default values
  canFrame_108 = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x108,
                  .data = {0x01, 0xF4, 0x01, 0x0F, 0xB3, 0x01, 0x00, 0x00}};

  canFrame_109 = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x109,
                  .data = {0x02, 0x00, 0x00, 0x00, 0x01, 0x20, 0xFF, 0xFF}};

  canFrame_118 = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x118,
                  .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  canFrame_208 = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x208,
                  .data = {0xFF, 0xF4, 0x01, 0xF0, 0x00, 0x00, 0xFA, 0x00}};

  canFrame_209 = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x209,
                  .data = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  // Initialize vehicle data structures with safe defaults
  memset(&vehicleChargeLimits, 0, sizeof(vehicleChargeLimits));
  memset(&vehicleChargeEstimate, 0, sizeof(vehicleChargeEstimate));
  memset(&vehicleSession, 0, sizeof(vehicleSession));
  memset(&vehicleDischargeLimits, 0, sizeof(vehicleDischargeLimits));
  memset(&vehicleDischargeEstimate, 0, sizeof(vehicleDischargeEstimate));
  memset(&vehicleDynamicControl, 0, sizeof(vehicleDynamicControl));

  // Set reasonable defaults for vehicle limits
  vehicleChargeLimits.minimumBatteryVoltage = 300;
  vehicleChargeLimits.maximumBatteryVoltage = 402;
  vehicleDischargeLimits.minimumDischargeVoltage = 260;
  vehicleSession.stateOfCharge = 50;

  // Initialize EVSE structures
  initializeEvseStructures();
}

/* ============================================================================
 *                              SETUP
 * ============================================================================
 * Called once during system initialization. Configures GPIO pins and
 * initializes the CHAdeMO V2H state machine.
 */
void ChademoV2HBattery::setup(void) {
  // Allocate Kraftkranen relay output pins
  if (!esp32hal->alloc_pins(Name, relay_main_pos, relay_main_neg, relay_precharge,
                            relay_d1, relay_lock)) {
    logging.println("CHAdeMO V2H: Failed to allocate relay pins");
    set_event(EVENT_GPIO_NOT_DEFINED, 0);
    return;
  }

  // Configure relay outputs - all OFF initially (Normally Open relays)
  pinMode(relay_main_pos, OUTPUT);
  digitalWrite(relay_main_pos, LOW);   // Relay 1: Main+ OFF

  pinMode(relay_main_neg, OUTPUT);
  digitalWrite(relay_main_neg, LOW);   // Relay 2: Main- OFF

  pinMode(relay_precharge, OUTPUT);
  digitalWrite(relay_precharge, LOW);  // Relay 3: Precharge OFF

  pinMode(relay_d1, OUTPUT);
  digitalWrite(relay_d1, LOW);         // Relay 4: CHAdeMO d1 OFF

  pinMode(relay_lock, OUTPUT);
  digitalWrite(relay_lock, LOW);       // Relay 5: Lock OFF

  // Configure input pins
  pinMode(pin_vehicle_enable, INPUT);  // CHAdeMO Pin 4 (j) - Vehicle Enable

  // Configure ADC for current sensor
  if (pin_current_sensor != GPIO_NUM_NC) {
    analogReadResolution(12);          // 12-bit ADC
    analogSetAttenuation(ADC_11db);    // Full 0-3.3V range
  }

  // Set battery protocol name for web display
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  // Initialize state machine
  currentState = V2H_IDLE;
  stateEntryTime = millis();

  // Prevent contactor closure until vehicle permission granted
  datalayer.system.status.battery_allows_contactor_closing = false;

  // Initialize with placeholder values (will be updated from vehicle)
  datalayer.battery.status.soh_pptt = 9900;    // 99% SOH assumed
  datalayer.battery.status.real_soc = 5000;    // 50% SOC placeholder

  // Conservative power limits - will be updated based on vehicle capabilities
  datalayer.battery.status.max_discharge_power_W = CHADEMO_V2H_MAX_POWER_W;
  datalayer.battery.status.max_charge_power_W = 0; // V2H = discharge only

  // Voltage limits for typical EV battery
  datalayer.battery.info.max_design_voltage_dV = 4100;  // 410V max
  datalayer.battery.info.min_design_voltage_dV = 2600;  // 260V min

  // Enable V2H discharge mode by default
  dischargeCompatible = true;
  evseStatus.dischargeCompatible = true;

  logging.println("CHAdeMO V2H: Setup complete, entering IDLE state");
  logging.println("CHAdeMO V2H: Waiting for vehicle connection...");
}

/* ============================================================================
 *                         INITIALIZE EVSE STRUCTURES
 * ============================================================================
 * Sets up EVSE data structures with default values before vehicle negotiation.
 */
void ChademoV2HBattery::initializeEvseStructures() {
  // EVSE Capabilities (0x108)
  evseCapabilities.contactorWeldDetection = 0x01;  // We support weld detection
  evseCapabilities.availableOutputVoltage = CHADEMO_V2H_MAX_OUTPUT_VOLTAGE_V;
  evseCapabilities.availableOutputCurrent = CHADEMO_V2H_MAX_OUTPUT_CURRENT_A;
  evseCapabilities.thresholdVoltage = 297;  // ~96 cells * 3.1V

  // EVSE Status (0x109)
  evseStatus.protocolNumber = CHADEMO_V2H_PROTO_V1_1;  // V1.1 supports V2H
  evseStatus.setpointVoltage = 0;
  evseStatus.setpointCurrent = 0;
  evseStatus.dischargeCompatible = true;  // KEY: Enable V2H discharge!
  evseStatus.evseStatus = false;
  evseStatus.evseError = false;
  evseStatus.connectorLocked = false;
  evseStatus.batteryIncompatible = false;
  evseStatus.chgDischError = false;
  evseStatus.chgDischStopControl = true;  // Start with stop asserted
  evseStatus.remainingTime10s = 0xFF;     // Unused
  evseStatus.remainingTime1min = 0xFF;    // Unused

  // EVSE Discharge Capabilities (0x208)
  evseDischargeCapabilities.presentDischargeCurrent = 0xFF;  // 0xFF - current
  evseDischargeCapabilities.availableInputVoltage = CHADEMO_V2H_MIN_VOLTAGE_V;
  evseDischargeCapabilities.availableInputCurrent = 0xFF - CHADEMO_V2H_MAX_OUTPUT_CURRENT_A;
  evseDischargeCapabilities.lowerThresholdVoltage = 0;

  // EVSE Discharge Estimate (0x209)
  evseDischargeEstimate.sequenceControlNumber = 0x02;  // V2H 2.0 sequence
  evseDischargeEstimate.remainingDischargeTime = 0x0000;  // Unused
}

/* ============================================================================
 *                         CAN FRAME RECEPTION
 * ============================================================================
 * Processes incoming CAN frames from the vehicle. Only handles CHAdeMO-specific
 * message IDs (0x100-0x202, 0x700).
 */
void ChademoV2HBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  // Filter for CHAdeMO message IDs only
  if (!((rx_frame.ID >= 0x100 && rx_frame.ID <= 0x202) || rx_frame.ID == 0x700)) {
    return;
  }

  lastCanRxTime = millis();
  vehicleCanReceived = true;

  switch (rx_frame.ID) {
    case 0x100:  // Vehicle charging limits
      processVehicleChargeLimits(rx_frame);
      break;

    case 0x101:  // Vehicle charging estimates
      processVehicleChargeEstimate(rx_frame);
      break;

    case 0x102:  // Vehicle session status - CRITICAL
      frameCounter++;
      // Skip first few frames as they may contain stale data
      if (vehicleCanInitialized && frameCounter < 20) {
        return;
      }
      processVehicleSession(rx_frame);
      break;

    case 0x200:  // Vehicle discharge limits (V2H)
      processVehicleDischargeLimits(rx_frame);
      break;

    case 0x201:  // Vehicle discharge estimate (V2H)
      processVehicleDischargeEstimate(rx_frame);
      break;

    case 0x110:  // Vehicle dynamic control (CHAdeMO 2.0)
      processVehicleDynamicControl(rx_frame);
      break;

    case 0x700:  // Vendor ID (informational)
      // Log but don't process - for diagnostics only
      break;

    default:
      break;
  }

  // Transition from INIT to NEGOTIATE on first CAN received
  if (currentState == V2H_INIT) {
    transitionToState(V2H_NEGOTIATE);
  }

  // Run state machine after processing CAN
  runStateMachine();
}

/* ============================================================================
 *                    VEHICLE CAN MESSAGE PROCESSORS
 * ============================================================================
 */

/**
 * Process H100 - Vehicle charging minimums
 */
void ChademoV2HBattery::processVehicleChargeLimits(const CAN_frame& frame) {
  vehicleChargeLimits.minimumChargeCurrent = frame.data.u8[0];
  vehicleChargeLimits.minimumBatteryVoltage =
      (frame.data.u8[3] << 8) | frame.data.u8[2];
  vehicleChargeLimits.maximumBatteryVoltage =
      (frame.data.u8[5] << 8) | frame.data.u8[4];
  vehicleChargeLimits.constantOfChargingRate = frame.data.u8[6];
}

/**
 * Process H101 - Vehicle charging estimates
 */
void ChademoV2HBattery::processVehicleChargeEstimate(const CAN_frame& frame) {
  vehicleChargeEstimate.maxChargingTime10s = frame.data.u8[1];
  vehicleChargeEstimate.maxChargingTime1min = frame.data.u8[2];
  vehicleChargeEstimate.estimatedChargingTime = frame.data.u8[3];
  vehicleChargeEstimate.ratedBatteryCapacity =
      (frame.data.u8[6] << 8) | frame.data.u8[5];
}

/**
 * Process H102 - Vehicle charging session (MOST CRITICAL for V2H)
 * This contains the discharge compatibility flag and current session state.
 */
void ChademoV2HBattery::processVehicleSession(const CAN_frame& frame) {
  uint16_t newTargetVoltage = (frame.data.u8[2] << 8) | frame.data.u8[1];
  uint16_t priorTargetVoltage = vehicleSession.targetBatteryVoltage;

  vehicleCanInitialized = true;
  vehiclePermission = digitalRead(pin_vehicle_enable);  // CHAdeMO Pin 4

  // Byte 0: Protocol version
  vehicleSession.controlProtocolNumber = frame.data.u8[0];

  // Byte 4: Fault flags
  vehicleSession.faultBatteryOverVoltage = bitRead(frame.data.u8[4], 0);
  vehicleSession.faultBatteryUnderVoltage = bitRead(frame.data.u8[4], 1);
  vehicleSession.faultBatteryCurrentDeviation = bitRead(frame.data.u8[4], 2);
  vehicleSession.faultHighBatteryTemperature = bitRead(frame.data.u8[4], 3);
  vehicleSession.faultBatteryVoltageDeviation = bitRead(frame.data.u8[4], 4);

  // Byte 5: Status flags - CRITICAL V2H BIT IS HERE!
  vehicleSession.statusVehicleChargingEnabled = bitRead(frame.data.u8[5], 0);
  vehicleSession.statusVehicleShifterPosition = bitRead(frame.data.u8[5], 1);
  vehicleSession.statusChargingError = bitRead(frame.data.u8[5], 2);
  vehicleSession.statusVehicle = bitRead(frame.data.u8[5], 3);
  vehicleSession.statusNormalStopRequest = bitRead(frame.data.u8[5], 4);
  // BIT 7: V2H DISCHARGE COMPATIBILITY FLAG!
  vehicleSession.statusVehicleDischargeCompatible = bitRead(frame.data.u8[5], 7);

  // Byte 6: State of charge
  vehicleSession.stateOfCharge = frame.data.u8[6];

  // Update current/voltage request
  vehicleSession.chargingCurrentRequest = frame.data.u8[3];
  vehicleSession.targetBatteryVoltage = newTargetVoltage;

  // === Safety Checks ===

  // Check for fault conditions
  if (checkVehicleFaults()) {
    transitionToState(V2H_STOP);
    return;
  }

  // Check for inconsistent permission state
  // (CAN received before plug but permission signal active = error)
  if ((currentState == V2H_INIT && vehiclePermission) ||
      (vehicleSession.statusVehicleChargingEnabled && !vehiclePermission)) {
    logging.println("CHAdeMO V2H: Inconsistent charge/discharge state");
    transitionToState(V2H_FAULT);
    return;
  }

  // Check if vehicle ended session (target voltage -> 0)
  if (priorTargetVoltage > 0 && newTargetVoltage == 0) {
    logging.println("CHAdeMO V2H: Vehicle requested session end");
    transitionToState(V2H_STOP);
    return;
  }

  // Check if vehicle withdrew permission during powerflow
  if (currentState == V2H_POWERFLOW && !vehiclePermission) {
    logging.println("CHAdeMO V2H: Vehicle permission withdrawn");
    transitionToState(V2H_STOP);
    return;
  }

  // === State Transitions Based on Session Data ===

  // Permission granted during negotiation -> advance to EV_ALLOWED
  if (vehiclePermission && currentState == V2H_NEGOTIATE) {
    transitionToState(V2H_EV_ALLOWED);
    return;
  }

  // Vehicle ready to start during EVSE_PREPARE
  if (vehiclePermission && currentState == V2H_EVSE_PREPARE &&
      priorTargetVoltage == 0 && newTargetVoltage > 0 &&
      vehicleSession.statusVehicleChargingEnabled) {
    transitionToState(V2H_EVSE_START);
    return;
  }
}

/**
 * Process H200 - Vehicle discharge limits (V2H specific)
 */
void ChademoV2HBattery::processVehicleDischargeLimits(const CAN_frame& frame) {
  vehicleDischargeLimits.maximumDischargeCurrent = frame.data.u8[0];
  vehicleDischargeLimits.minimumDischargeVoltage =
      (frame.data.u8[5] << 8) | frame.data.u8[4];
  vehicleDischargeLimits.minimumBatteryDischargeLevel = frame.data.u8[6];
  vehicleDischargeLimits.maxRemainingCapacityForChg = frame.data.u8[7];

  // Check if we've hit minimum discharge voltage
  if (getMeasuredVoltage() <= vehicleDischargeLimits.minimumDischargeVoltage &&
      currentState > V2H_NEGOTIATE) {
    logging.println("CHAdeMO V2H: Minimum discharge voltage reached");
    transitionToState(V2H_STOP);
  }
}

/**
 * Process H201 - Vehicle discharge estimate (V2H)
 */
void ChademoV2HBattery::processVehicleDischargeEstimate(const CAN_frame& frame) {
  vehicleDischargeEstimate.v2hSequenceNumber = frame.data.u8[0];
  vehicleDischargeEstimate.approxDischargeCompletionTime =
      (frame.data.u8[2] << 8) | frame.data.u8[1];
  vehicleDischargeEstimate.availableVehicleEnergy =
      (frame.data.u8[4] << 8) | frame.data.u8[3];
}

/**
 * Process H110 - Vehicle dynamic control (CHAdeMO 2.0)
 */
void ChademoV2HBattery::processVehicleDynamicControl(const CAN_frame& frame) {
  vehicleDynamicControl.dynamicControlSupport = bitRead(frame.data.u8[0], 0);
  vehicleDynamicControl.highCurrentControlSupport = bitRead(frame.data.u8[0], 1);
  vehicleDynamicControl.highVoltageControlSupport = bitRead(frame.data.u8[0], 2);
}

/* ============================================================================
 *                      EVSE CAN MESSAGE BUILDERS
 * ============================================================================
 * These functions update the CAN frame data to be transmitted to the vehicle.
 */

/**
 * Build H108 - EVSE capabilities
 * Sent every 100ms during session.
 */
void ChademoV2HBattery::buildEvseCapabilities() {
  // Update threshold voltage based on vehicle max (minus 2% safety margin)
  evseCapabilities.thresholdVoltage =
      vehicleChargeLimits.maximumBatteryVoltage -
      (vehicleChargeLimits.maximumBatteryVoltage * CHADEMO_V2H_VOLTAGE_TOLERANCE_PCT / 100);

  // Calculate available current based on power limit and voltage
  if (evseCapabilities.availableOutputVoltage > 0) {
    evseCapabilities.availableOutputCurrent =
        CHADEMO_V2H_MAX_POWER_W / evseCapabilities.availableOutputVoltage;
  }

  // Build CAN frame
  canFrame_108.data.u8[0] = evseCapabilities.contactorWeldDetection;
  canFrame_108.data.u8[1] = lowByte(evseCapabilities.availableOutputVoltage);
  canFrame_108.data.u8[2] = highByte(evseCapabilities.availableOutputVoltage);
  canFrame_108.data.u8[3] = evseCapabilities.availableOutputCurrent;
  canFrame_108.data.u8[4] = lowByte(evseCapabilities.thresholdVoltage);
  canFrame_108.data.u8[5] = highByte(evseCapabilities.thresholdVoltage);
  canFrame_108.data.u8[6] = 0x00;  // Unused
  canFrame_108.data.u8[7] = 0x00;  // Unused
}

/**
 * Build H109 - EVSE status
 * Contains the critical discharge_compatible flag for V2H!
 */
void ChademoV2HBattery::buildEvseStatus() {
  // Update setpoints based on vehicle request (capped by our limits)
  evseStatus.setpointVoltage = min(vehicleSession.targetBatteryVoltage,
                                    evseCapabilities.availableOutputVoltage);
  evseStatus.setpointCurrent = min(vehicleSession.chargingCurrentRequest,
                                    evseCapabilities.availableOutputCurrent);

  // In discharge mode, use measured values for setpoints
  if (currentState == V2H_POWERFLOW) {
    evseStatus.setpointVoltage = getMeasuredVoltage();
    evseStatus.setpointCurrent = abs(getMeasuredCurrent());
  }

  // Check for battery incompatibility
  if ((vehicleSession.targetBatteryVoltage > evseCapabilities.availableOutputVoltage) ||
      (vehicleChargeLimits.maximumBatteryVoltage < evseCapabilities.thresholdVoltage)) {
    evseStatus.evseError = true;
    evseStatus.batteryIncompatible = true;
    evseStatus.chgDischStopControl = true;
    transitionToState(V2H_FAULT);
  }

  // Calculate remaining time (simplified - 60 minutes default)
  evseStatus.remainingTime1min = 60;

  // Build CAN frame
  canFrame_109.data.u8[0] = evseStatus.protocolNumber;
  canFrame_109.data.u8[1] = lowByte(evseStatus.setpointVoltage);
  canFrame_109.data.u8[2] = highByte(evseStatus.setpointVoltage);
  canFrame_109.data.u8[3] = evseStatus.setpointCurrent;

  // Byte 4: Discharge compatible flag - CRITICAL FOR V2H!
  canFrame_109.data.u8[4] = evseStatus.dischargeCompatible ? 0x01 : 0x00;

  // Byte 5: Status bits packed
  canFrame_109.data.u8[5] =
      (evseStatus.evseStatus ? 0x01 : 0x00) |
      (evseStatus.evseError ? 0x02 : 0x00) |
      (evseStatus.connectorLocked ? 0x04 : 0x00) |
      (evseStatus.batteryIncompatible ? 0x08 : 0x00) |
      (evseStatus.chgDischError ? 0x10 : 0x00) |
      (evseStatus.chgDischStopControl ? 0x20 : 0x00);

  canFrame_109.data.u8[6] = evseStatus.remainingTime10s;
  canFrame_109.data.u8[7] = evseStatus.remainingTime1min;
}

/**
 * Build H208 - EVSE discharge capabilities (V2H specific)
 */
void ChademoV2HBattery::buildEvseDischargeCapabilities() {
  // Present discharge current (0xFF encoding: 0xFF - actual current)
  evseDischargeCapabilities.presentDischargeCurrent = 0xFF - abs(getMeasuredCurrent());

  // Set available input based on vehicle's limits
  evseDischargeCapabilities.availableInputVoltage =
      vehicleDischargeLimits.minimumDischargeVoltage;

  // Max input current (0xFF encoding)
  evseDischargeCapabilities.availableInputCurrent =
      0xFF - min((uint8_t)CHADEMO_V2H_MAX_OUTPUT_CURRENT_A,
                 vehicleDischargeLimits.maximumDischargeCurrent);

  // Lower threshold with 2% safety margin above minimum
  evseDischargeCapabilities.lowerThresholdVoltage =
      vehicleDischargeLimits.minimumDischargeVoltage +
      (vehicleDischargeLimits.minimumDischargeVoltage * CHADEMO_V2H_VOLTAGE_TOLERANCE_PCT / 100);

  // Build CAN frame
  canFrame_208.data.u8[0] = evseDischargeCapabilities.presentDischargeCurrent;
  canFrame_208.data.u8[1] = lowByte(evseDischargeCapabilities.availableInputVoltage);
  canFrame_208.data.u8[2] = highByte(evseDischargeCapabilities.availableInputVoltage);
  canFrame_208.data.u8[3] = evseDischargeCapabilities.availableInputCurrent;
  canFrame_208.data.u8[4] = 0x00;  // Unused
  canFrame_208.data.u8[5] = 0x00;  // Unused
  canFrame_208.data.u8[6] = lowByte(evseDischargeCapabilities.lowerThresholdVoltage);
  canFrame_208.data.u8[7] = highByte(evseDischargeCapabilities.lowerThresholdVoltage);
}

/**
 * Build H209 - EVSE discharge estimate
 */
void ChademoV2HBattery::buildEvseDischargeEstimate() {
  canFrame_209.data.u8[0] = evseDischargeEstimate.sequenceControlNumber;
  canFrame_209.data.u8[1] = lowByte(evseDischargeEstimate.remainingDischargeTime);
  canFrame_209.data.u8[2] = highByte(evseDischargeEstimate.remainingDischargeTime);
  // Bytes 3-7 unused
}

/* ============================================================================
 *                           CAN TRANSMISSION
 * ============================================================================
 * Called periodically to send CAN frames to the vehicle.
 * Per CHAdeMO spec, messages should be sent every 100ms.
 */
void ChademoV2HBattery::transmit_can(unsigned long currentMillis) {
  runStateMachine();

  // Don't transmit until vehicle has initiated communication
  if (currentState <= V2H_INIT) {
    return;
  }

  // Send at 100ms intervals
  if (currentMillis - lastCanTxTime < CHADEMO_V2H_CAN_INTERVAL_MS) {
    return;
  }
  lastCanTxTime = currentMillis;

  // Build and send EVSE frames
  buildEvseCapabilities();
  buildEvseStatus();
  buildEvseDischargeCapabilities();
  buildEvseDischargeEstimate();

  transmit_can_frame(&canFrame_108);
  transmit_can_frame(&canFrame_109);

  // Only send discharge frames if we're V2H compatible
  if (dischargeCompatible) {
    transmit_can_frame(&canFrame_208);
    transmit_can_frame(&canFrame_209);
  }

  // Send dynamic control frame for CHAdeMO 2.0 vehicles
  if (vehicleSession.controlProtocolNumber >= CHADEMO_V2H_PROTO_V2_0) {
    transmit_can_frame(&canFrame_118);
  }
}

/* ============================================================================
 *                          STATE MACHINE
 * ============================================================================
 * The heart of the CHAdeMO V2H implementation. Manages the handshake sequence
 * and power transfer states.
 */
void ChademoV2HBattery::transitionToState(ChademoV2HState newState) {
  if (newState == currentState) return;

  logging.printf("CHAdeMO V2H: State %d -> %d\n", currentState, newState);

  previousState = currentState;
  currentState = newState;
  stateEntryTime = millis();

  // Update extended datalayer for web UI
  datalayer_extended.chademo.CHADEMO_Status = static_cast<uint8_t>(currentState);
}

void ChademoV2HBattery::runStateMachine() {
  unsigned long now = millis();

  // === Global Safety Checks ===

  // Vehicle not in park during active session
  if (currentState >= V2H_EV_ALLOWED &&
      vehicleSession.statusVehicleShifterPosition) {
    logging.println("CHAdeMO V2H: Vehicle not in park, aborting");
    transitionToState(V2H_STOP);
  }

  // Permission withdrawn during active session
  if (currentState >= V2H_EV_ALLOWED && !vehiclePermission) {
    logging.println("CHAdeMO V2H: Permission withdrawn");
    transitionToState(V2H_STOP);
  }

  // CAN timeout check (no messages for 2 seconds during active session)
  if (currentState > V2H_INIT &&
      (now - lastCanRxTime) > 2000) {
    logging.println("CHAdeMO V2H: CAN timeout");
    transitionToState(V2H_FAULT);
  }

  // === State-Specific Handlers ===
  switch (currentState) {
    case V2H_IDLE:
      handleStateIdle();
      break;
    case V2H_CONNECTED:
      handleStateConnected();
      break;
    case V2H_INIT:
      handleStateInit();
      break;
    case V2H_NEGOTIATE:
      handleStateNegotiate();
      break;
    case V2H_EV_ALLOWED:
      handleStateEvAllowed();
      break;
    case V2H_EVSE_PREPARE:
      handleStateEvsePrepare();
      break;
    case V2H_EVSE_START:
      handleStateEvseStart();
      break;
    case V2H_EVSE_CONTACTORS:
      handleStateContactors();
      break;
    case V2H_POWERFLOW:
      handleStatePowerflow();
      break;
    case V2H_STOP:
      handleStateStop();
      break;
    case V2H_FAULT:
      handleStateFault();
      break;
    default:
      transitionToState(V2H_FAULT);
      break;
  }
}

/**
 * IDLE State: Waiting for connection
 *
 * Kraftkranen: All relays OFF, waiting for vehicle to be connected.
 * Since we don't have proximity sensor, we wait for user command or
 * vehicle CAN messages to start.
 */
void ChademoV2HBattery::handleStateIdle() {
  // Ensure all relays are OFF (safe state)
  digitalWrite(relay_main_pos, LOW);   // Relay 1: Main+ OFF
  digitalWrite(relay_main_neg, LOW);   // Relay 2: Main- OFF
  digitalWrite(relay_precharge, LOW);  // Relay 3: Precharge OFF
  digitalWrite(relay_d1, LOW);         // Relay 4: d1 OFF
  digitalWrite(relay_lock, LOW);       // Relay 5: Lock OFF

  contactorsReady = false;
  plugInserted = false;

  // Kraftkranen: No proximity sensor, check for vehicle CAN or manual start
  // If we receive CAN from vehicle, that means it's connected
  if (vehicleCanReceived) {
    logging.println("CHAdeMO V2H: Vehicle CAN detected, starting handshake");
    plugInserted = true;
    transitionToState(V2H_CONNECTED);
  }
}

/**
 * CONNECTED State: Vehicle detected, starting handshake
 *
 * Kraftkranen Sequence Step 1: Engage Lock (R5) -> Start Signal (R4)
 */
void ChademoV2HBattery::handleStateConnected() {
  if (!vehicleCanReceived) {
    // Lost vehicle communication
    logging.println("CHAdeMO V2H: Lost vehicle CAN, returning to IDLE");
    transitionToState(V2H_IDLE);
    return;
  }

  // Step 1: Engage connector lock (Relay 5)
  digitalWrite(relay_lock, HIGH);
  logging.println("CHAdeMO V2H: Connector LOCKED (Relay 5)");

  // Small delay for lock to engage
  delay(KRAFTKRANEN_CONTACTOR_DELAY_MS);

  // Step 2: Send "Charger Start" signal via Relay 4 (CHAdeMO Pin 2 / d1)
  digitalWrite(relay_d1, HIGH);
  logging.println("CHAdeMO V2H: Charger Start signal ON (Relay 4)");

  // Transition to INIT - waiting for vehicle CAN response
  transitionToState(V2H_INIT);
}

/**
 * INIT State: Waiting for first vehicle CAN message
 */
void ChademoV2HBattery::handleStateInit() {
  // Initialize EVSE structures while waiting
  initializeEvseStructures();

  // Timeout check
  if (millis() - stateEntryTime > CHADEMO_V2H_INIT_TIMEOUT_MS) {
    logging.println("CHAdeMO V2H: Init timeout, no vehicle CAN received");
    transitionToState(V2H_STOP);
  }

  // State transition to NEGOTIATE happens in handle_incoming_can_frame
  // when first vehicle CAN is received
}

/**
 * NEGOTIATE State: Exchanging capabilities with vehicle
 */
void ChademoV2HBattery::handleStateNegotiate() {
  // Keep stop control asserted during negotiation
  evseStatus.chgDischStopControl = true;
  evseStatus.evseStatus = false;

  // Check for V2H compatibility
  if (!checkDischargeCompatibility()) {
    logging.println("CHAdeMO V2H: Vehicle not V2H compatible");
    evseStatus.batteryIncompatible = true;
    transitionToState(V2H_STOP);
    return;
  }

  // State transition to EV_ALLOWED happens in processVehicleSession
  // when vehicle grants permission
}

/**
 * EV_ALLOWED State: Vehicle has granted permission
 *
 * Kraftkranen: Verify Vehicle Enable (Pin 4) is HIGH via voltage divider
 */
void ChademoV2HBattery::handleStateEvAllowed() {
  logging.println("CHAdeMO V2H: Checking vehicle permission");

  // Read Vehicle Enable signal (CHAdeMO Pin 4 via voltage divider)
  vehiclePermission = digitalRead(pin_vehicle_enable);

  if (vehiclePermission) {
    logging.println("CHAdeMO V2H: Vehicle Enable HIGH - permission granted");

    // Lock should already be engaged from CONNECTED state
    evseStatus.connectorLocked = true;

    transitionToState(V2H_EVSE_PREPARE);
  } else {
    // Wait for vehicle to assert Pin 4
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 1000) {
      logging.println("CHAdeMO V2H: Waiting for Vehicle Enable (Pin 4)...");
      lastLog = millis();
    }
  }
}

/**
 * EVSE_PREPARE State: Preparing for power transfer
 *
 * Kraftkranen: Vehicle permission confirmed, ready to start precharge sequence
 */
void ChademoV2HBattery::handleStateEvsePrepare() {
  logging.println("CHAdeMO V2H: EVSE preparing for precharge");

  // Verify vehicle still permits operation
  if (vehicleSession.statusVehicleChargingEnabled && vehiclePermission) {
    evsePermission = true;

    // Keep stop control asserted during preparation
    evseStatus.chgDischStopControl = true;
    evseStatus.evseStatus = false;

    // Ready to start - transition to EVSE_START for precharge
    transitionToState(V2H_EVSE_START);
  } else {
    // Vehicle withdrew permission
    logging.println("CHAdeMO V2H: Vehicle withdrew permission");
    transitionToState(V2H_STOP);
  }
}

/**
 * EVSE_START State: Begin Kraftkranen precharge sequence
 *
 * Kraftkranen Step 3: Engage Main- (R2) + Precharge (R3)
 */
void ChademoV2HBattery::handleStateEvseStart() {
  logging.println("CHAdeMO V2H: Starting precharge sequence");

  // Step 3: Engage Negative contactor (Relay 2)
  digitalWrite(relay_main_neg, HIGH);
  logging.println("CHAdeMO V2H: Main NEGATIVE engaged (Relay 2)");

  delay(KRAFTKRANEN_CONTACTOR_DELAY_MS);

  // Step 3: Engage Precharge relay (Relay 3)
  digitalWrite(relay_precharge, HIGH);
  logging.println("CHAdeMO V2H: PRECHARGE engaged (Relay 3)");

  evseStatus.chgDischStopControl = true;  // Still asserting stop
  evseStatus.evseStatus = false;

  // Signal that precharge is in progress
  datalayer.system.status.contactors_engaged = 3;  // 3 = precharging

  transitionToState(V2H_EVSE_CONTACTORS);
}

/**
 * EVSE_CONTACTORS State: Kraftkranen precharge sequence
 *
 * Kraftkranen Step 4: Wait 3 seconds for Deye capacitors to charge
 * Kraftkranen Step 5: Engage Main+ (R1) -> Disengage Precharge (R3)
 */
void ChademoV2HBattery::handleStateContactors() {
  static bool prechargeStarted = false;
  static unsigned long prechargeStartTime = 0;

  if (!prechargeStarted) {
    prechargeStartTime = millis();
    prechargeStarted = true;
    logging.println("CHAdeMO V2H: Precharging Deye capacitors...");
  }

  // Step 4: Wait for precharge to complete (3 seconds)
  unsigned long elapsed = millis() - prechargeStartTime;

  if (elapsed < KRAFTKRANEN_PRECHARGE_TIME_MS) {
    // Still precharging - log progress every second
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 1000) {
      logging.printf("CHAdeMO V2H: Precharging... %lu/%d ms\n",
                     elapsed, KRAFTKRANEN_PRECHARGE_TIME_MS);
      lastLog = millis();
    }
    return;
  }

  // Step 5: Precharge complete - engage Main+ (Relay 1)
  logging.println("CHAdeMO V2H: Precharge complete, engaging Main+");
  digitalWrite(relay_main_pos, HIGH);
  logging.println("CHAdeMO V2H: Main POSITIVE engaged (Relay 1)");

  delay(KRAFTKRANEN_CONTACTOR_DELAY_MS);

  // Step 5: Disengage Precharge relay (Relay 3)
  digitalWrite(relay_precharge, LOW);
  logging.println("CHAdeMO V2H: PRECHARGE disengaged (Relay 3)");

  // All contactors closed!
  contactorsReady = true;
  prechargeStarted = false;
  datalayer.system.status.contactors_engaged = 1;  // 1 = closed

  logging.printf("CHAdeMO V2H: Contactors READY, voltage: %dV\n",
                 getMeasuredVoltage());

  // Verify V2H discharge compatibility
  if (evseStatus.dischargeCompatible &&
      vehicleSession.statusVehicleDischargeCompatible) {
    // Clear stop control - enable power flow!
    evseStatus.chgDischStopControl = false;
    evseStatus.evseStatus = true;
    transitionToState(V2H_POWERFLOW);
  } else {
    logging.println("CHAdeMO V2H: Discharge compatibility mismatch");
    transitionToState(V2H_STOP);
  }
}

/**
 * POWERFLOW State: Active power transfer (discharge)
 */
void ChademoV2HBattery::handleStatePowerflow() {
  // Active discharge in progress!
  evseStatus.chgDischStopControl = false;
  evseStatus.evseStatus = true;

  // Check for minimum discharge voltage
  if (getMeasuredVoltage() <= vehicleDischargeLimits.minimumDischargeVoltage) {
    logging.println("CHAdeMO V2H: Minimum voltage reached");
    transitionToState(V2H_STOP);
  }

  // Vehicle stop request
  if (vehicleSession.statusNormalStopRequest) {
    logging.println("CHAdeMO V2H: Vehicle requested stop");
    transitionToState(V2H_STOP);
  }
}

/**
 * STOP State: Controlled shutdown
 *
 * Kraftkranen shutdown sequence (reverse of precharge):
 * 1. Wait for current to drop to safe level
 * 2. Disengage Main+ (R1)
 * 3. Disengage Main- (R2) and Precharge (R3)
 * 4. Turn off CHAdeMO signals (R4, R5)
 */
void ChademoV2HBattery::handleStateStop() {
  static bool shutdownStarted = false;

  if (!shutdownStarted) {
    logging.println("CHAdeMO V2H: Initiating controlled shutdown");
    shutdownStarted = true;
  }

  // Assert stop control on CHAdeMO CAN
  evseStatus.chgDischStopControl = true;
  evseStatus.evseStatus = false;
  evsePermission = false;
  vehiclePermission = false;

  // Wait for current to drop before opening contactors
  // Per CHAdeMO spec: protect EV contactors from arc damage
  int16_t currentA = getMeasuredCurrent();

  if (abs(currentA) <= CHADEMO_V2H_SAFE_CURRENT_A) {
    logging.printf("CHAdeMO V2H: Current safe (%dA), opening contactors\n", currentA);

    // Step 1: Disengage Main+ (Relay 1)
    digitalWrite(relay_main_pos, LOW);
    logging.println("CHAdeMO V2H: Main+ DISENGAGED (Relay 1)");

    delay(KRAFTKRANEN_CONTACTOR_DELAY_MS);

    // Step 2: Disengage Precharge if still on (Relay 3)
    digitalWrite(relay_precharge, LOW);
    logging.println("CHAdeMO V2H: Precharge DISENGAGED (Relay 3)");

    // Step 3: Disengage Main- (Relay 2)
    digitalWrite(relay_main_neg, LOW);
    logging.println("CHAdeMO V2H: Main- DISENGAGED (Relay 2)");

    delay(KRAFTKRANEN_CONTACTOR_DELAY_MS);

    // Step 4: Turn off CHAdeMO signals
    digitalWrite(relay_d1, LOW);   // Relay 4: Charger Start OFF
    logging.println("CHAdeMO V2H: Charger Start OFF (Relay 4)");

    digitalWrite(relay_lock, LOW); // Relay 5: Unlock connector
    logging.println("CHAdeMO V2H: Connector UNLOCKED (Relay 5)");

    contactorsReady = false;
    datalayer.system.status.contactors_engaged = 0;  // 0 = open
    shutdownStarted = false;

    logging.println("CHAdeMO V2H: Shutdown complete");
    transitionToState(V2H_IDLE);
    return;
  }

  // Log waiting for current to drop
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 1000) {
    logging.printf("CHAdeMO V2H: Waiting for current to drop (%dA > %dA safe)\n",
                   abs(currentA), CHADEMO_V2H_SAFE_CURRENT_A);
    lastLog = millis();
  }

  // Timeout - force shutdown even if current high (safety concern!)
  if (millis() - stateEntryTime > CHADEMO_V2H_STOP_TIMEOUT_MS) {
    logging.println("CHAdeMO V2H: STOP TIMEOUT - forcing emergency shutdown!");

    // Emergency: turn off all relays immediately
    digitalWrite(relay_main_pos, LOW);
    digitalWrite(relay_precharge, LOW);
    digitalWrite(relay_main_neg, LOW);
    digitalWrite(relay_d1, LOW);
    digitalWrite(relay_lock, LOW);

    contactorsReady = false;
    datalayer.system.status.contactors_engaged = 0;
    shutdownStarted = false;

    transitionToState(V2H_IDLE);
  }
}

/**
 * FAULT State: Emergency shutdown - requires power cycle
 *
 * Kraftkranen EMERGENCY: Immediately turn off ALL relays.
 * This state latches and cannot be cleared without power cycle.
 */
void ChademoV2HBattery::handleStateFault() {
  logging.println("CHAdeMO V2H: FAULT - EMERGENCY SHUTDOWN!");

  // Set all error flags
  evseStatus.evseError = true;
  evseStatus.chgDischError = true;
  evseStatus.chgDischStopControl = true;

  // IMMEDIATELY turn off ALL relays - no delays, no waiting for current
  digitalWrite(relay_main_pos, LOW);   // Relay 1: Main+ OFF
  digitalWrite(relay_main_neg, LOW);   // Relay 2: Main- OFF
  digitalWrite(relay_precharge, LOW);  // Relay 3: Precharge OFF
  digitalWrite(relay_d1, LOW);         // Relay 4: CHAdeMO d1 OFF
  digitalWrite(relay_lock, LOW);       // Relay 5: Lock OFF

  evsePermission = false;
  vehiclePermission = false;
  contactorsReady = false;
  datalayer.system.status.contactors_engaged = 2;  // 2 = error

  // FAULT state can only be cleared by power cycle
  // This is a safety feature per CHAdeMO spec
  logging.println("CHAdeMO V2H: All relays OFF - power cycle required to clear");
}

/* ============================================================================
 *                          SAFETY FUNCTIONS
 * ============================================================================
 */

/**
 * Check vehicle fault flags from H102
 */
bool ChademoV2HBattery::checkVehicleFaults() {
  if (vehicleSession.faultBatteryOverVoltage) {
    logging.println("CHAdeMO V2H: Vehicle fault - Over voltage");
    return true;
  }
  if (vehicleSession.faultBatteryUnderVoltage) {
    logging.println("CHAdeMO V2H: Vehicle fault - Under voltage");
    return true;
  }
  if (vehicleSession.faultBatteryCurrentDeviation) {
    logging.println("CHAdeMO V2H: Vehicle fault - Current deviation");
    return true;
  }
  if (vehicleSession.faultHighBatteryTemperature) {
    logging.println("CHAdeMO V2H: Vehicle fault - High temperature");
    return true;
  }
  if (vehicleSession.faultBatteryVoltageDeviation) {
    logging.println("CHAdeMO V2H: Vehicle fault - Voltage deviation");
    return true;
  }
  return false;
}

/**
 * Check voltage compatibility between vehicle and EVSE
 */
bool ChademoV2HBattery::checkVoltageCompatibility() {
  return (vehicleChargeLimits.maximumBatteryVoltage <= CHADEMO_V2H_MAX_OUTPUT_VOLTAGE_V) &&
         (vehicleChargeLimits.minimumBatteryVoltage >= CHADEMO_V2H_MIN_VOLTAGE_V);
}

/**
 * Check V2H discharge compatibility
 * Both vehicle and EVSE must support discharge for V2H to work
 */
bool ChademoV2HBattery::checkDischargeCompatibility() {
  return evseStatus.dischargeCompatible &&
         vehicleSession.statusVehicleDischargeCompatible;
}

/**
 * Emergency safety shutdown - turns off ALL Kraftkranen relays
 */
void ChademoV2HBattery::safetyShutdown(const char* reason) {
  logging.printf("CHAdeMO V2H: SAFETY SHUTDOWN - %s\n", reason);

  // Immediately turn off all relays
  digitalWrite(relay_main_pos, LOW);   // Relay 1
  digitalWrite(relay_main_neg, LOW);   // Relay 2
  digitalWrite(relay_precharge, LOW);  // Relay 3
  digitalWrite(relay_d1, LOW);         // Relay 4
  digitalWrite(relay_lock, LOW);       // Relay 5

  evsePermission = false;
  vehiclePermission = false;
  contactorsReady = false;
  datalayer.system.status.contactors_engaged = 2;  // Error state
  transitionToState(V2H_FAULT);
}

/* ============================================================================
 *                         UTILITY FUNCTIONS
 * ============================================================================
 */

uint16_t ChademoV2HBattery::getMeasuredVoltage() {
  // Try to get voltage from shunt if available
  if (datalayer.shunt.available) {
    return datalayer.shunt.measured_voltage_dV / 10;  // Convert dV to V
  }
  // Fallback to battery voltage
  return datalayer.battery.status.voltage_dV / 10;
}

int16_t ChademoV2HBattery::getMeasuredCurrent() {
  // Try to get current from shunt if available
  if (datalayer.shunt.available) {
    return datalayer.shunt.measured_amperage_dA / 10;  // Convert dA to A
  }

  // Kraftkranen: Read from ACS758 Hall effect current sensor
  if (pin_current_sensor != GPIO_NUM_NC) {
    // Read ADC value (12-bit: 0-4095)
    uint16_t adcValue = analogRead(pin_current_sensor);

    // Convert ADC to millivolts
    // Formula: mV = (adcValue / ADC_RESOLUTION) * ADC_REFERENCE_MV
    uint32_t sensorMv = ((uint32_t)adcValue * ADC_REFERENCE_MV) / ADC_RESOLUTION;

    // Convert millivolts to current
    // ACS758: Output = VCC/2 at 0A, ±sensitivity per amp
    // Formula: current = (sensorMv - zeroPoint) / sensitivity
    int32_t currentMa = ((int32_t)sensorMv - ACS758_ZERO_CURRENT_MV) * 1000 / ACS758_SENSITIVITY_MV_PER_A;

    // Return current in Amps
    return (int16_t)(currentMa / 1000);
  }

  // Fallback to battery current
  return datalayer.battery.status.current_dA / 10;
}

/* ============================================================================
 *                         UPDATE VALUES
 * ============================================================================
 * Maps CHAdeMO V2H data to the datalayer for inverter communication.
 * This is called periodically to update the shared data structure.
 */
void ChademoV2HBattery::update_values() {
  // Always report CAN as alive (we don't require constant CAN before plug)
  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;

  // Handle user requests
  if (datalayer_extended.chademo.UserRequestStop) {
    transitionToState(V2H_STOP);
    datalayer_extended.chademo.UserRequestStop = false;
  }
  if (datalayer_extended.chademo.UserRequestRestart) {
    transitionToState(V2H_IDLE);
    datalayer_extended.chademo.UserRequestRestart = false;
  }

  // === Map vehicle data to datalayer ===

  // State of charge from vehicle (convert % to pptt)
  datalayer.battery.status.real_soc = vehicleSession.stateOfCharge * 100;

  // Max discharge power from vehicle limits
  datalayer.battery.status.max_discharge_power_W =
      vehicleDischargeLimits.maximumDischargeCurrent *
      vehicleChargeLimits.maximumBatteryVoltage;

  // Voltage (only update when vehicle is connected)
  if (vehicleCanReceived) {
    datalayer.battery.status.voltage_dV = getMeasuredVoltage() * 10;
  }

  // Battery capacity from vehicle (convert 0.1kWh to Wh)
  datalayer.battery.info.total_capacity_Wh =
      vehicleChargeEstimate.ratedBatteryCapacity * 100;

  // Calculate remaining capacity
  datalayer.battery.status.remaining_capacity_Wh =
      (datalayer.battery.status.real_soc / 10000.0) *
      datalayer.battery.info.total_capacity_Wh;

  // === Update extended datalayer for web UI ===
  datalayer_extended.chademo.CHADEMO_Status = static_cast<uint8_t>(currentState);
  datalayer_extended.chademo.ControlProtocolNumberEV =
      vehicleSession.controlProtocolNumber;
  datalayer_extended.chademo.FaultBatteryVoltageDeviation =
      vehicleSession.faultBatteryVoltageDeviation;
  datalayer_extended.chademo.FaultHighBatteryTemperature =
      vehicleSession.faultHighBatteryTemperature;
  datalayer_extended.chademo.FaultBatteryCurrentDeviation =
      vehicleSession.faultBatteryCurrentDeviation;
  datalayer_extended.chademo.FaultBatteryUnderVoltage =
      vehicleSession.faultBatteryUnderVoltage;
  datalayer_extended.chademo.FaultBatteryOverVoltage =
      vehicleSession.faultBatteryOverVoltage;
}

/* ============================================================================
 *                         CONTROL FUNCTIONS
 * ============================================================================
 */

void ChademoV2HBattery::chademo_restart() {
  datalayer_extended.chademo.UserRequestRestart = true;
}

void ChademoV2HBattery::chademo_stop() {
  datalayer_extended.chademo.UserRequestStop = true;
}
