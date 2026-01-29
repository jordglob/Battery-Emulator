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
  // Initialize GPIO pins from HAL or use defaults
  pin_d1        = esp32hal->CHADEMO_PIN_2();
  pin_j         = esp32hal->CHADEMO_PIN_4();
  pin_pp        = esp32hal->CHADEMO_PIN_7();
  pin_k         = esp32hal->CHADEMO_PIN_10();
  pin_lock      = esp32hal->CHADEMO_LOCK();
  pin_precharge = esp32hal->PRECHARGE_PIN();
  pin_contactor = esp32hal->POSITIVE_CONTACTOR_PIN();

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
  // Allocate GPIO pins - fail gracefully if not available
  if (!esp32hal->alloc_pins(Name, pin_d1, pin_k, pin_j, pin_pp, pin_lock)) {
    logging.println("CHAdeMO V2H: Failed to allocate GPIO pins");
    set_event(EVENT_GPIO_NOT_DEFINED, 0);
    return;
  }

  // Configure output pins
  pinMode(pin_d1, OUTPUT);
  digitalWrite(pin_d1, LOW);   // EVSE not ready initially

  pinMode(pin_k, OUTPUT);
  digitalWrite(pin_k, LOW);    // Charge enable off

  pinMode(pin_lock, OUTPUT);
  digitalWrite(pin_lock, LOW); // Connector unlocked

  // Configure input pins
  pinMode(pin_j, INPUT);       // Vehicle permission signal
  pinMode(pin_pp, INPUT);      // Proximity/plug detection

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
  vehiclePermission = digitalRead(pin_j);

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
 * IDLE State: Waiting for plug insertion
 */
void ChademoV2HBattery::handleStateIdle() {
  // Ensure safe state
  digitalWrite(pin_d1, LOW);
  digitalWrite(pin_k, LOW);
  digitalWrite(pin_lock, LOW);  // Unlock connector

  // Check for plug insertion
  plugInserted = digitalRead(pin_pp);

  if (plugInserted) {
    logging.println("CHAdeMO V2H: Plug inserted");
    transitionToState(V2H_CONNECTED);
  }
}

/**
 * CONNECTED State: Plug inserted, starting communication
 */
void ChademoV2HBattery::handleStateConnected() {
  if (!plugInserted) {
    // Plug removed
    transitionToState(V2H_IDLE);
    return;
  }

  // Assert EVSE ready signal (Pin 2/d1)
  digitalWrite(pin_d1, HIGH);

  // Transition to INIT - waiting for vehicle CAN
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
 */
void ChademoV2HBattery::handleStateEvAllowed() {
  logging.println("CHAdeMO V2H: Vehicle permission granted");

  if (vehiclePermission) {
    // Lock connector
    digitalWrite(pin_lock, HIGH);
    evseStatus.connectorLocked = true;

    // TODO: Add solenoid verification if available

    transitionToState(V2H_EVSE_PREPARE);
  }
}

/**
 * EVSE_PREPARE State: Preparing for power transfer
 */
void ChademoV2HBattery::handleStateEvsePrepare() {
  logging.println("CHAdeMO V2H: EVSE preparing");

  // Verify voltage is safe before enabling
  if (vehicleSession.statusVehicleChargingEnabled) {
    if (getMeasuredVoltage() < CHADEMO_V2H_SAFE_VOLTAGE_V) {
      // Enable charge signal (Pin 10/k)
      digitalWrite(pin_k, HIGH);
      evsePermission = true;
    } else {
      logging.printf("CHAdeMO V2H: Voltage too high for startup: %dV\n",
                     getMeasuredVoltage());
    }

    // Keep stop control asserted during preparation
    evseStatus.chgDischStopControl = true;
    evseStatus.evseStatus = false;
  } else {
    // Vehicle withdrew permission
    transitionToState(V2H_STOP);
  }

  // State transition to EVSE_START happens in processVehicleSession
}

/**
 * EVSE_START State: Signaling start of session
 */
void ChademoV2HBattery::handleStateEvseStart() {
  logging.println("CHAdeMO V2H: Starting session");

  // Allow contactor closure
  datalayer.system.status.battery_allows_contactor_closing = true;

  evseStatus.chgDischStopControl = true;  // Still asserting stop
  evseStatus.evseStatus = false;

  transitionToState(V2H_EVSE_CONTACTORS);
}

/**
 * EVSE_CONTACTORS State: Closing contactors
 */
void ChademoV2HBattery::handleStateContactors() {
  logging.println("CHAdeMO V2H: Contactor sequence");

  // Read contactor status (implementation depends on hardware)
  bool prechargeComplete = (digitalRead(pin_precharge) == LOW);
  bool contactorClosed = (digitalRead(pin_contactor) == HIGH);
  contactorsReady = prechargeComplete && contactorClosed;

  if (contactorsReady) {
    logging.printf("CHAdeMO V2H: Contactors ready, voltage: %dV\n",
                   getMeasuredVoltage());

    // Verify both sides are discharge compatible
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

  // Timeout check
  if (millis() - stateEntryTime > CHADEMO_V2H_PRECHARGE_TIMEOUT_MS) {
    logging.println("CHAdeMO V2H: Contactor timeout");
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
 */
void ChademoV2HBattery::handleStateStop() {
  logging.println("CHAdeMO V2H: Stopping");

  // Assert stop control
  evseStatus.chgDischStopControl = true;
  evseStatus.evseStatus = false;
  evsePermission = false;
  vehiclePermission = false;

  // Prevent contactor closure
  datalayer.system.status.battery_allows_contactor_closing = false;

  // Wait for current to drop before opening contactors
  // Per CHAdeMO spec: protect EV contactors from arc damage
  if (abs(getMeasuredCurrent()) <= CHADEMO_V2H_SAFE_CURRENT_A &&
      getMeasuredVoltage() <= CHADEMO_V2H_SAFE_VOLTAGE_V) {
    // Safe to de-energize
    digitalWrite(pin_k, LOW);   // Disable charge enable
    digitalWrite(pin_d1, LOW);  // EVSE not ready

    transitionToState(V2H_IDLE);
  }

  // Timeout - force shutdown even if current high (safety concern!)
  if (millis() - stateEntryTime > CHADEMO_V2H_STOP_TIMEOUT_MS) {
    logging.println("CHAdeMO V2H: Stop timeout, forcing shutdown");
    digitalWrite(pin_k, LOW);
    digitalWrite(pin_d1, LOW);
    transitionToState(V2H_IDLE);
  }
}

/**
 * FAULT State: Error condition - requires power cycle
 */
void ChademoV2HBattery::handleStateFault() {
  logging.println("CHAdeMO V2H: FAULT state - power cycle required");

  // Set all error flags
  evseStatus.evseError = true;
  evseStatus.chgDischError = true;
  evseStatus.chgDischStopControl = true;

  // De-energize everything
  digitalWrite(pin_k, LOW);
  digitalWrite(pin_d1, LOW);

  evsePermission = false;
  vehiclePermission = false;
  datalayer.system.status.battery_allows_contactor_closing = false;

  // FAULT state can only be cleared by power cycle
  // This is a safety feature per CHAdeMO spec
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
 * Emergency safety shutdown
 */
void ChademoV2HBattery::safetyShutdown(const char* reason) {
  logging.printf("CHAdeMO V2H: SAFETY SHUTDOWN - %s\n", reason);
  digitalWrite(pin_k, LOW);
  digitalWrite(pin_d1, LOW);
  evsePermission = false;
  vehiclePermission = false;
  datalayer.system.status.battery_allows_contactor_closing = false;
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
