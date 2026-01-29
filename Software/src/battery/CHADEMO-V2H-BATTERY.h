/**
 * @file CHADEMO-V2H-BATTERY.h
 * @brief CHAdeMO V2H (Vehicle-to-Home) Bidirectional Protocol Implementation
 *
 * This module implements the CHAdeMO protocol specifically optimized for V2H
 * (Vehicle-to-Home) discharge operations. It enables an EV battery to supply
 * power to a home inverter through proper CHAdeMO handshaking.
 *
 * ============================================================================
 *                              SAFETY WARNING
 * ============================================================================
 * This module controls HIGH VOLTAGE DC systems (typically 300-450V).
 * Improper use can result in DEATH, FIRE, or EQUIPMENT DAMAGE.
 *
 * MANDATORY REQUIREMENTS:
 * 1. External precharge circuit with appropriate resistor (typically 50-100 ohm)
 * 2. Properly rated contactors (DC rated, >500V, appropriate current rating)
 * 3. Isolation monitoring before energizing
 * 4. Physical interlock mechanisms
 * 5. Emergency stop capability
 * 6. Qualified personnel for installation and testing
 *
 * DO NOT operate without proper safety circuits and testing!
 * ============================================================================
 *
 * @author Based on dalathegreat's Battery-Emulator CHAdeMO implementation
 * @version 1.0
 * @date 2024
 */

#ifndef CHADEMO_V2H_BATTERY_H
#define CHADEMO_V2H_BATTERY_H

#include <Arduino.h>
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/hal/hal.h"
#include "CanBattery.h"

/* ============================================================================
 *                    GPIO PIN DEFINITIONS - WAVESHARE ESP32
 * ============================================================================
 * These pins are suggested defaults for Waveshare ESP32 boards.
 * Adjust according to your specific hardware configuration.
 *
 * CHAdeMO Connector Pin Reference:
 * - Pin 2 (d1): EVSE ready signal output (EVSE -> Vehicle)
 * - Pin 4 (j):  Charge permission input (Vehicle -> EVSE)
 * - Pin 7 (PP): Proximity/plug detection input
 * - Pin 10 (k): Charge enable output (EVSE -> Vehicle)
 * - Lock:       Connector lock solenoid control
 */

// Default GPIO mappings for Waveshare ESP32-S3
// These can be overridden in platformio.ini or hardware HAL
#ifndef CHADEMO_V2H_PIN_2_DEFAULT
#define CHADEMO_V2H_PIN_2_DEFAULT   GPIO_NUM_4   // EVSE Ready (d1) - OUTPUT
#endif

#ifndef CHADEMO_V2H_PIN_4_DEFAULT
#define CHADEMO_V2H_PIN_4_DEFAULT   GPIO_NUM_5   // Vehicle Permission (j) - INPUT
#endif

#ifndef CHADEMO_V2H_PIN_7_DEFAULT
#define CHADEMO_V2H_PIN_7_DEFAULT   GPIO_NUM_6   // Proximity Detection (PP) - INPUT
#endif

#ifndef CHADEMO_V2H_PIN_10_DEFAULT
#define CHADEMO_V2H_PIN_10_DEFAULT  GPIO_NUM_7   // Charge Enable (k) - OUTPUT
#endif

#ifndef CHADEMO_V2H_LOCK_DEFAULT
#define CHADEMO_V2H_LOCK_DEFAULT    GPIO_NUM_15  // Connector Lock - OUTPUT
#endif

// Precharge and Contactor pins (typically shared with main system)
#ifndef CHADEMO_V2H_PRECHARGE_DEFAULT
#define CHADEMO_V2H_PRECHARGE_DEFAULT          GPIO_NUM_16  // Precharge relay - OUTPUT
#endif

#ifndef CHADEMO_V2H_POSITIVE_CONTACTOR_DEFAULT
#define CHADEMO_V2H_POSITIVE_CONTACTOR_DEFAULT GPIO_NUM_17  // Main positive contactor - OUTPUT
#endif

/* ============================================================================
 *                         TIMING CONSTANTS
 * ============================================================================
 * CHAdeMO protocol has strict timing requirements. These must be adhered to
 * for proper handshaking and safe operation.
 *
 * Reference: CHAdeMO 2.0 Protocol Specification
 */

// CAN message intervals (milliseconds)
#define CHADEMO_V2H_CAN_INTERVAL_MS       100   // Standard 100ms CAN cycle
#define CHADEMO_V2H_FAST_CAN_INTERVAL_MS  10    // Fast mode during critical states

// State transition timeouts (milliseconds)
#define CHADEMO_V2H_INIT_TIMEOUT_MS       5000  // Max wait for vehicle CAN
#define CHADEMO_V2H_NEGOTIATE_TIMEOUT_MS  10000 // Max negotiation time
#define CHADEMO_V2H_PRECHARGE_TIMEOUT_MS  5000  // Max precharge duration
#define CHADEMO_V2H_STOP_TIMEOUT_MS       3000  // Max time to reach safe state

// Voltage/current thresholds
#define CHADEMO_V2H_SAFE_VOLTAGE_V        20    // Voltage considered safe for contactor ops
#define CHADEMO_V2H_SAFE_CURRENT_A        5     // Current considered safe for disconnect
#define CHADEMO_V2H_VOLTAGE_TOLERANCE_PCT 2     // Threshold voltage tolerance percentage

/* ============================================================================
 *                      V2H EVSE CAPABILITY DEFAULTS
 * ============================================================================
 * These define what the EVSE (this device) advertises to the vehicle.
 * Adjust based on your inverter's actual capabilities.
 */
#define CHADEMO_V2H_MAX_OUTPUT_VOLTAGE_V  410   // Maximum voltage EVSE can handle
#define CHADEMO_V2H_MAX_OUTPUT_CURRENT_A  32    // Maximum discharge current
#define CHADEMO_V2H_MAX_POWER_W           10000 // Maximum power (approx 10kW)
#define CHADEMO_V2H_MIN_VOLTAGE_V         200   // Minimum operating voltage

/* ============================================================================
 *                         STATE MACHINE STATES
 * ============================================================================
 * The V2H discharge sequence follows these states according to CHAdeMO spec.
 * State progression: IDLE -> CONNECTED -> INIT -> NEGOTIATE -> EV_ALLOWED ->
 *                    EVSE_PREPARE -> EVSE_START -> CONTACTORS -> POWERFLOW
 */
enum ChademoV2HState : uint8_t {
  V2H_FAULT             = 0,   // Fault condition - requires power cycle to clear
  V2H_STOP              = 1,   // Controlled shutdown in progress
  V2H_IDLE              = 2,   // Waiting for plug insertion
  V2H_CONNECTED         = 3,   // Plug inserted, not yet communicating
  V2H_INIT              = 4,   // Awaiting first vehicle CAN message
  V2H_NEGOTIATE         = 5,   // Exchanging capabilities with vehicle
  V2H_EV_ALLOWED        = 6,   // Vehicle has granted permission
  V2H_EVSE_PREPARE      = 7,   // EVSE preparing (insulation test, etc.)
  V2H_EVSE_START        = 8,   // EVSE signaling start
  V2H_EVSE_CONTACTORS   = 9,   // Contactor closure sequence
  V2H_POWERFLOW         = 10,  // Active power transfer (discharge)
  V2H_STATE_COUNT       = 11   // Number of states (for bounds checking)
};

/* ============================================================================
 *                           PROTOCOL VERSION
 * ============================================================================
 */
enum ChademoV2HProtocolVersion : uint8_t {
  CHADEMO_V2H_PROTO_V1_0      = 0x01,  // CHAdeMO 1.0 (basic)
  CHADEMO_V2H_PROTO_V1_1      = 0x02,  // CHAdeMO 1.1 (V2H support added)
  CHADEMO_V2H_PROTO_V2_0      = 0x03,  // CHAdeMO 2.0 (enhanced V2H)
  CHADEMO_V2H_PROTO_V2_0_APP_A = 0x04, // CHAdeMO 2.0 Appendix A
  CHADEMO_V2H_PROTO_V2_0_APP_B = 0x05  // CHAdeMO 2.0 Appendix B (dynamic control)
};

/**
 * @class ChademoV2HBattery
 * @brief CHAdeMO V2H bidirectional discharge controller
 *
 * This class implements the EVSE side of CHAdeMO V2H protocol, allowing
 * an EV battery to discharge to a home inverter system.
 */
class ChademoV2HBattery : public CanBattery {
public:
  ChademoV2HBattery();

  // Core Battery interface implementation
  virtual void setup(void) override;
  virtual void handle_incoming_can_frame(CAN_frame rx_frame) override;
  virtual void update_values() override;
  virtual void transmit_can(unsigned long currentMillis) override;

  // V2H specific controls
  bool supports_chademo_restart() override { return true; }
  bool supports_chademo_stop() override { return true; }

  void chademo_restart() override;
  void chademo_stop() override;

  BatteryHtmlRenderer& get_status_renderer() override { return renderer; }

  static constexpr const char* Name = "CHAdeMO V2H Discharge Mode";

private:
  /* ========== GPIO Pins ========== */
  gpio_num_t pin_d1;           // Pin 2 - EVSE ready signal (output)
  gpio_num_t pin_j;            // Pin 4 - Vehicle permission (input)
  gpio_num_t pin_pp;           // Pin 7 - Proximity detection (input)
  gpio_num_t pin_k;            // Pin 10 - Charge enable (output)
  gpio_num_t pin_lock;         // Connector lock solenoid (output)
  gpio_num_t pin_precharge;    // Precharge relay (output)
  gpio_num_t pin_contactor;    // Main contactor (output)

  /* ========== State Machine ========== */
  ChademoV2HState currentState;
  ChademoV2HState previousState;
  unsigned long stateEntryTime;
  unsigned long lastCanRxTime;
  unsigned long lastCanTxTime;

  /* ========== Flags ========== */
  bool plugInserted;
  bool vehicleCanReceived;
  bool vehicleCanInitialized;
  bool vehiclePermission;
  bool evsePermission;
  bool dischargeCompatible;
  bool contactorsReady;
  uint8_t frameCounter;

  /* ========== Vehicle Data Structures (Received from EV) ========== */

  // H100 - Vehicle charging/discharging minimums
  struct {
    uint8_t  minimumChargeCurrent;        // Byte 0: Minimum current (A)
    uint16_t minimumBatteryVoltage;       // Bytes 2-3: Min voltage (V)
    uint16_t maximumBatteryVoltage;       // Bytes 4-5: Max voltage (V)
    uint8_t  constantOfChargingRate;      // Byte 6: Charging rate constant
  } vehicleChargeLimits;

  // H101 - Vehicle charging estimates
  struct {
    uint8_t  maxChargingTime10s;          // Byte 1: Max charge time (10s units)
    uint8_t  maxChargingTime1min;         // Byte 2: Max charge time (1min units)
    uint8_t  estimatedChargingTime;       // Byte 3: Estimated time to full
    uint16_t ratedBatteryCapacity;        // Bytes 5-6: Capacity (0.1 kWh units)
  } vehicleChargeEstimate;

  // H102 - Vehicle charging session status (CRITICAL for V2H)
  struct {
    uint8_t  controlProtocolNumber;       // Byte 0: Protocol version
    uint16_t targetBatteryVoltage;        // Bytes 1-2: Target voltage (V)
    uint8_t  chargingCurrentRequest;      // Byte 3: Current request (A)

    // Byte 4: Fault flags
    bool faultBatteryOverVoltage;
    bool faultBatteryUnderVoltage;
    bool faultBatteryCurrentDeviation;
    bool faultHighBatteryTemperature;
    bool faultBatteryVoltageDeviation;

    // Byte 5: Status flags
    bool statusVehicleChargingEnabled;    // Bit 0: Vehicle allows charging
    bool statusVehicleShifterPosition;    // Bit 1: Vehicle not in park = stop
    bool statusChargingError;             // Bit 2: Vehicle charging error
    bool statusVehicle;                   // Bit 3: Vehicle status
    bool statusNormalStopRequest;         // Bit 4: Normal stop requested
    bool statusVehicleDischargeCompatible;// Bit 7: V2H DISCHARGE COMPATIBLE!

    uint8_t stateOfCharge;                // Byte 6: SOC (%)
  } vehicleSession;

  // H200 - Vehicle discharge limits (V2H specific)
  struct {
    uint8_t  maximumDischargeCurrent;     // Byte 0: Max discharge current (A)
    uint16_t minimumDischargeVoltage;     // Bytes 4-5: Min discharge voltage (V)
    uint8_t  minimumBatteryDischargeLevel;// Byte 6: Min SOC for discharge (%)
    uint8_t  maxRemainingCapacityForChg;  // Byte 7: Max SOC to allow charging
  } vehicleDischargeLimits;

  // H201 - Vehicle discharge estimate
  struct {
    uint8_t  v2hSequenceNumber;           // Byte 0: V2H sequence control
    uint16_t approxDischargeCompletionTime;// Bytes 1-2: Est. time to min SOC
    uint16_t availableVehicleEnergy;      // Bytes 3-4: Available energy (0.1kWh)
  } vehicleDischargeEstimate;

  // H110 - Vehicle dynamic control (CHAdeMO 2.0)
  struct {
    bool dynamicControlSupport;           // Bit 0: Dynamic control supported
    bool highCurrentControlSupport;       // Bit 1: High current control
    bool highVoltageControlSupport;       // Bit 2: High voltage control
  } vehicleDynamicControl;

  /* ========== EVSE Data Structures (Sent to EV) ========== */

  // H108 - EVSE capabilities
  struct {
    uint8_t  contactorWeldDetection;      // Byte 0: Weld detection support
    uint16_t availableOutputVoltage;      // Bytes 1-2: Max voltage (V)
    uint8_t  availableOutputCurrent;      // Byte 3: Max current (A)
    uint16_t thresholdVoltage;            // Bytes 4-5: Upper voltage threshold
  } evseCapabilities;

  // H109 - EVSE status
  struct {
    uint8_t  protocolNumber;              // Byte 0: Protocol version
    uint16_t setpointVoltage;             // Bytes 1-2: Output voltage setpoint
    uint8_t  setpointCurrent;             // Byte 3: Output current setpoint
    bool     dischargeCompatible;         // Byte 4, Bit 0: V2H COMPATIBLE!

    // Byte 5: Status bits
    bool     evseStatus;                  // Bit 0: EVSE ready
    bool     evseError;                   // Bit 1: EVSE error
    bool     connectorLocked;             // Bit 2: Connector lock engaged
    bool     batteryIncompatible;         // Bit 3: Incompatible battery
    bool     chgDischError;               // Bit 4: Charge/discharge error
    bool     chgDischStopControl;         // Bit 5: Stop charging/discharging

    uint8_t  remainingTime10s;            // Byte 6: Remaining time (10s)
    uint8_t  remainingTime1min;           // Byte 7: Remaining time (1min)
  } evseStatus;

  // H208 - EVSE discharge capabilities (V2H specific)
  struct {
    uint8_t  presentDischargeCurrent;     // Byte 0: Current discharge rate
    uint16_t availableInputVoltage;       // Bytes 1-2: Min input voltage
    uint8_t  availableInputCurrent;       // Byte 3: Max input current
    uint16_t lowerThresholdVoltage;       // Bytes 6-7: Lower voltage threshold
  } evseDischargeCapabilities;

  // H209 - EVSE discharge estimate
  struct {
    uint8_t  sequenceControlNumber;       // Byte 0: Sequence number
    uint16_t remainingDischargeTime;      // Bytes 1-2: Est. remaining time
  } evseDischargeEstimate;

  /* ========== CAN Frames ========== */
  CAN_frame canFrame_108;  // EVSE Capabilities
  CAN_frame canFrame_109;  // EVSE Status
  CAN_frame canFrame_118;  // EVSE Dynamic Control (V2.0)
  CAN_frame canFrame_208;  // EVSE Discharge Capabilities
  CAN_frame canFrame_209;  // EVSE Discharge Estimate

  /* ========== Measured Values (from shunt/sensors) ========== */
  uint16_t measuredVoltage;
  int16_t  measuredCurrent;

  /* ========== HTML Renderer ========== */
  BatteryDefaultRenderer renderer;

  /* ========== Private Methods ========== */

  // CAN message processors
  void processVehicleChargeLimits(const CAN_frame& frame);      // 0x100
  void processVehicleChargeEstimate(const CAN_frame& frame);    // 0x101
  void processVehicleSession(const CAN_frame& frame);           // 0x102
  void processVehicleDischargeLimits(const CAN_frame& frame);   // 0x200
  void processVehicleDischargeEstimate(const CAN_frame& frame); // 0x201
  void processVehicleDynamicControl(const CAN_frame& frame);    // 0x110

  // CAN message builders
  void buildEvseCapabilities();     // 0x108
  void buildEvseStatus();           // 0x109
  void buildEvseDischargeCapabilities(); // 0x208
  void buildEvseDischargeEstimate();     // 0x209

  // State machine
  void runStateMachine();
  void transitionToState(ChademoV2HState newState);
  void handleStateIdle();
  void handleStateConnected();
  void handleStateInit();
  void handleStateNegotiate();
  void handleStateEvAllowed();
  void handleStateEvsePrepare();
  void handleStateEvseStart();
  void handleStateContactors();
  void handleStatePowerflow();
  void handleStateStop();
  void handleStateFault();

  // Safety checks
  bool checkVehicleFaults();
  bool checkVoltageCompatibility();
  bool checkDischargeCompatibility();
  void safetyShutdown(const char* reason);

  // Utility functions
  void initializeEvseStructures();
  uint16_t getMeasuredVoltage();
  int16_t getMeasuredCurrent();
  void updateDataLayer();
};

#endif // CHADEMO_V2H_BATTERY_H
