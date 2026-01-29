#ifndef __HW_6CH_RELAY_S3_H__
#define __HW_6CH_RELAY_S3_H__

#include "hal.h"
#include "../utils/types.h"

/**
 * @brief HAL configuration for Kraftkranen v2.0 CHAdeMO V2H Controller
 *
 * Hardware: Waveshare ESP32-S3-Relay-6CH
 * - ESP32-S3 with 16MB Flash, 8MB PSRAM
 * - 6x 12V relay outputs (active high, normally open)
 * - Native CAN interface
 * - RS485 interface
 *
 * Power Architecture:
 * - 12V external adapter powers board via VSYS/Screw terminals
 * - 12V VCC bridged to relay COM terminals for external components
 * - Common GND shared: Waveshare, CHAdeMO Pin 1, Nissan Junction Box coils
 *
 * ============================================================================
 *                         RELAY OUTPUT MAPPING
 * ============================================================================
 * All relays are Normally Open (NO), switching 12V to external components.
 *
 * Relay 1 (GPIO 4):  Nissan Main Relay (+) Coil - High Voltage Positive
 * Relay 2 (GPIO 5):  Nissan Main Relay (-) Coil - High Voltage Negative
 * Relay 3 (GPIO 6):  Nissan Precharge Relay Coil - Bypasses Main+ with resistor
 * Relay 4 (GPIO 7):  CHAdeMO Pin 2 (d1) - "Charger Start" 12V signal to car
 * Relay 5 (GPIO 15): CHAdeMO Pin 7 - Connector Lock solenoid
 * Relay 6 (GPIO 16): Spare (available for cooling fan)
 *
 * ============================================================================
 *                         INPUT MAPPING
 * ============================================================================
 * CAN Bus:
 * - CAN TX: GPIO 17 -> CHAdeMO Pin 9 (CAN-L)
 * - CAN RX: GPIO 18 -> CHAdeMO Pin 8 (CAN-H)
 *
 * Vehicle Enable (CHAdeMO Pin 4):
 * - 12V signal from car when ready
 * - Voltage divider (10k/3.3k) steps down to 3.3V
 * - Connected to GPIO 9 (input)
 *
 * Current Sensor (ACS758 50A/100A):
 * - Hall effect sensor on main DC line
 * - Connected to GPIO 10 (ADC1_CH9)
 *
 * ============================================================================
 *                      PRECHARGE SEQUENCE (Kraftkranen)
 * ============================================================================
 * 1. HANDSHAKE: Relay 5 (Lock) -> Relay 4 (Start Signal)
 * 2. WAIT:      Wait for CAN response + 12V on Vehicle Enable input
 * 3. PRECHARGE: Relay 2 (Main Neg) + Relay 3 (Precharge)
 * 4. STABILIZE: Wait 3 seconds for Deye caps to charge via resistor
 * 5. RUN:       Relay 1 (Main Pos) -> Disengage Relay 3 (Precharge)
 * ============================================================================
 */

class Relay6chS3Hal : public Esp32Hal {
 public:
  const char* name() { return "Kraftkranen v2.0 (6CH Relay S3)"; }

  virtual void set_default_configuration_values() {
    BatteryEmulatorSettingsStore settings;
    if (!settings.settingExists("CANFREQ")) {
      settings.saveUInt("CANFREQ", 16);  // 500kbps default for CHAdeMO
    }
  }

  // ==================== CAN Bus ====================
  // Native CAN on ESP32-S3 -> CHAdeMO Pins 8 (H) & 9 (L)
  virtual gpio_num_t CAN_TX_PIN() { return GPIO_NUM_17; }
  virtual gpio_num_t CAN_RX_PIN() { return GPIO_NUM_18; }

  // ==================== RS485 Interface ====================
  virtual gpio_num_t RS485_TX_PIN() { return GPIO_NUM_43; }
  virtual gpio_num_t RS485_RX_PIN() { return GPIO_NUM_44; }
  virtual gpio_num_t RS485_SE_PIN() { return GPIO_NUM_8; }  // Direction control

  // ==================== MCP2515 CAN Add-on (SPI) ====================
  // Optional secondary CAN via external MCP2515 module
  virtual gpio_num_t MCP2515_SCK() { return GPIO_NUM_36; }
  virtual gpio_num_t MCP2515_MOSI() { return GPIO_NUM_35; }
  virtual gpio_num_t MCP2515_MISO() { return GPIO_NUM_37; }
  virtual gpio_num_t MCP2515_CS() { return GPIO_NUM_39; }
  virtual gpio_num_t MCP2515_INT() { return GPIO_NUM_40; }

  // ==================== Contactor Relays (Nissan Junction Box) ====================
  virtual gpio_num_t POSITIVE_CONTACTOR_PIN() { return GPIO_NUM_4; }   // Relay 1: Main+
  virtual gpio_num_t NEGATIVE_CONTACTOR_PIN() { return GPIO_NUM_5; }   // Relay 2: Main-
  virtual gpio_num_t PRECHARGE_PIN() { return GPIO_NUM_6; }            // Relay 3: Precharge

  // ==================== CHAdeMO Signal Relays ====================
  // These relays switch 12V signals to the CHAdeMO connector
  virtual gpio_num_t CHADEMO_PIN_2() { return GPIO_NUM_7; }   // Relay 4: d1 "Charger Start"
  virtual gpio_num_t CHADEMO_LOCK() { return GPIO_NUM_15; }   // Relay 5: Pin 7 Lock solenoid

  // ==================== CHAdeMO Input Signals ====================
  // Vehicle Enable: CHAdeMO Pin 4 (j) - 12V through voltage divider
  virtual gpio_num_t CHADEMO_PIN_4() { return GPIO_NUM_9; }   // INPUT: Vehicle permission

  // ==================== Current Sensor ====================
  // ACS758 Hall sensor on main DC line (analog output)
  virtual gpio_num_t CURRENT_SENSOR_PIN() { return GPIO_NUM_10; }  // ADC1_CH9

  // ==================== Spare Relay ====================
  // Relay 6: Available for cooling fan or future expansion
  virtual gpio_num_t SPARE_RELAY_PIN() { return GPIO_NUM_16; }  // Relay 6

  // ==================== LED Indicator ====================
  virtual gpio_num_t LED_PIN() { return GPIO_NUM_38; }
  virtual uint8_t LED_MAX_BRIGHTNESS() { return 255; }

  // ==================== I2C Display (optional) ====================
  virtual gpio_num_t DISPLAY_SDA_PIN() { return GPIO_NUM_1; }
  virtual gpio_num_t DISPLAY_SCL_PIN() { return GPIO_NUM_2; }

  // ==================== Equipment Stop ====================
  virtual gpio_num_t EQUIPMENT_STOP_PIN() { return GPIO_NUM_41; }

  // ==================== Unused/Reserved ====================
  // These pins are available on headers for future expansion
  virtual gpio_num_t WUP_PIN1() { return GPIO_NUM_13; }
  virtual gpio_num_t WUP_PIN2() { return GPIO_NUM_14; }

  // CHAdeMO Pin 10 (k) not used in Kraftkranen design
  virtual gpio_num_t CHADEMO_PIN_10() { return GPIO_NUM_NC; }
  // CHAdeMO Pin 7 (PP) proximity not used - using lock relay instead
  virtual gpio_num_t CHADEMO_PIN_7() { return GPIO_NUM_NC; }

  // BMS_POWER not used - Relay 4 repurposed for CHAdeMO Pin 2
  virtual gpio_num_t BMS_POWER() { return GPIO_NUM_NC; }

  std::vector<comm_interface> available_interfaces() {
    return {comm_interface::CanNative, comm_interface::CanAddonMcp2515,
            comm_interface::RS485, comm_interface::Modbus};
  }

  virtual const char* name_for_comm_interface(comm_interface comm) {
    switch (comm) {
      case comm_interface::CanNative:
        return "CAN (CHAdeMO)";
      case comm_interface::CanAddonMcp2515:
        return "CAN (MCP2515 add-on)";
      case comm_interface::RS485:
        return "RS485";
      case comm_interface::Modbus:
        return "Modbus";
      default:
        return Esp32Hal::name_for_comm_interface(comm);
    }
  }
};

#define HalClass Relay6chS3Hal

/* ----- Error checks below, don't change ----- */
#ifndef HW_CONFIGURED
#define HW_CONFIGURED
#else
#error Multiple HW defined! Please select a single HW
#endif

#endif
