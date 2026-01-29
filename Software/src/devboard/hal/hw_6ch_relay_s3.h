#ifndef __HW_6CH_RELAY_S3_H__
#define __HW_6CH_RELAY_S3_H__

#include "hal.h"
#include "../utils/types.h"

/**
 * @brief HAL configuration for Waveshare ESP32-S3 6-Channel Relay Board
 *
 * This board features:
 * - ESP32-S3 with 16MB Flash, 8MB PSRAM
 * - 6 relay outputs (active high)
 * - RS485 interface
 * - CAN interface (directly exposed)
 * - Multiple GPIO headers
 *
 * Default GPIO assignments (adjust based on your specific board revision):
 *
 * Relay Outputs (directly usable for contactors):
 * - Relay 1: GPIO 4  -> POSITIVE_CONTACTOR
 * - Relay 2: GPIO 5  -> NEGATIVE_CONTACTOR
 * - Relay 3: GPIO 6  -> PRECHARGE
 * - Relay 4: GPIO 7  -> BMS_POWER
 * - Relay 5: GPIO 15 -> CHADEMO_LOCK / EXTRA
 * - Relay 6: GPIO 16 -> INVERTER_CONTACTOR_ENABLE
 *
 * CAN Bus (directly exposed):
 * - CAN TX: GPIO 17
 * - CAN RX: GPIO 18
 *
 * RS485:
 * - TX: GPIO 43
 * - RX: GPIO 44
 * - SE: GPIO 8 (direction control)
 *
 * CHAdeMO signals:
 * - Pin 2 (d1): GPIO 9
 * - Pin 10 (k): GPIO 10
 * - Pin 7 (PP): GPIO 11
 * - Pin 4 (j): GPIO 12
 * - Lock: GPIO 15 (Relay 5)
 *
 * LED: GPIO 38 (directly exposed)
 */

class Relay6chS3Hal : public Esp32Hal {
 public:
  const char* name() { return "Waveshare 6CH Relay S3"; }

  virtual void set_default_configuration_values() {
    BatteryEmulatorSettingsStore settings;
    if (!settings.settingExists("CANFREQ")) {
      settings.saveUInt("CANFREQ", 16);  // 500kbps default
    }
  }

  // Native CAN on ESP32-S3
  virtual gpio_num_t CAN_TX_PIN() { return GPIO_NUM_17; }
  virtual gpio_num_t CAN_RX_PIN() { return GPIO_NUM_18; }

  // RS485 interface
  virtual gpio_num_t RS485_TX_PIN() { return GPIO_NUM_43; }
  virtual gpio_num_t RS485_RX_PIN() { return GPIO_NUM_44; }
  virtual gpio_num_t RS485_SE_PIN() { return GPIO_NUM_8; }  // Direction control

  // MCP2515 CAN add-on (directly exposed SPI pins)
  virtual gpio_num_t MCP2515_SCK() { return GPIO_NUM_36; }
  virtual gpio_num_t MCP2515_MOSI() { return GPIO_NUM_35; }
  virtual gpio_num_t MCP2515_MISO() { return GPIO_NUM_37; }
  virtual gpio_num_t MCP2515_CS() { return GPIO_NUM_39; }
  virtual gpio_num_t MCP2515_INT() { return GPIO_NUM_40; }

  // Contactor handling - using relay outputs
  virtual gpio_num_t POSITIVE_CONTACTOR_PIN() { return GPIO_NUM_4; }   // Relay 1
  virtual gpio_num_t NEGATIVE_CONTACTOR_PIN() { return GPIO_NUM_5; }   // Relay 2
  virtual gpio_num_t PRECHARGE_PIN() { return GPIO_NUM_6; }            // Relay 3
  virtual gpio_num_t BMS_POWER() { return GPIO_NUM_7; }                // Relay 4

  // Inverter contactor enable
  virtual gpio_num_t INVERTER_CONTACTOR_ENABLE_PIN() { return GPIO_NUM_16; }  // Relay 6

  // CHAdeMO support pins
  virtual gpio_num_t CHADEMO_PIN_2() { return GPIO_NUM_9; }    // d1 - EVSE ready
  virtual gpio_num_t CHADEMO_PIN_10() { return GPIO_NUM_10; }  // k - Charge enable
  virtual gpio_num_t CHADEMO_PIN_7() { return GPIO_NUM_11; }   // PP - Proximity
  virtual gpio_num_t CHADEMO_PIN_4() { return GPIO_NUM_12; }   // j - Permission
  virtual gpio_num_t CHADEMO_LOCK() { return GPIO_NUM_15; }    // Relay 5 - Lock solenoid

  // LED indicator
  virtual gpio_num_t LED_PIN() { return GPIO_NUM_38; }
  virtual uint8_t LED_MAX_BRIGHTNESS() { return 255; }

  // I2C display (directly exposed headers)
  virtual gpio_num_t DISPLAY_SDA_PIN() { return GPIO_NUM_1; }
  virtual gpio_num_t DISPLAY_SCL_PIN() { return GPIO_NUM_2; }

  // Equipment stop (directly exposed input)
  virtual gpio_num_t EQUIPMENT_STOP_PIN() { return GPIO_NUM_41; }

  // Battery wake up pins
  virtual gpio_num_t WUP_PIN1() { return GPIO_NUM_13; }
  virtual gpio_num_t WUP_PIN2() { return GPIO_NUM_14; }

  std::vector<comm_interface> available_interfaces() {
    return {comm_interface::Modbus, comm_interface::RS485, comm_interface::CanNative,
            comm_interface::CanAddonMcp2515};
  }

  virtual const char* name_for_comm_interface(comm_interface comm) {
    switch (comm) {
      case comm_interface::CanNative:
        return "CAN (Native ESP32-S3)";
      case comm_interface::CanAddonMcp2515:
        return "CAN (MCP2515 add-on)";
      case comm_interface::Modbus:
        return "Modbus";
      case comm_interface::RS485:
        return "RS485";
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
