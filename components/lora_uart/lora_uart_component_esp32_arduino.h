#pragma once

#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include <driver/uart.h>
#include <HardwareSerial.h>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "uart.h"
#include "LoraSx1262.h"

namespace esphome {
namespace uart {

class ESP32ArduinoLoraUARTComponent : public UARTComponent, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void write_array(const uint8_t *data, size_t len) override;

  bool peek_byte(uint8_t *data) override;
  bool read_array(uint8_t *data, size_t len) override;

  int available() override;
  void flush() override;

  uint32_t get_config();

  HardwareSerial *get_hw_serial() {
    //going to return NULL since we don't have a HW Serial
    //not sure what this will impact (probably don't use with logger?) 
    //return this->hw_serial_;
    return NULL; }
  uint8_t get_hw_serial_number() { 
    //going to return 0 since we don't have a HW Serial
    //not sure what this will impact (probably don't use with logger?) 
    //return this->number_; 
    return 0;   }

  /**
   * Load the UART with the current settings.
   * @param dump_config (Optional, default `true`): True for displaying new settings or
   * false to change it quitely
   *
   * Example:
   * ```cpp
   * id(uart1).load_settings();
   * ```
   *
   * This will load the current UART interface with the latest settings (baud_rate, parity, etc).
   */
  void load_settings(bool dump_config) override;
  void load_settings() override { this->load_settings(true); }

 protected:
  LoraSx1262 radio;
  void check_logger_conflict() override;

  //don't need these...maybe
  //HardwareSerial *hw_serial_{nullptr};
  uint8_t number_{0};
};

}  // namespace uart
}  // namespace esphome

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
