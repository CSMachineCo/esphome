#pragma once

#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include <driver/uart.h>
#include <HardwareSerial.h>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "LoraSx1262.h"

namespace esphome {
namespace uart {
//namespace lora_uart{

class ESP32ArduinoLoraUARTComponent : public UARTComponent, public Component {
 public:

  void set_rssi_sensor(sensor::Sensor *rssi_sensor) { rssi_sensor_ = rssi_sensor; }
  void set_snr_sensor(sensor::Sensor *snr_sensor) { snr_sensor_ = snr_sensor; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void write_array(const uint8_t *data, size_t len) override;

  bool peek_byte(uint8_t *data) override;

  // @param data Pointer to the array where the read data will be stored.
  // @param len Number of bytes to read.
  // @return True if the specified number of bytes were successfully read, false otherwise.
  bool read_array(uint8_t *data, size_t len) override;

  int available() override;
  void flush() override;

  void set_nss_pin(InternalGPIOPin *pin) {_pin_NSS = pin;}
  void set_sclk_pin(InternalGPIOPin *pin) {_pin_SCK = pin;}  
  void set_reset_pin(InternalGPIOPin *pin) {_pin_RESET = pin;}
  void set_dio1_pin(InternalGPIOPin *pin) {_pin_DIO1 = pin;}

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
  int radio_init = 0;
  sensor::Sensor *rssi_sensor_{nullptr};
  sensor::Sensor *snr_sensor_{nullptr};

  void check_logger_conflict() override;

  //this funtion reads available data from radio and loads it into the read buffer
  void read_radio();

  //don't need these...maybe
  //HardwareSerial *hw_serial_{nullptr};
  uint8_t number_{0};

  InternalGPIOPin *_pin_NSS, *_pin_SCK, *_pin_RESET, *_pin_DIO1;

  //internal buffers since radio driver doesn't do it
  uint8_t temp_buffer_[256];
  uint8_t read_buffer_[512];
  int buff_write_ptr_, buff_read_ptr_;

};

//}  // namesapce lora_uart
}  // namespace uart
}  // namespace esphome

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
