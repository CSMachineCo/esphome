#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/lora_uart/lora_uart_component_esp32_arduino.h"

namespace esphome {
namespace lora_signal {

class LoRaSignalSensor : public sensor::Sensor, public PollingComponent {
 public:
  void update() override { this->publish_state(lora_uart::global_lora_component->lora_rssi()); }
  void dump_config() override;

  std::string unique_id() override { return get_mac_address() + "-lorasignal"; }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
};

}  // namespace wifi_signal
}  // namespace esphome
