#include "lora_signal_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace lora_signal {

static const char *const TAG = "lora_signal.sensor";

void LoRaSignalSensor::dump_config() { LOG_SENSOR("", "LoRa Signal", this); }

}  // namespace lora_signal
}  // namespace esphome
