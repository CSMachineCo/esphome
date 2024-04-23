import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)

DEPENDENCIES = ["lora_uart"]
lora_signal_ns = cg.esphome_ns.namespace("lora_signal")
LoRaSignalSensor = lora_signal_ns.class_(
    "LoRaSignalSensor", sensor.Sensor, cg.PollingComponent
)

CONFIG_SCHEMA = sensor.sensor_schema(
    LoRaSignalSensor,
    unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(cv.polling_component_schema("10s"))


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
