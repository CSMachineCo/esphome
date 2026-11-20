import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import (
    CONF_ZIGBEE_WIFI_BRIDGE_ID,
    ZigbeeWifiBridgeComponent,
    zigbee_wifi_bridge_ns,
)

CONF_ZIGBEE_CHANNEL = "zigbee_channel"

ZigbeeBridgeChannelNumber = zigbee_wifi_bridge_ns.class_(
    "ZigbeeBridgeChannelNumber", number.Number, cg.Parented.template(ZigbeeWifiBridgeComponent)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_ZIGBEE_CHANNEL): number.number_schema(
            ZigbeeBridgeChannelNumber,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:radio-tower",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    if CONF_ZIGBEE_CHANNEL in config:
        num = await number.new_number(
            config[CONF_ZIGBEE_CHANNEL],
            min_value=11,
            max_value=26,
            step=1,
        )
        await cg.register_parented(num, config[CONF_ZIGBEE_WIFI_BRIDGE_ID])
        cg.add(hub.set_zigbee_channel_number(num))
