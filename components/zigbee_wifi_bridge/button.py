import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import DEVICE_CLASS_RESTART, ENTITY_CATEGORY_CONFIG

from . import (
    CONF_ZIGBEE_WIFI_BRIDGE_ID,
    ZigbeeWifiBridgeComponent,
    zigbee_wifi_bridge_ns,
)

CONF_FACTORY_RESET_ZIGBEE = "factory_reset_zigbee"

ZigbeeBridgeFactoryResetButton = zigbee_wifi_bridge_ns.class_(
    "ZigbeeBridgeFactoryResetButton", button.Button, cg.Parented.template(ZigbeeWifiBridgeComponent)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_FACTORY_RESET_ZIGBEE): button.button_schema(
            ZigbeeBridgeFactoryResetButton,
            device_class=DEVICE_CLASS_RESTART,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:nuke",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    if CONF_FACTORY_RESET_ZIGBEE in config:
        btn = await button.new_button(config[CONF_FACTORY_RESET_ZIGBEE])
        await cg.register_parented(btn, config[CONF_ZIGBEE_WIFI_BRIDGE_ID])
        cg.add(hub.set_factory_reset_button(btn))
