import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_ZIGBEE_WIFI_BRIDGE_ID, ZigbeeWifiBridgeComponent

CONF_TUNNEL_CONNECTED = "tunnel_connected"
CONF_C6_LINK = "c6_link"
CONF_C6_JOINED = "c6_joined"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_TUNNEL_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
        cv.Optional(CONF_C6_LINK): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_C6_JOINED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    if CONF_TUNNEL_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_TUNNEL_CONNECTED])
        cg.add(hub.set_tunnel_connected_binary_sensor(sens))

    if CONF_C6_LINK in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_C6_LINK])
        cg.add(hub.set_c6_link_binary_sensor(sens))

    if CONF_C6_JOINED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_C6_JOINED])
        cg.add(hub.set_c6_joined_binary_sensor(sens))
