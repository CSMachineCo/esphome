import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_ZIGBEE_WIFI_BRIDGE_ID, ZigbeeWifiBridgeComponent

CONF_BRIDGE_ROLE = "bridge_role"
CONF_PEER_IP = "peer_ip"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_BRIDGE_ROLE): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:bridge",
        ),
        cv.Optional(CONF_PEER_IP): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:ip-network",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    if CONF_BRIDGE_ROLE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_BRIDGE_ROLE])
        cg.add(hub.set_bridge_role_text_sensor(sens))

    if CONF_PEER_IP in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PEER_IP])
        cg.add(hub.set_peer_ip_text_sensor(sens))
