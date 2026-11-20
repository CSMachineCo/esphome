import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import (
    CONF_ZIGBEE_WIFI_BRIDGE_ID,
    ZigbeeWifiBridgeComponent,
    zigbee_wifi_bridge_ns,
)

CONF_BENCH_ISOLATION = "bench_isolation"

ZigbeeBridgeBenchIsolationSwitch = zigbee_wifi_bridge_ns.class_(
    "ZigbeeBridgeBenchIsolationSwitch", switch.Switch, cg.Parented.template(ZigbeeWifiBridgeComponent)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_BENCH_ISOLATION): switch.switch_schema(
            ZigbeeBridgeBenchIsolationSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:shield-lock",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    if CONF_BENCH_ISOLATION in config:
        sw = await switch.new_switch(config[CONF_BENCH_ISOLATION])
        await cg.register_parented(sw, config[CONF_ZIGBEE_WIFI_BRIDGE_ID])
        cg.add(hub.set_bench_isolation_switch(sw))
