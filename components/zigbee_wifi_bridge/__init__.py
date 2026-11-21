import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

CODEOWNERS = ["@zigbee-wifi-bridge"]
DEPENDENCIES = ["wifi"]
AUTO_LOAD = ["mdns"]

CONF_ZIGBEE_WIFI_BRIDGE_ID = "zigbee_wifi_bridge_id"

CONF_ROLE = "role"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_RTS_PIN = "rts_pin"
CONF_CTS_PIN = "cts_pin"
CONF_BAUD_RATE = "baud_rate"
CONF_ZIGBEE_CHANNEL = "zigbee_channel"
CONF_MAX_CHILDREN = "max_children"
CONF_BENCH_ISOLATION = "bench_isolation"
CONF_ZIGBEE_ANTENNA = "zigbee_antenna"
CONF_TUNNEL_PORT = "tunnel_port"
CONF_USE_MDNS = "use_mdns"
CONF_MASTER_HOST = "master_host"
CONF_MASTER_STATIC_IP = "master_static_ip"
CONF_TUNNEL_PSK = "tunnel_psk"

zigbee_wifi_bridge_ns = cg.esphome_ns.namespace("zigbee_wifi_bridge")
ZigbeeWifiBridgeComponent = zigbee_wifi_bridge_ns.class_(
    "ZigbeeWifiBridgeComponent", cg.Component
)
BridgeRole = zigbee_wifi_bridge_ns.enum("BridgeRole")
ZigbeeAntenna = zigbee_wifi_bridge_ns.enum("ZigbeeAntenna")

BRIDGE_ROLES = {
    "MASTER": BridgeRole.ROLE_MASTER,
    "SLAVE": BridgeRole.ROLE_SLAVE,
}

ZIGBEE_ANTENNAS = {
    "INTERNAL": ZigbeeAntenna.ANTENNA_INTERNAL,
    "EXTERNAL": ZigbeeAntenna.ANTENNA_EXTERNAL,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZigbeeWifiBridgeComponent),
            cv.Optional(CONF_ROLE, default="MASTER"): cv.enum(BRIDGE_ROLES, upper=True),
            cv.Optional(CONF_TX_PIN, default=9): cv.int_range(min=0, max=21),
            cv.Optional(CONF_RX_PIN, default=10): cv.int_range(min=0, max=21),
            cv.Optional(CONF_RTS_PIN, default=-1): cv.int_range(min=-1, max=21),
            cv.Optional(CONF_CTS_PIN, default=-1): cv.int_range(min=-1, max=21),
            cv.Optional(CONF_BAUD_RATE, default=2000000): cv.int_range(
                min=115200, max=4000000
            ),
            cv.Optional(CONF_ZIGBEE_CHANNEL, default=15): cv.int_range(min=11, max=26),
            cv.Optional(CONF_MAX_CHILDREN, default=16): cv.int_range(min=4, max=32),
            cv.Optional(CONF_BENCH_ISOLATION, default=True): cv.boolean,
            cv.Optional(CONF_ZIGBEE_ANTENNA, default="INTERNAL"): cv.enum(
                ZIGBEE_ANTENNAS, upper=True
            ),
            cv.Optional(CONF_TUNNEL_PORT, default=7842): cv.port,
            cv.Optional(CONF_USE_MDNS, default=True): cv.boolean,
            cv.Optional(CONF_MASTER_HOST, default="zbbridge-master"): cv.string,
            cv.Optional(CONF_MASTER_STATIC_IP, default="192.168.1.100"): cv.string,
            cv.Optional(
                CONF_TUNNEL_PSK, default="change-this-32byte-bridge-secret"
            ): cv.string,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    # ESPHome 2026.2+ excludes unused built-in IDF components by default.
    if hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("mbedtls")
        esp32.include_builtin_idf_component("nvs_flash")
        esp32.include_builtin_idf_component("driver")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_role(config[CONF_ROLE]))
    cg.add(var.set_uart_pins(
        config[CONF_TX_PIN],
        config[CONF_RX_PIN],
        config[CONF_RTS_PIN],
        config[CONF_CTS_PIN],
    ))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_zigbee_channel(config[CONF_ZIGBEE_CHANNEL]))
    cg.add(var.set_max_children(config[CONF_MAX_CHILDREN]))
    cg.add(var.set_bench_isolation(config[CONF_BENCH_ISOLATION]))
    cg.add(var.set_zigbee_antenna(config[CONF_ZIGBEE_ANTENNA]))
    cg.add(var.set_tunnel_port(config[CONF_TUNNEL_PORT]))
    cg.add(var.set_use_mdns(config[CONF_USE_MDNS]))
    cg.add(var.set_master_host(config[CONF_MASTER_HOST]))
    cg.add(var.set_master_static_ip(config[CONF_MASTER_STATIC_IP]))
    cg.add(var.set_tunnel_psk(config[CONF_TUNNEL_PSK]))
