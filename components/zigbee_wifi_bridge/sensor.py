import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_MILLISECOND,
)

from . import CONF_ZIGBEE_WIFI_BRIDGE_ID, ZigbeeWifiBridgeComponent

CONF_TUNNEL_RTT = "tunnel_rtt"
CONF_ZIGBEE_CHANNEL = "zigbee_channel"
CONF_ZIGBEE_PAN_ID = "zigbee_pan_id"
CONF_C6_SHORT_ADDRESS = "c6_short_address"
CONF_RF_TX_COUNT = "rf_tx_count"
CONF_RF_RX_COUNT = "rf_rx_count"
CONF_TUNNEL_TX_COUNT = "tunnel_tx_count"
CONF_TUNNEL_RX_COUNT = "tunnel_rx_count"
CONF_RF_ISOLATED_DROPS = "rf_isolated_drops"
CONF_UART_RX_BYTES = "uart_rx_bytes"
CONF_UART_CRC_ERRORS = "uart_crc_errors"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZIGBEE_WIFI_BRIDGE_ID): cv.use_id(
            ZigbeeWifiBridgeComponent
        ),
        cv.Optional(CONF_TUNNEL_RTT): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLISECOND,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:timer-outline",
        ),
        cv.Optional(CONF_ZIGBEE_CHANNEL): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:radio-tower",
        ),
        cv.Optional(CONF_ZIGBEE_PAN_ID): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:zigbee",
        ),
        cv.Optional(CONF_C6_SHORT_ADDRESS): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:identifier",
        ),
        cv.Optional(CONF_RF_TX_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:upload-network",
        ),
        cv.Optional(CONF_RF_RX_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:download-network",
        ),
        cv.Optional(CONF_TUNNEL_TX_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:arrow-up-bold-box-outline",
        ),
        cv.Optional(CONF_TUNNEL_RX_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:arrow-down-bold-box-outline",
        ),
        cv.Optional(CONF_RF_ISOLATED_DROPS): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:shield-lock-outline",
        ),
        cv.Optional(CONF_UART_RX_BYTES): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:serial-port",
        ),
        cv.Optional(CONF_UART_CRC_ERRORS): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:alert-circle-outline",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ZIGBEE_WIFI_BRIDGE_ID])

    sensor_setters = [
        (CONF_TUNNEL_RTT, "set_tunnel_rtt_sensor"),
        (CONF_ZIGBEE_CHANNEL, "set_zigbee_channel_sensor"),
        (CONF_ZIGBEE_PAN_ID, "set_zigbee_pan_id_sensor"),
        (CONF_C6_SHORT_ADDRESS, "set_c6_short_address_sensor"),
        (CONF_RF_TX_COUNT, "set_rf_tx_count_sensor"),
        (CONF_RF_RX_COUNT, "set_rf_rx_count_sensor"),
        (CONF_TUNNEL_TX_COUNT, "set_tunnel_tx_count_sensor"),
        (CONF_TUNNEL_RX_COUNT, "set_tunnel_rx_count_sensor"),
        (CONF_RF_ISOLATED_DROPS, "set_rf_isolated_drops_sensor"),
        (CONF_UART_RX_BYTES, "set_uart_rx_bytes_sensor"),
        (CONF_UART_CRC_ERRORS, "set_uart_crc_errors_sensor"),
    ]

    for key, setter in sensor_setters:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))
