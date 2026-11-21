#ifndef ZB_BRIDGE_PROTOCOL_H
#define ZB_BRIDGE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZB_BRIDGE_MAGIC             0x5A
#define ZB_BRIDGE_PROTO_VERSION     2
#define ZB_BRIDGE_MAX_PSDU          128
#define ZB_BRIDGE_MAX_PAYLOAD       256
#define ZB_BRIDGE_MAX_WIRE_FRAME    (6 + ZB_BRIDGE_MAX_PAYLOAD + 2)
#define ZB_BRIDGE_COBS_BUF_SIZE     (ZB_BRIDGE_MAX_WIRE_FRAME + 8)

/* Frame flags for ZB_MSG_IEEE802154_FRAME */
#define ZB_FRAME_FLAG_BROADCAST     (1U << 0)
#define ZB_FRAME_FLAG_ACK_REQ       (1U << 1)
#define ZB_FRAME_FLAG_BEACON        (1U << 2)
#define ZB_FRAME_FLAG_BEACON_REQ    (1U << 3)

/* Config sync flags */
#define ZB_CFG_FLAG_FACTORY_RESET   (1U << 0)

typedef enum {
    ZB_ANTENNA_INTERNAL = 0x00, /* XIAO ESP32-C6 GPIO14 LOW (default ceramic antenna) */
    ZB_ANTENNA_EXTERNAL = 0x01, /* XIAO ESP32-C6 GPIO14 HIGH (external U.FL antenna) */
} zb_zigbee_antenna_t;

typedef enum {
    ZB_ROLE_UNCONFIGURED = 0x00,
    ZB_ROLE_MASTER       = 0x01, /* Connects to Zigbee Coordinator over RF (TCP Server) */
    ZB_ROLE_SLAVE        = 0x02, /* Remote area router for clients (TCP Client) */
} zb_bridge_role_t;

typedef enum {
    ZB_MSG_IEEE802154_FRAME  = 0x01, /* Encapsulated IEEE 802.15.4 PSDU */
    ZB_MSG_TX_ACK_CONFIRM    = 0x02, /* Tunnel delivery confirmation */
    ZB_MSG_PEER_IDENTITY     = 0x03, /* Sync short_addr + IEEE64 across link */
    ZB_MSG_TUNNEL_STATUS     = 0x04, /* C3 -> C6 WiFi & TCP tunnel status */
    ZB_MSG_HEARTBEAT         = 0x05, /* Keepalive & RTT probe */
    ZB_MSG_CONFIG_SYNC       = 0x06, /* C3 -> C6 runtime role/channel/isolation/antenna config */
    ZB_MSG_CONFIG_ACK        = 0x07, /* C6 -> C3 config applied acknowledgment */
    ZB_MSG_PHY_STATS         = 0x08, /* C6 -> C3 live Zigbee & RF/Tunnel counters */
    ZB_MSG_C6_LOG            = 0x09, /* C6 -> C3 mirrored serial console log chunk */
} zb_bridge_msg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  msg_type;
    uint16_t seq_id;
    uint16_t payload_len;
} zb_wire_hdr_t;

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    int8_t   rssi;
    uint8_t  lqi;
    uint8_t  psdu_len; /* PSDU byte count including length byte frame[0] */
    uint8_t  psdu[ZB_BRIDGE_MAX_PSDU];
} zb_msg_frame_t;

typedef struct __attribute__((packed)) {
    uint16_t acked_seq_id;
    uint8_t  status; /* 0 = OK, 1 = TIMEOUT/FAIL */
    uint8_t  dsn;
} zb_msg_ack_confirm_t;

typedef struct __attribute__((packed)) {
    uint8_t  role;
    uint8_t  joined;
    uint8_t  channel;
    uint8_t  reserved;
    uint16_t short_addr;
    uint16_t pan_id;
    uint8_t  ieee_addr[8];
} zb_msg_peer_identity_t;

typedef struct __attribute__((packed)) {
    uint8_t  wifi_connected;
    uint8_t  tunnel_connected;
    int8_t   wifi_rssi;
    uint8_t  reserved;
    uint16_t rtt_ms;
    uint8_t  peer_ip[4];
} zb_msg_tunnel_status_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint32_t rx_frames;
    uint32_t tx_frames;
} zb_msg_heartbeat_t;

typedef struct __attribute__((packed)) {
    uint8_t  role;            /* zb_bridge_role_t: MASTER or SLAVE */
    uint8_t  zigbee_channel;  /* 11..26 */
    uint8_t  max_children;    /* 4..32 */
    uint8_t  bench_isolation; /* 0 or 1 */
    uint8_t  flags;           /* ZB_CFG_FLAG_FACTORY_RESET */
    uint8_t  zigbee_antenna;  /* zb_zigbee_antenna_t: 0 = internal, 1 = external */
    uint16_t config_version;
} zb_msg_config_sync_t;

typedef struct __attribute__((packed)) {
    uint8_t  role;
    uint8_t  zigbee_channel;
    uint8_t  max_children;
    uint8_t  bench_isolation;
    uint8_t  applied_status;  /* 0 = OK, 1 = Rebooting to apply */
    uint8_t  zigbee_antenna;  /* zb_zigbee_antenna_t: 0 = internal, 1 = external */
    uint16_t config_version;
} zb_msg_config_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t  role;
    uint8_t  joined;
    uint8_t  channel;
    uint8_t  bench_isolation;
    uint16_t short_addr;
    uint16_t pan_id;
    uint32_t rf_tx_count;
    uint32_t rf_rx_count;
    uint32_t tunnel_tx_count;
    uint32_t tunnel_rx_count;
    uint32_t rf_isolated_drops;
    uint8_t  zigbee_antenna;  /* zb_zigbee_antenna_t: 0 = internal, 1 = external */
} zb_msg_phy_stats_t;

static inline uint16_t zb_bridge_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static inline size_t zb_cobs_encode(const uint8_t *input, size_t len, uint8_t *output)
{
    size_t read_idx = 0;
    size_t write_idx = 1;
    size_t code_idx = 0;
    uint8_t code = 1;

    while (read_idx < len) {
        if (input[read_idx] == 0) {
            output[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
            read_idx++;
        } else {
            output[write_idx++] = input[read_idx++];
            code++;
            if (code == 0xFF) {
                output[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
    }
    output[code_idx] = code;
    output[write_idx++] = 0x00; /* Packet delimiter */
    return write_idx;
}

static inline size_t zb_cobs_decode(const uint8_t *input, size_t len, uint8_t *output, size_t max_out)
{
    size_t read_idx = 0;
    size_t write_idx = 0;

    while (read_idx < len) {
        uint8_t code = input[read_idx];
        if (code == 0) {
            return 0;
        }
        read_idx++;
        for (uint8_t i = 1; i < code; i++) {
            if (read_idx >= len || write_idx >= max_out) {
                return 0;
            }
            output[write_idx++] = input[read_idx++];
        }
        if (code < 0xFF && read_idx < len) {
            if (write_idx >= max_out) {
                return 0;
            }
            output[write_idx++] = 0;
        }
    }
    return write_idx;
}

#ifdef __cplusplus
}
#endif

#endif /* ZB_BRIDGE_PROTOCOL_H */
