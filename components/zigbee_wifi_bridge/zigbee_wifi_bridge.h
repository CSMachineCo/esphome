#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "bridge_protocol.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

namespace esphome {
namespace zigbee_wifi_bridge {

enum BridgeRole : uint8_t {
  ROLE_MASTER = ZB_ROLE_MASTER,
  ROLE_SLAVE = ZB_ROLE_SLAVE,
};

class ZigbeeWifiBridgeComponent;

#ifdef USE_SWITCH
class ZigbeeBridgeBenchIsolationSwitch
    : public switch_::Switch,
      public Parented<ZigbeeWifiBridgeComponent> {
 protected:
  void write_state(bool state) override;
};
#endif

#ifdef USE_NUMBER
class ZigbeeBridgeChannelNumber
    : public number::Number,
      public Parented<ZigbeeWifiBridgeComponent> {
 protected:
  void control(float value) override;
};
#endif

#ifdef USE_BUTTON
class ZigbeeBridgeFactoryResetButton
    : public button::Button,
      public Parented<ZigbeeWifiBridgeComponent> {
 protected:
  void press_action() override;
};
#endif

class ZigbeeWifiBridgeComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_role(BridgeRole role) { this->role_ = role; }
  void set_uart_pins(int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    this->tx_pin_ = tx_pin;
    this->rx_pin_ = rx_pin;
    this->rts_pin_ = rts_pin;
    this->cts_pin_ = cts_pin;
  }
  void set_baud_rate(int baud_rate) { this->baud_rate_ = baud_rate; }
  void set_zigbee_channel(uint8_t ch) {
    this->yaml_zigbee_channel_ = ch;
    this->zigbee_channel_ = ch;
  }
  void set_max_children(uint8_t max_children) { this->max_children_ = max_children; }
  void set_bench_isolation(bool isolation) {
    this->yaml_bench_isolation_ = isolation;
    this->bench_isolation_ = isolation;
  }
  void set_tunnel_port(uint16_t port) { this->tunnel_port_ = port; }
  void set_use_mdns(bool use_mdns) { this->use_mdns_ = use_mdns; }
  void set_master_host(const std::string &host) { this->master_host_ = host; }
  void set_master_static_ip(const std::string &ip) { this->master_static_ip_ = ip; }
  void set_tunnel_psk(const std::string &psk) { this->tunnel_psk_ = psk; }

#ifdef USE_BINARY_SENSOR
  void set_tunnel_connected_binary_sensor(binary_sensor::BinarySensor *s) { this->tunnel_connected_bs_ = s; }
  void set_c6_link_binary_sensor(binary_sensor::BinarySensor *s) { this->c6_link_bs_ = s; }
  void set_c6_joined_binary_sensor(binary_sensor::BinarySensor *s) { this->c6_joined_bs_ = s; }
#endif

#ifdef USE_SENSOR
  void set_tunnel_rtt_sensor(sensor::Sensor *s) { this->tunnel_rtt_sensor_ = s; }
  void set_zigbee_channel_sensor(sensor::Sensor *s) { this->zigbee_channel_sensor_ = s; }
  void set_zigbee_pan_id_sensor(sensor::Sensor *s) { this->zigbee_pan_id_sensor_ = s; }
  void set_c6_short_address_sensor(sensor::Sensor *s) { this->c6_short_address_sensor_ = s; }
  void set_rf_tx_count_sensor(sensor::Sensor *s) { this->rf_tx_count_sensor_ = s; }
  void set_rf_rx_count_sensor(sensor::Sensor *s) { this->rf_rx_count_sensor_ = s; }
  void set_tunnel_tx_count_sensor(sensor::Sensor *s) { this->tunnel_tx_count_sensor_ = s; }
  void set_tunnel_rx_count_sensor(sensor::Sensor *s) { this->tunnel_rx_count_sensor_ = s; }
  void set_rf_isolated_drops_sensor(sensor::Sensor *s) { this->rf_isolated_drops_sensor_ = s; }
  void set_uart_rx_bytes_sensor(sensor::Sensor *s) { this->uart_rx_bytes_sensor_ = s; }
  void set_uart_crc_errors_sensor(sensor::Sensor *s) { this->uart_crc_errors_sensor_ = s; }
#endif

#ifdef USE_TEXT_SENSOR
  void set_bridge_role_text_sensor(text_sensor::TextSensor *s) { this->bridge_role_ts_ = s; }
  void set_peer_ip_text_sensor(text_sensor::TextSensor *s) { this->peer_ip_ts_ = s; }
#endif

#ifdef USE_SWITCH
  void set_bench_isolation_switch(ZigbeeBridgeBenchIsolationSwitch *sw) { this->bench_isolation_switch_ = sw; }
#endif
#ifdef USE_NUMBER
  void set_zigbee_channel_number(ZigbeeBridgeChannelNumber *num) { this->zigbee_channel_number_ = num; }
#endif
#ifdef USE_BUTTON
  void set_factory_reset_button(ZigbeeBridgeFactoryResetButton *btn) { this->factory_reset_button_ = btn; }
#endif

  void set_runtime_bench_isolation(bool enabled);
  void set_runtime_zigbee_channel(uint8_t channel);
  void trigger_c6_factory_reset();

 protected:
  static void uart_rx_task_trampoline(void *arg);
  static void config_sync_task_trampoline(void *arg);
  static void tunnel_task_trampoline(void *arg);
  static void heartbeat_task_trampoline(void *arg);

  void uart_rx_task();
  void config_sync_task();
  void tunnel_task();
  void heartbeat_task();

  void load_or_sync_nvs();
  void save_runtime_nvs();

  void handle_local_c6_frame(const zb_wire_hdr_t *hdr, const uint8_t *decoded, size_t dec_len);
  void send_framed_uart_msg(zb_bridge_msg_type_t type, const void *payload, uint16_t payload_len);
  bool send_raw_uart_wire(const uint8_t *raw_wire_frame, size_t len);
  void push_config_sync(uint8_t flags);
  void push_status_to_c6();

  void compute_hmac_tag(const uint8_t *data, size_t len, uint8_t out_tag[8]) const;
  bool send_tunnel_wire_frame(const uint8_t *raw_wire_frame, size_t len);
  void send_ack_confirm_to_tunnel(uint16_t seq_id, uint8_t dsn);
  void handle_tunnel_socket_session(int sock, const uint8_t peer_ip[4], int listen_sock);
  bool resolve_master_ipv4(uint32_t *out_s_addr);
  bool is_wifi_connected() const;

  void append_c6_log_chunk(const char *data, size_t len);
  void flush_c6_logs();
  void publish_entities(bool force);

  BridgeRole role_{ROLE_MASTER};
  int tx_pin_{9};
  int rx_pin_{10};
  int rts_pin_{-1};
  int cts_pin_{-1};
  int baud_rate_{2000000};
  uint8_t yaml_zigbee_channel_{15};
  uint8_t zigbee_channel_{15};
  uint8_t max_children_{16};
  bool yaml_bench_isolation_{true};
  bool bench_isolation_{true};
  uint16_t tunnel_port_{7842};
  bool use_mdns_{true};
  std::string master_host_{"zbbridge-master"};
  std::string master_static_ip_{"192.168.1.100"};
  std::string tunnel_psk_{"change-this-32byte-bridge-secret"};

  SemaphoreHandle_t uart_tx_mutex_{nullptr};
  SemaphoreHandle_t sock_mutex_{nullptr};
  SemaphoreHandle_t state_mutex_{nullptr};
  SemaphoreHandle_t log_mutex_{nullptr};

  volatile int tunnel_sock_{-1};
  volatile uint16_t rtt_ms_{0};
  uint8_t peer_ip_[4]{0, 0, 0, 0};
  uint32_t tunnel_tx_frames_{0};
  uint32_t tunnel_rx_frames_{0};

  uint16_t local_uart_seq_{1};
  uint16_t config_version_{1};
  uint16_t acked_config_ver_{0};
  bool config_synced_{false};
  bool c6_link_seen_{false};
  uint32_t uart_rx_bytes_{0};
  uint32_t uart_crc_err_{0};
  uint32_t uart_tx_sync_{0};
  zb_msg_phy_stats_t phy_stats_{};

  std::string c6_log_accum_;
  std::deque<std::string> c6_log_queue_;

  uint32_t last_publish_ms_{0};
  bool initial_controls_published_{false};
  bool mdns_service_registered_{false};

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *tunnel_connected_bs_{nullptr};
  binary_sensor::BinarySensor *c6_link_bs_{nullptr};
  binary_sensor::BinarySensor *c6_joined_bs_{nullptr};
#endif

#ifdef USE_SENSOR
  sensor::Sensor *tunnel_rtt_sensor_{nullptr};
  sensor::Sensor *zigbee_channel_sensor_{nullptr};
  sensor::Sensor *zigbee_pan_id_sensor_{nullptr};
  sensor::Sensor *c6_short_address_sensor_{nullptr};
  sensor::Sensor *rf_tx_count_sensor_{nullptr};
  sensor::Sensor *rf_rx_count_sensor_{nullptr};
  sensor::Sensor *tunnel_tx_count_sensor_{nullptr};
  sensor::Sensor *tunnel_rx_count_sensor_{nullptr};
  sensor::Sensor *rf_isolated_drops_sensor_{nullptr};
  sensor::Sensor *uart_rx_bytes_sensor_{nullptr};
  sensor::Sensor *uart_crc_errors_sensor_{nullptr};
#endif

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *bridge_role_ts_{nullptr};
  text_sensor::TextSensor *peer_ip_ts_{nullptr};
#endif

#ifdef USE_SWITCH
  ZigbeeBridgeBenchIsolationSwitch *bench_isolation_switch_{nullptr};
#endif
#ifdef USE_NUMBER
  ZigbeeBridgeChannelNumber *zigbee_channel_number_{nullptr};
#endif
#ifdef USE_BUTTON
  ZigbeeBridgeFactoryResetButton *factory_reset_button_{nullptr};
#endif
};

}  // namespace zigbee_wifi_bridge
}  // namespace esphome
