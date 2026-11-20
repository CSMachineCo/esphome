#include "zigbee_wifi_bridge.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/core/log.h"
#include "mbedtls/md.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace esphome {
namespace zigbee_wifi_bridge {

static const char *const TAG = "zigbee_wifi_bridge";
static const char *const C6_LOG_TAG = "c6";
static constexpr uart_port_t ZB_UART_PORT = UART_NUM_1;
static constexpr size_t UART_RX_BUF_SIZE = 2048;
static constexpr size_t UART_TX_BUF_SIZE = 2048;
static constexpr size_t TUNNEL_HMAC_LEN = 8;
static constexpr size_t MAX_C6_LOG_QUEUE_LINES = 48;
static constexpr size_t MAX_C6_LOG_LINE_LEN = 220;
static const char *const NVS_NAMESPACE = "zb_esphome";

#ifdef USE_SWITCH
void ZigbeeBridgeBenchIsolationSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_runtime_bench_isolation(state);
  }
  this->publish_state(state);
}
#endif

#ifdef USE_NUMBER
void ZigbeeBridgeChannelNumber::control(float value) {
  uint8_t ch = static_cast<uint8_t>(value + 0.5f);
  if (ch < 11) ch = 11;
  if (ch > 26) ch = 26;
  if (this->parent_ != nullptr) {
    this->parent_->set_runtime_zigbee_channel(ch);
  }
  this->publish_state(static_cast<float>(ch));
}
#endif

#ifdef USE_BUTTON
void ZigbeeBridgeFactoryResetButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->trigger_c6_factory_reset();
  }
}
#endif

static void configure_tcp_socket(int sock) {
  int nodelay = 1;
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  int keepalive = 1;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
  int keepidle = 5;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
  int keepintvl = 2;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
  int keepcnt = 3;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
}

static bool recv_exact(int sock, uint8_t *buf, size_t len) {
  size_t got = 0;
  while (got < len) {
    ssize_t r = recv(sock, buf + got, len - got, 0);
    if (r <= 0) return false;
    got += static_cast<size_t>(r);
  }
  return true;
}

static bool connect_with_timeout(int sock, const struct sockaddr_in *dest, int timeout_ms, int *out_err) {
  int flags = fcntl(sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }

  int rc = connect(sock, reinterpret_cast<const struct sockaddr *>(dest), sizeof(*dest));
  if (rc == 0) {
    if (flags >= 0) fcntl(sock, F_SETFL, flags);
    return true;
  }

  if (errno != EINPROGRESS) {
    if (out_err) *out_err = errno;
    if (flags >= 0) fcntl(sock, F_SETFL, flags);
    return false;
  }

  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(sock, &wfds);
  struct timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  rc = select(sock + 1, nullptr, &wfds, nullptr, &tv);
  if (rc <= 0) {
    if (out_err) *out_err = (rc == 0) ? ETIMEDOUT : errno;
    if (flags >= 0) fcntl(sock, F_SETFL, flags);
    return false;
  }

  int so_err = 0;
  socklen_t len = sizeof(so_err);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &len);
  if (flags >= 0) fcntl(sock, F_SETFL, flags);
  if (so_err != 0) {
    if (out_err) *out_err = so_err;
    return false;
  }
  return true;
}

void ZigbeeWifiBridgeComponent::setup() {
  this->uart_tx_mutex_ = xSemaphoreCreateMutex();
  this->sock_mutex_ = xSemaphoreCreateMutex();
  this->state_mutex_ = xSemaphoreCreateMutex();
  this->log_mutex_ = xSemaphoreCreateMutex();

  if (!this->uart_tx_mutex_ || !this->sock_mutex_ || !this->state_mutex_ || !this->log_mutex_) {
    ESP_LOGE(TAG, "Failed to create bridge mutexes");
    this->mark_failed();
    return;
  }

  this->load_or_sync_nvs();

  uart_config_t cfg{};
  cfg.baud_rate = this->baud_rate_;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = (this->rts_pin_ >= 0 && this->cts_pin_ >= 0)
                      ? UART_HW_FLOWCTRL_CTS_RTS
                      : UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  esp_err_t err = uart_driver_install(ZB_UART_PORT, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, nullptr, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  err = uart_param_config(ZB_UART_PORT, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  err = uart_set_pin(ZB_UART_PORT, this->tx_pin_, this->rx_pin_, this->rts_pin_, this->cts_pin_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  gpio_set_pull_mode(static_cast<gpio_num_t>(this->rx_pin_), GPIO_PULLUP_ONLY);

  xTaskCreate(uart_rx_task_trampoline, "zb_uart_rx", 4096, this, 12, nullptr);
  xTaskCreate(config_sync_task_trampoline, "zb_cfg_sync", 3072, this, 6, nullptr);
  xTaskCreate(tunnel_task_trampoline, "zb_tunnel", 6144, this, 10, nullptr);
  xTaskCreate(heartbeat_task_trampoline, "zb_hb", 3072, this, 4, nullptr);

  ESP_LOGI(TAG, "Zigbee WiFi Bridge initialized (role=%s, UART TX=%d RX=%d @ %d bps, ch=%u, port=%u)",
           this->role_ == ROLE_MASTER ? "MASTER" : "SLAVE",
           this->tx_pin_, this->rx_pin_, this->baud_rate_,
           this->zigbee_channel_, this->tunnel_port_);
}

void ZigbeeWifiBridgeComponent::loop() {
  this->flush_c6_logs();

  uint32_t now = millis();
  if (!this->initial_controls_published_ || (now - this->last_publish_ms_ >= 3000U)) {
    this->publish_entities(!this->initial_controls_published_);
    this->initial_controls_published_ = true;
    this->last_publish_ms_ = now;
  }
}

void ZigbeeWifiBridgeComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Zigbee WiFi Bridge:");
  ESP_LOGCONFIG(TAG, "  Role: %s", this->role_ == ROLE_MASTER ? "MASTER" : "SLAVE");
  ESP_LOGCONFIG(TAG, "  UART1 Pins: TX=%d, RX=%d, RTS=%d, CTS=%d @ %d bps",
                this->tx_pin_, this->rx_pin_, this->rts_pin_, this->cts_pin_, this->baud_rate_);
  ESP_LOGCONFIG(TAG, "  Zigbee Channel: %u, Max Children: %u, Bench Isolation: %s",
                this->zigbee_channel_, this->max_children_,
                this->bench_isolation_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Tunnel Port: %u, Discovery: %s (%s)",
                this->tunnel_port_,
                this->use_mdns_ ? "mDNS" : "Static IP",
                this->use_mdns_ ? this->master_host_.c_str() : this->master_static_ip_.c_str());
}

void ZigbeeWifiBridgeComponent::load_or_sync_nvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  nvs_handle_t h = 0;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
    return;
  }

  uint8_t saved_yaml_ch = 0;
  uint8_t saved_yaml_iso = 0;
  uint8_t saved_ch = this->yaml_zigbee_channel_;
  uint8_t saved_iso = this->yaml_bench_isolation_ ? 1 : 0;
  uint16_t saved_ver = 1;

  bool has_yaml_sig =
      (nvs_get_u8(h, "yaml_ch", &saved_yaml_ch) == ESP_OK) &&
      (nvs_get_u8(h, "yaml_iso", &saved_yaml_iso) == ESP_OK);

  if (has_yaml_sig &&
      saved_yaml_ch == this->yaml_zigbee_channel_ &&
      saved_yaml_iso == (this->yaml_bench_isolation_ ? 1 : 0)) {
    if (nvs_get_u8(h, "channel", &saved_ch) == ESP_OK && saved_ch >= 11 && saved_ch <= 26) {
      this->zigbee_channel_ = saved_ch;
    }
    if (nvs_get_u8(h, "iso", &saved_iso) == ESP_OK) {
      this->bench_isolation_ = (saved_iso != 0);
    }
    if (nvs_get_u16(h, "cfg_ver", &saved_ver) == ESP_OK && saved_ver > 0) {
      this->config_version_ = saved_ver;
    }
  } else {
    nvs_set_u8(h, "yaml_ch", this->yaml_zigbee_channel_);
    nvs_set_u8(h, "yaml_iso", this->yaml_bench_isolation_ ? 1 : 0);
    nvs_set_u8(h, "channel", this->zigbee_channel_);
    nvs_set_u8(h, "iso", this->bench_isolation_ ? 1 : 0);
    if (nvs_get_u16(h, "cfg_ver", &saved_ver) == ESP_OK && saved_ver > 0) {
      this->config_version_ = saved_ver + 1;
    }
    nvs_set_u16(h, "cfg_ver", this->config_version_);
    nvs_commit(h);
  }
  nvs_close(h);
}

void ZigbeeWifiBridgeComponent::save_runtime_nvs() {
  nvs_handle_t h = 0;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
    return;
  }
  nvs_set_u8(h, "yaml_ch", this->yaml_zigbee_channel_);
  nvs_set_u8(h, "yaml_iso", this->yaml_bench_isolation_ ? 1 : 0);
  nvs_set_u8(h, "channel", this->zigbee_channel_);
  nvs_set_u8(h, "iso", this->bench_isolation_ ? 1 : 0);
  nvs_set_u16(h, "cfg_ver", this->config_version_);
  nvs_commit(h);
  nvs_close(h);
}

void ZigbeeWifiBridgeComponent::set_runtime_bench_isolation(bool enabled) {
  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  this->bench_isolation_ = enabled;
  this->config_version_++;
  this->config_synced_ = false;
  this->save_runtime_nvs();
  xSemaphoreGive(this->state_mutex_);

  ESP_LOGI(TAG, "Runtime bench isolation -> %s (ver=%u)", enabled ? "ON" : "OFF", this->config_version_);
  this->push_config_sync(0);
}

void ZigbeeWifiBridgeComponent::set_runtime_zigbee_channel(uint8_t channel) {
  if (channel < 11) channel = 11;
  if (channel > 26) channel = 26;

  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  this->zigbee_channel_ = channel;
  this->config_version_++;
  this->config_synced_ = false;
  this->save_runtime_nvs();
  xSemaphoreGive(this->state_mutex_);

  ESP_LOGI(TAG, "Runtime Zigbee channel -> %u (ver=%u)", channel, this->config_version_);
  this->push_config_sync(0);
}

void ZigbeeWifiBridgeComponent::trigger_c6_factory_reset() {
  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  this->config_version_++;
  this->config_synced_ = false;
  this->save_runtime_nvs();
  xSemaphoreGive(this->state_mutex_);

  ESP_LOGW(TAG, "Sending ZB_CFG_FLAG_FACTORY_RESET to ESP32-C6 (ver=%u)", this->config_version_);
  this->push_config_sync(ZB_CFG_FLAG_FACTORY_RESET);
}

bool ZigbeeWifiBridgeComponent::is_wifi_connected() const {
  return wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_connected();
}

bool ZigbeeWifiBridgeComponent::send_raw_uart_wire(const uint8_t *raw_wire_frame, size_t len) {
  if (!this->uart_tx_mutex_ || !raw_wire_frame || len > ZB_BRIDGE_MAX_WIRE_FRAME) return false;

  uint8_t cobs[ZB_BRIDGE_COBS_BUF_SIZE];
  size_t enc_len = zb_cobs_encode(raw_wire_frame, len, cobs);

  xSemaphoreTake(this->uart_tx_mutex_, portMAX_DELAY);
  int written = uart_write_bytes(ZB_UART_PORT, reinterpret_cast<const char *>(cobs), enc_len);
  xSemaphoreGive(this->uart_tx_mutex_);
  return written == static_cast<int>(enc_len);
}

void ZigbeeWifiBridgeComponent::send_framed_uart_msg(zb_bridge_msg_type_t type,
                                                     const void *payload,
                                                     uint16_t payload_len) {
  if (payload_len > ZB_BRIDGE_MAX_PAYLOAD) return;

  uint8_t raw[ZB_BRIDGE_MAX_WIRE_FRAME];
  auto *hdr = reinterpret_cast<zb_wire_hdr_t *>(raw);
  hdr->magic = ZB_BRIDGE_MAGIC;
  hdr->msg_type = static_cast<uint8_t>(type);
  hdr->seq_id = this->local_uart_seq_++;
  hdr->payload_len = payload_len;
  if (payload_len > 0 && payload != nullptr) {
    memcpy(raw + sizeof(zb_wire_hdr_t), payload, payload_len);
  }

  size_t body_len = sizeof(zb_wire_hdr_t) + payload_len;
  uint16_t crc = zb_bridge_crc16(raw, body_len);
  raw[body_len] = static_cast<uint8_t>(crc & 0xFFU);
  raw[body_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

  this->send_raw_uart_wire(raw, body_len + 2);
}

void ZigbeeWifiBridgeComponent::push_config_sync(uint8_t flags) {
  zb_msg_config_sync_t sync{};
  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  sync.role = static_cast<uint8_t>(this->role_);
  sync.zigbee_channel = this->zigbee_channel_;
  sync.max_children = this->max_children_;
  sync.bench_isolation = this->bench_isolation_ ? 1 : 0;
  sync.flags = flags;
  sync.config_version = this->config_version_;
  xSemaphoreGive(this->state_mutex_);

  this->send_framed_uart_msg(ZB_MSG_CONFIG_SYNC, &sync, sizeof(sync));
}

void ZigbeeWifiBridgeComponent::push_status_to_c6() {
  bool wifi_up = this->is_wifi_connected();
  wifi_ap_record_t ap_info{};
  int8_t rssi = -127;
  if (wifi_up && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    rssi = ap_info.rssi;
  }

  zb_msg_tunnel_status_t st{};
  st.wifi_connected = wifi_up ? 1 : 0;
  st.tunnel_connected = (this->tunnel_sock_ >= 0) ? 1 : 0;
  st.wifi_rssi = rssi;
  st.rtt_ms = this->rtt_ms_;

  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  memcpy(st.peer_ip, this->peer_ip_, 4);
  xSemaphoreGive(this->state_mutex_);

  this->send_framed_uart_msg(ZB_MSG_TUNNEL_STATUS, &st, sizeof(st));
}

void ZigbeeWifiBridgeComponent::append_c6_log_chunk(const char *data, size_t len) {
  if (!this->log_mutex_ || !data || len == 0) return;

  xSemaphoreTake(this->log_mutex_, portMAX_DELAY);
  for (size_t i = 0; i < len; i++) {
    char c = data[i];
    if (c == '\r') continue;
    if (c == '\n') {
      if (!this->c6_log_accum_.empty()) {
        if (this->c6_log_queue_.size() >= MAX_C6_LOG_QUEUE_LINES) {
          this->c6_log_queue_.pop_front();
        }
        this->c6_log_queue_.push_back(std::move(this->c6_log_accum_));
        this->c6_log_accum_.clear();
      }
    } else {
      if (static_cast<unsigned char>(c) >= 0x20 || c == '\t') {
        this->c6_log_accum_.push_back(c);
        if (this->c6_log_accum_.size() >= MAX_C6_LOG_LINE_LEN) {
          if (this->c6_log_queue_.size() >= MAX_C6_LOG_QUEUE_LINES) {
            this->c6_log_queue_.pop_front();
          }
          this->c6_log_queue_.push_back(std::move(this->c6_log_accum_));
          this->c6_log_accum_.clear();
        }
      }
    }
  }
  xSemaphoreGive(this->log_mutex_);
}

void ZigbeeWifiBridgeComponent::flush_c6_logs() {
  if (!this->log_mutex_) return;

  std::vector<std::string> lines;
  xSemaphoreTake(this->log_mutex_, portMAX_DELAY);
  while (!this->c6_log_queue_.empty()) {
    lines.push_back(std::move(this->c6_log_queue_.front()));
    this->c6_log_queue_.pop_front();
  }
  xSemaphoreGive(this->log_mutex_);

  for (const auto &line : lines) {
    ESP_LOGI(C6_LOG_TAG, "%s", line.c_str());
  }
}

void ZigbeeWifiBridgeComponent::handle_local_c6_frame(const zb_wire_hdr_t *hdr,
                                                      const uint8_t *decoded,
                                                      size_t dec_len) {
  const uint8_t *payload = decoded + sizeof(zb_wire_hdr_t);

  switch (static_cast<zb_bridge_msg_type_t>(hdr->msg_type)) {
    case ZB_MSG_CONFIG_ACK:
      if (hdr->payload_len >= sizeof(zb_msg_config_ack_t)) {
        const auto *ack = reinterpret_cast<const zb_msg_config_ack_t *>(payload);
        xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
        this->c6_link_seen_ = true;
        this->acked_config_ver_ = ack->config_version;
        this->config_synced_ =
            (ack->config_version == this->config_version_) &&
            (ack->role == static_cast<uint8_t>(this->role_)) &&
            (ack->zigbee_channel == this->zigbee_channel_);
        xSemaphoreGive(this->state_mutex_);

        ESP_LOGI(TAG, "C6 config ACK: role=%s ch=%u ver=%u status=%u",
                 ack->role == ZB_ROLE_MASTER ? "MASTER" : "SLAVE",
                 ack->zigbee_channel, ack->config_version, ack->applied_status);
      }
      break;

    case ZB_MSG_PHY_STATS:
      if (hdr->payload_len >= sizeof(zb_msg_phy_stats_t)) {
        xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
        memcpy(&this->phy_stats_, payload, sizeof(zb_msg_phy_stats_t));
        this->c6_link_seen_ = true;
        xSemaphoreGive(this->state_mutex_);
      }
      break;

    case ZB_MSG_C6_LOG:
      if (hdr->payload_len > 0) {
        xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
        this->c6_link_seen_ = true;
        xSemaphoreGive(this->state_mutex_);
        this->append_c6_log_chunk(reinterpret_cast<const char *>(payload), hdr->payload_len);
      }
      break;

    case ZB_MSG_IEEE802154_FRAME:
    case ZB_MSG_TX_ACK_CONFIRM:
    case ZB_MSG_PEER_IDENTITY:
    default:
      this->send_tunnel_wire_frame(decoded, dec_len);
      break;
  }
}

void ZigbeeWifiBridgeComponent::uart_rx_task_trampoline(void *arg) {
  static_cast<ZigbeeWifiBridgeComponent *>(arg)->uart_rx_task();
}

void ZigbeeWifiBridgeComponent::config_sync_task_trampoline(void *arg) {
  static_cast<ZigbeeWifiBridgeComponent *>(arg)->config_sync_task();
}

void ZigbeeWifiBridgeComponent::tunnel_task_trampoline(void *arg) {
  static_cast<ZigbeeWifiBridgeComponent *>(arg)->tunnel_task();
}

void ZigbeeWifiBridgeComponent::heartbeat_task_trampoline(void *arg) {
  static_cast<ZigbeeWifiBridgeComponent *>(arg)->heartbeat_task();
}

void ZigbeeWifiBridgeComponent::uart_rx_task() {
  uint8_t chunk[256];
  uint8_t cobs_buf[ZB_BRIDGE_COBS_BUF_SIZE];
  uint8_t decoded[ZB_BRIDGE_MAX_WIRE_FRAME];
  size_t cobs_len = 0;

  while (true) {
    int n = uart_read_bytes(ZB_UART_PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(10));
    if (n <= 0) continue;

    xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
    this->uart_rx_bytes_ += static_cast<uint32_t>(n);
    xSemaphoreGive(this->state_mutex_);

    for (int i = 0; i < n; i++) {
      uint8_t b = chunk[i];
      if (b == 0x00) {
        if (cobs_len > 0) {
          size_t dec_len = zb_cobs_decode(cobs_buf, cobs_len, decoded, sizeof(decoded));
          cobs_len = 0;
          if (dec_len >= sizeof(zb_wire_hdr_t) + 2) {
            const auto *hdr = reinterpret_cast<const zb_wire_hdr_t *>(decoded);
            size_t expected = sizeof(zb_wire_hdr_t) + hdr->payload_len + 2;
            if (hdr->magic == ZB_BRIDGE_MAGIC &&
                hdr->payload_len <= ZB_BRIDGE_MAX_PAYLOAD &&
                dec_len == expected) {
              uint16_t rx_crc = static_cast<uint16_t>(decoded[dec_len - 2]) |
                                (static_cast<uint16_t>(decoded[dec_len - 1]) << 8);
              uint16_t calc_crc = zb_bridge_crc16(decoded, dec_len - 2);
              if (rx_crc == calc_crc) {
                this->handle_local_c6_frame(hdr, decoded, dec_len);
              } else {
                xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
                this->uart_crc_err_++;
                uint32_t err_cnt = this->uart_crc_err_;
                xSemaphoreGive(this->state_mutex_);
                if (err_cnt <= 5U) {
                  ESP_LOGW(TAG, "UART CRC error #%lu from C6", static_cast<unsigned long>(err_cnt));
                }
              }
            } else {
              xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
              this->uart_crc_err_++;
              xSemaphoreGive(this->state_mutex_);
            }
          } else {
            xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
            this->uart_crc_err_++;
            xSemaphoreGive(this->state_mutex_);
          }
        }
      } else {
        if (cobs_len < sizeof(cobs_buf)) {
          cobs_buf[cobs_len++] = b;
        } else {
          cobs_len = 0;
        }
      }
    }
  }
}

void ZigbeeWifiBridgeComponent::config_sync_task() {
  vTaskDelay(pdMS_TO_TICKS(250));
  uint32_t wait_loops = 0;

  while (true) {
    xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
    bool synced = this->config_synced_;
    bool link_seen = this->c6_link_seen_;
    bool stats_mismatch = link_seen &&
                          (this->phy_stats_.role != static_cast<uint8_t>(this->role_) ||
                           this->phy_stats_.channel != this->zigbee_channel_ ||
                           this->phy_stats_.bench_isolation != (this->bench_isolation_ ? 1 : 0));
    uint32_t rx_bytes = this->uart_rx_bytes_;
    uint32_t crc_err = this->uart_crc_err_;
    xSemaphoreGive(this->state_mutex_);

    bool need_sync = !synced || stats_mismatch;
    if (need_sync) {
      this->push_config_sync(0);
      xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
      this->uart_tx_sync_++;
      uint32_t tx_sync = this->uart_tx_sync_;
      xSemaphoreGive(this->state_mutex_);

      if (!link_seen) {
        wait_loops++;
        if ((wait_loops % 4U) == 1U) {
          ESP_LOGW(TAG,
                   "No UART reply from C6 yet (sync_tx=%lu, rx_bytes=%lu, err=%lu) on TX=GPIO%d RX=GPIO%d @ %d bps",
                   static_cast<unsigned long>(tx_sync),
                   static_cast<unsigned long>(rx_bytes),
                   static_cast<unsigned long>(crc_err),
                   this->tx_pin_, this->rx_pin_, this->baud_rate_);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(need_sync ? 1500 : 5000));
  }
}

void ZigbeeWifiBridgeComponent::compute_hmac_tag(const uint8_t *data, size_t len, uint8_t out_tag[8]) const {
  uint8_t full_hmac[32];
  const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(md_info,
                  reinterpret_cast<const unsigned char *>(this->tunnel_psk_.data()),
                  this->tunnel_psk_.size(),
                  data, len,
                  full_hmac);
  memcpy(out_tag, full_hmac, TUNNEL_HMAC_LEN);
}

bool ZigbeeWifiBridgeComponent::send_tunnel_wire_frame(const uint8_t *raw_wire_frame, size_t len) {
  if (!raw_wire_frame || len == 0 || len > ZB_BRIDGE_MAX_WIRE_FRAME) return false;
  if (this->tunnel_sock_ < 0) return false;

  uint8_t pkt[2 + ZB_BRIDGE_MAX_WIRE_FRAME + TUNNEL_HMAC_LEN];
  pkt[0] = static_cast<uint8_t>(len & 0xFFU);
  pkt[1] = static_cast<uint8_t>((len >> 8) & 0xFFU);
  memcpy(pkt + 2, raw_wire_frame, len);
  this->compute_hmac_tag(raw_wire_frame, len, pkt + 2 + len);

  size_t total = 2 + len + TUNNEL_HMAC_LEN;
  xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
  int sock = this->tunnel_sock_;
  bool ok = false;
  if (sock >= 0) {
    ssize_t sent = send(sock, pkt, total, 0);
    if (sent == static_cast<ssize_t>(total)) {
      this->tunnel_tx_frames_++;
      ok = true;
    }
  }
  xSemaphoreGive(this->sock_mutex_);
  return ok;
}

void ZigbeeWifiBridgeComponent::send_ack_confirm_to_tunnel(uint16_t seq_id, uint8_t dsn) {
  uint8_t raw[sizeof(zb_wire_hdr_t) + sizeof(zb_msg_ack_confirm_t) + 2];
  auto *hdr = reinterpret_cast<zb_wire_hdr_t *>(raw);
  hdr->magic = ZB_BRIDGE_MAGIC;
  hdr->msg_type = ZB_MSG_TX_ACK_CONFIRM;
  hdr->seq_id = seq_id;
  hdr->payload_len = sizeof(zb_msg_ack_confirm_t);

  auto *ack = reinterpret_cast<zb_msg_ack_confirm_t *>(raw + sizeof(zb_wire_hdr_t));
  ack->acked_seq_id = seq_id;
  ack->status = 0;
  ack->dsn = dsn;

  size_t body_len = sizeof(zb_wire_hdr_t) + sizeof(zb_msg_ack_confirm_t);
  uint16_t crc = zb_bridge_crc16(raw, body_len);
  raw[body_len] = static_cast<uint8_t>(crc & 0xFFU);
  raw[body_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

  this->send_tunnel_wire_frame(raw, body_len + 2);
}

void ZigbeeWifiBridgeComponent::handle_tunnel_socket_session(int sock,
                                                             const uint8_t peer_ip[4],
                                                             int listen_sock) {
  configure_tcp_socket(sock);

  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  memcpy(this->peer_ip_, peer_ip, 4);
  xSemaphoreGive(this->state_mutex_);

  this->tunnel_sock_ = sock;
  ESP_LOGI(TAG, "Tunnel connected (%s) with peer %u.%u.%u.%u",
           this->role_ == ROLE_MASTER ? "MASTER" : "SLAVE",
           peer_ip[0], peer_ip[1], peer_ip[2], peer_ip[3]);
  this->push_status_to_c6();

  uint8_t frame_buf[ZB_BRIDGE_MAX_WIRE_FRAME];
  uint8_t rx_tag[TUNNEL_HMAC_LEN];
  uint8_t calc_tag[TUNNEL_HMAC_LEN];

  while (true) {
    if (listen_sock >= 0) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(sock, &rfds);
      FD_SET(listen_sock, &rfds);
      int max_fd = (listen_sock > sock) ? listen_sock : sock;
      if (select(max_fd + 1, &rfds, nullptr, nullptr, nullptr) <= 0) {
        break;
      }
      if (FD_ISSET(listen_sock, &rfds)) {
        ESP_LOGW(TAG, "Incoming peer reconnect on port %u -> replacing previous tunnel session",
                 this->tunnel_port_);
        break;
      }
      if (!FD_ISSET(sock, &rfds)) {
        continue;
      }
    }

    uint8_t len_bytes[2];
    if (!recv_exact(sock, len_bytes, 2)) break;

    uint16_t frame_len = static_cast<uint16_t>(len_bytes[0]) |
                         (static_cast<uint16_t>(len_bytes[1]) << 8);
    if (frame_len < sizeof(zb_wire_hdr_t) + 2 || frame_len > ZB_BRIDGE_MAX_WIRE_FRAME) break;
    if (!recv_exact(sock, frame_buf, frame_len)) break;
    if (!recv_exact(sock, rx_tag, TUNNEL_HMAC_LEN)) break;

    this->compute_hmac_tag(frame_buf, frame_len, calc_tag);
    if (memcmp(rx_tag, calc_tag, TUNNEL_HMAC_LEN) != 0) {
      ESP_LOGW(TAG, "HMAC mismatch, dropping packet");
      continue;
    }

    const auto *hdr = reinterpret_cast<const zb_wire_hdr_t *>(frame_buf);
    if (hdr->msg_type == ZB_MSG_HEARTBEAT && hdr->payload_len >= sizeof(zb_msg_heartbeat_t)) {
      const auto *hb = reinterpret_cast<const zb_msg_heartbeat_t *>(frame_buf + sizeof(zb_wire_hdr_t));
      if (this->role_ == ROLE_MASTER) {
        this->send_tunnel_wire_frame(frame_buf, frame_len);
      } else {
        uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        if (now_ms >= hb->timestamp_ms) {
          this->rtt_ms_ = static_cast<uint16_t>(now_ms - hb->timestamp_ms);
        }
      }
      continue;
    }

    xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
    this->tunnel_rx_frames_++;
    xSemaphoreGive(this->state_mutex_);

    this->send_raw_uart_wire(frame_buf, frame_len);

    if (hdr->msg_type == ZB_MSG_IEEE802154_FRAME && hdr->payload_len >= 8) {
      const auto *msg = reinterpret_cast<const zb_msg_frame_t *>(frame_buf + sizeof(zb_wire_hdr_t));
      if (msg->flags & ZB_FRAME_FLAG_ACK_REQ) {
        this->send_ack_confirm_to_tunnel(hdr->seq_id, msg->psdu[3]);
      }
    }
  }

  xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
  this->tunnel_sock_ = -1;
  close(sock);
  xSemaphoreGive(this->sock_mutex_);

  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  memset(this->peer_ip_, 0, sizeof(this->peer_ip_));
  xSemaphoreGive(this->state_mutex_);

  ESP_LOGW(TAG, "Tunnel session closed");
  this->push_status_to_c6();
}

bool ZigbeeWifiBridgeComponent::resolve_master_ipv4(uint32_t *out_s_addr) {
  if (this->use_mdns_) {
    esp_ip4_addr_t addr{};
    if (mdns_query_a(this->master_host_.c_str(), 2000, &addr) == ESP_OK && addr.addr != 0) {
      *out_s_addr = addr.addr;
      return true;
    }
    return false;
  }
  struct in_addr addr{};
  if (inet_aton(this->master_static_ip_.c_str(), &addr) != 0) {
    *out_s_addr = addr.s_addr;
    return true;
  }
  return false;
}

void ZigbeeWifiBridgeComponent::tunnel_task() {
  while (!this->is_wifi_connected()) {
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  // Ensure WiFi modem sleep is disabled so DTIM intervals do not cause 200-400 ms RTT spikes.
  esp_wifi_set_ps(WIFI_PS_NONE);

  // ESPHome's mdns component initializes mdns_init(); safely register our _zbbridge._tcp service on MASTER.
  if (this->role_ == ROLE_MASTER && !this->mdns_service_registered_) {
    esp_err_t init_err = mdns_init();
    if (init_err == ESP_OK || init_err == ESP_ERR_INVALID_STATE) {
      esp_err_t svc_err = mdns_service_add("ZigbeeBridgeMaster", "_zbbridge", "_tcp",
                                           this->tunnel_port_, nullptr, 0);
      if (svc_err == ESP_OK || svc_err == ESP_ERR_INVALID_ARG) {
        this->mdns_service_registered_ = true;
        ESP_LOGI(TAG, "mDNS service _zbbridge._tcp active on port %u", this->tunnel_port_);
      }
    }
  }

  if (this->role_ == ROLE_MASTER) {
    while (true) {
      while (!this->is_wifi_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
      if (listen_sock < 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      int reuse = 1;
      setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

      struct sockaddr_in bind_addr{};
      bind_addr.sin_family = AF_INET;
      bind_addr.sin_port = htons(this->tunnel_port_);
      bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

      if (bind(listen_sock, reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr)) == 0 &&
          listen(listen_sock, 2) == 0) {
        ESP_LOGI(TAG, "MASTER TCP tunnel listening on port %u", this->tunnel_port_);
        while (true) {
          struct sockaddr_in peer_addr{};
          socklen_t addr_len = sizeof(peer_addr);
          int client = accept(listen_sock, reinterpret_cast<struct sockaddr *>(&peer_addr), &addr_len);
          if (client >= 0) {
            uint8_t ip_bytes[4];
            memcpy(ip_bytes, &peer_addr.sin_addr.s_addr, 4);
            this->handle_tunnel_socket_session(client, ip_bytes, listen_sock);
          } else {
            vTaskDelay(pdMS_TO_TICKS(500));
          }
        }
      } else {
        ESP_LOGE(TAG, "MASTER failed to bind/listen on port %u (errno=%d)", this->tunnel_port_, errno);
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
  } else {
    while (true) {
      if (!this->is_wifi_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }

      uint32_t master_s_addr = 0;
      if (!this->resolve_master_ipv4(&master_s_addr)) {
        if (this->use_mdns_) {
          ESP_LOGI(TAG, "SLAVE waiting to resolve MASTER (%s.local)...", this->master_host_.c_str());
        } else {
          ESP_LOGW(TAG, "SLAVE invalid static MASTER IP '%s'", this->master_static_ip_.c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }

      struct in_addr addr{};
      addr.s_addr = master_s_addr;
      char ip_str[16];
      inet_ntop(AF_INET, &addr, ip_str, sizeof(ip_str));
      ESP_LOGI(TAG, "SLAVE connecting to MASTER %s:%u (%s)...",
               ip_str, this->tunnel_port_, this->use_mdns_ ? "mDNS" : "static IP");

      int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
      if (sock < 0) {
        ESP_LOGW(TAG, "SLAVE socket() failed (errno=%d)", errno);
        vTaskDelay(pdMS_TO_TICKS(1500));
        continue;
      }

      struct sockaddr_in dest{};
      dest.sin_family = AF_INET;
      dest.sin_port = htons(this->tunnel_port_);
      dest.sin_addr = addr;

      int conn_err = 0;
      if (connect_with_timeout(sock, &dest, 5000, &conn_err)) {
        uint8_t ip_bytes[4];
        memcpy(ip_bytes, &dest.sin_addr.s_addr, 4);
        this->handle_tunnel_socket_session(sock, ip_bytes, -1);
      } else {
        ESP_LOGW(TAG, "SLAVE connect to %s:%u failed (errno=%d: %s), retrying...",
                 ip_str, this->tunnel_port_, conn_err, strerror(conn_err));
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
  }
}

void ZigbeeWifiBridgeComponent::heartbeat_task() {
  uint16_t hb_seq = 1;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    this->push_status_to_c6();

    if (this->role_ == ROLE_SLAVE && this->tunnel_sock_ >= 0) {
      uint8_t raw[sizeof(zb_wire_hdr_t) + sizeof(zb_msg_heartbeat_t) + 2];
      auto *hdr = reinterpret_cast<zb_wire_hdr_t *>(raw);
      hdr->magic = ZB_BRIDGE_MAGIC;
      hdr->msg_type = ZB_MSG_HEARTBEAT;
      hdr->seq_id = hb_seq++;
      hdr->payload_len = sizeof(zb_msg_heartbeat_t);

      auto *hb = reinterpret_cast<zb_msg_heartbeat_t *>(raw + sizeof(zb_wire_hdr_t));
      hb->timestamp_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
      xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
      hb->rx_frames = this->tunnel_rx_frames_;
      hb->tx_frames = this->tunnel_tx_frames_;
      xSemaphoreGive(this->state_mutex_);

      size_t body_len = sizeof(zb_wire_hdr_t) + sizeof(zb_msg_heartbeat_t);
      uint16_t crc = zb_bridge_crc16(raw, body_len);
      raw[body_len] = static_cast<uint8_t>(crc & 0xFFU);
      raw[body_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

      this->send_tunnel_wire_frame(raw, body_len + 2);
    }
  }
}

void ZigbeeWifiBridgeComponent::publish_entities(bool force) {
  zb_msg_phy_stats_t stats{};
  uint8_t peer_ip[4]{0, 0, 0, 0};
  uint8_t channel = 15;
  bool isolation = true;
  bool c6_link = false;
  uint32_t rx_bytes = 0;
  uint32_t crc_err = 0;
  uint32_t tun_tx = 0;
  uint32_t tun_rx = 0;

  xSemaphoreTake(this->state_mutex_, portMAX_DELAY);
  stats = this->phy_stats_;
  memcpy(peer_ip, this->peer_ip_, 4);
  channel = this->zigbee_channel_;
  isolation = this->bench_isolation_;
  c6_link = this->c6_link_seen_;
  rx_bytes = this->uart_rx_bytes_;
  crc_err = this->uart_crc_err_;
  tun_tx = this->tunnel_tx_frames_;
  tun_rx = this->tunnel_rx_frames_;
  xSemaphoreGive(this->state_mutex_);

  bool tunnel_up = (this->tunnel_sock_ >= 0);
  uint16_t rtt = this->rtt_ms_;

#ifdef USE_BINARY_SENSOR
  if (this->tunnel_connected_bs_ != nullptr &&
      (force || !this->tunnel_connected_bs_->has_state() || this->tunnel_connected_bs_->state != tunnel_up)) {
    this->tunnel_connected_bs_->publish_state(tunnel_up);
  }
  if (this->c6_link_bs_ != nullptr &&
      (force || !this->c6_link_bs_->has_state() || this->c6_link_bs_->state != c6_link)) {
    this->c6_link_bs_->publish_state(c6_link);
  }
  bool joined = (stats.joined != 0);
  if (this->c6_joined_bs_ != nullptr &&
      (force || !this->c6_joined_bs_->has_state() || this->c6_joined_bs_->state != joined)) {
    this->c6_joined_bs_->publish_state(joined);
  }
#endif

#ifdef USE_SENSOR
  auto publish_if_changed = [force](sensor::Sensor *s, float val) {
    if (s != nullptr && (force || !s->has_state() || s->raw_state != val)) {
      s->publish_state(val);
    }
  };
  publish_if_changed(this->tunnel_rtt_sensor_, static_cast<float>(rtt));
  publish_if_changed(this->zigbee_channel_sensor_, static_cast<float>(stats.channel ? stats.channel : channel));
  publish_if_changed(this->zigbee_pan_id_sensor_, static_cast<float>(stats.pan_id));
  publish_if_changed(this->c6_short_address_sensor_, static_cast<float>(stats.short_addr));
  publish_if_changed(this->rf_tx_count_sensor_, static_cast<float>(stats.rf_tx_count));
  publish_if_changed(this->rf_rx_count_sensor_, static_cast<float>(stats.rf_rx_count));
  publish_if_changed(this->tunnel_tx_count_sensor_, static_cast<float>(tun_tx));
  publish_if_changed(this->tunnel_rx_count_sensor_, static_cast<float>(tun_rx));
  publish_if_changed(this->rf_isolated_drops_sensor_, static_cast<float>(stats.rf_isolated_drops));
  publish_if_changed(this->uart_rx_bytes_sensor_, static_cast<float>(rx_bytes));
  publish_if_changed(this->uart_crc_errors_sensor_, static_cast<float>(crc_err));
#endif

#ifdef USE_TEXT_SENSOR
  if (this->bridge_role_ts_ != nullptr) {
    const char *role_str = (this->role_ == ROLE_MASTER) ? "MASTER" : "SLAVE";
    if (force || !this->bridge_role_ts_->has_state() || this->bridge_role_ts_->state != role_str) {
      this->bridge_role_ts_->publish_state(role_str);
    }
  }
  if (this->peer_ip_ts_ != nullptr) {
    char ip_buf[20];
    snprintf(ip_buf, sizeof(ip_buf), "%u.%u.%u.%u",
             peer_ip[0], peer_ip[1], peer_ip[2], peer_ip[3]);
    if (force || !this->peer_ip_ts_->has_state() || this->peer_ip_ts_->state != ip_buf) {
      this->peer_ip_ts_->publish_state(ip_buf);
    }
  }
#endif

#ifdef USE_SWITCH
  if (force && this->bench_isolation_switch_ != nullptr) {
    this->bench_isolation_switch_->publish_state(isolation);
  }
#endif

#ifdef USE_NUMBER
  if (force && this->zigbee_channel_number_ != nullptr) {
    this->zigbee_channel_number_->publish_state(static_cast<float>(channel));
  }
#endif
}

}  // namespace zigbee_wifi_bridge
}  // namespace esphome
