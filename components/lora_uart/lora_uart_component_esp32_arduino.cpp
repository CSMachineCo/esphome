#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "lora_uart_component_esp32_arduino.h"

#ifdef USE_LOGGER
#include "esphome/components/logger/logger.h"
#endif

namespace esphome {
namespace uart {
//namespace lora_uart{
static const char *const TAG = "lora_uart.arduino_esp32";

static const uint32_t UART_PARITY_EVEN = 0 << 0;
static const uint32_t UART_PARITY_ODD = 1 << 0;
static const uint32_t UART_PARITY_ENABLE = 1 << 1;
static const uint32_t UART_NB_BIT_5 = 0 << 2;
static const uint32_t UART_NB_BIT_6 = 1 << 2;
static const uint32_t UART_NB_BIT_7 = 2 << 2;
static const uint32_t UART_NB_BIT_8 = 3 << 2;
static const uint32_t UART_NB_STOP_BIT_1 = 1 << 4;
static const uint32_t UART_NB_STOP_BIT_2 = 3 << 4;
static const uint32_t UART_TICK_APB_CLOCK = 1 << 27;

ESP32ArduinoLoraUARTComponent::ESP32ArduinoLoraUARTComponent(){lora_uart::global_lora_component = this;}

//Don;t think we need this function...
uint32_t ESP32ArduinoLoraUARTComponent::get_config() {
  uint32_t config = 0;

  /*
   * All bits numbers below come from
   * framework-arduinoespressif32/cores/esp32/esp32-hal-uart.h
   * And more specifically conf0 union in uart_dev_t.
   *
   * Below is bit used from conf0 union.
   * <name>:<bits position>  <values>
   * parity:0                0:even 1:odd
   * parity_en:1             Set this bit to enable uart parity check.
   * bit_num:2-4             0:5bits 1:6bits 2:7bits 3:8bits
   * stop_bit_num:4-6        stop bit. 1:1bit  2:1.5bits  3:2bits
   * tick_ref_always_on:27   select the clock.1：apb clock：ref_tick
   */

  if (this->parity_ == UART_CONFIG_PARITY_EVEN) {
    config |= UART_PARITY_EVEN | UART_PARITY_ENABLE;
  } else if (this->parity_ == UART_CONFIG_PARITY_ODD) {
    config |= UART_PARITY_ODD | UART_PARITY_ENABLE;
  }

  switch (this->data_bits_) {
    case 5:
      config |= UART_NB_BIT_5;
      break;
    case 6:
      config |= UART_NB_BIT_6;
      break;
    case 7:
      config |= UART_NB_BIT_7;
      break;
    case 8:
      config |= UART_NB_BIT_8;
      break;
  }

  if (this->stop_bits_ == 1) {
    config |= UART_NB_STOP_BIT_1;
  } else {
    config |= UART_NB_STOP_BIT_2;
  }

  config |= UART_TICK_APB_CLOCK;

  return config;
}

void ESP32ArduinoLoraUARTComponent::setup() {
  
  lora_uart::global_lora_component = this;

//////////This is all old uart stuff...///////////////
/*
  bool is_default_tx, is_default_rx;
#ifdef CONFIG_IDF_TARGET_ESP32C3
  is_default_tx = tx_pin_ == nullptr || tx_pin_->get_pin() == 21;
  is_default_rx = rx_pin_ == nullptr || rx_pin_->get_pin() == 20;
#else
  is_default_tx = tx_pin_ == nullptr || tx_pin_->get_pin() == 1;
  is_default_rx = rx_pin_ == nullptr || rx_pin_->get_pin() == 3;
#endif
  static uint8_t next_uart_num = 0;
  if (is_default_tx && is_default_rx && next_uart_num == 0) {
#if ARDUINO_USB_CDC_ON_BOOT
    this->hw_serial_ = &Serial0;
#else
    this->hw_serial_ = &Serial;
#endif
    next_uart_num++;
  } else {
#ifdef USE_LOGGER
    // The logger doesn't use this UART component, instead it targets the UARTs
    // directly (i.e. Serial/Serial0, Serial1, and Serial2). If the logger is
    // enabled, skip the UART that it is configured to use.
    if (logger::global_logger->get_baud_rate() > 0 && logger::global_logger->get_uart() == next_uart_num) {
      next_uart_num++;
    }
#endif  // USE_LOGGER

    if (next_uart_num >= UART_NUM_MAX) {
      ESP_LOGW(TAG, "Maximum number of UART components created already.");
      this->mark_failed();
      return;
    }

    this->number_ = next_uart_num;
    this->hw_serial_ = new HardwareSerial(next_uart_num++);  // NOLINT(cppcoreguidelines-owning-memory)
  }
*/
  this->load_settings(false);
}

void ESP32ArduinoLoraUARTComponent::load_settings(bool dump_config) {
  buff_write_ptr_ = 0;
  buff_read_ptr_ = 0;

  int8_t MOSI = this->tx_pin_ != nullptr ? this->tx_pin_->get_pin() : -1;
  int8_t MISO = this->rx_pin_ != nullptr ? this->rx_pin_->get_pin() : -1;
  int8_t SCLK = this->_pin_SCK != nullptr ? this->_pin_SCK->get_pin() : -1;
  int8_t NSS = this->_pin_NSS != nullptr ? this->_pin_NSS->get_pin() : -1;
  int8_t RESET = this->_pin_RESET != nullptr ? this->_pin_RESET->get_pin() : -1;
  int8_t DIO1 = this->_pin_DIO1 != nullptr ? this->_pin_DIO1->get_pin() : -1;

  bool invert = false;
  if (tx_pin_ != nullptr && tx_pin_->is_inverted())
    invert = true;
  if (rx_pin_ != nullptr && rx_pin_->is_inverted())
    invert = true;
  //this->hw_serial_->setRxBufferSize(this->rx_buffer_size_);
  //this->hw_serial_->begin(this->baud_rate_, get_config(), rx, tx, invert);

  ESP_LOGD(TAG, "Setting up LoRa UART...");
  radio_init = 1;
  // Since we're pretty much emulating an esphome uart component with an LoRa radio on SPI
  // Turns out somebody wrote a really simple driver for that...
  if (!radio.begin(MISO, MOSI, NSS, SCLK, RESET, DIO1)) { //Initialize radio
    ESP_LOGW(TAG, "Failed to Initialize LoRa Radio");
    radio_init = -1;}
  else
  {
    radio_init = 2;
  }
  
  //TODO: Make optional config
  //FREQUENCY - Set frequency to 915Mhz (default 915Mhz)
  radio.configSetFrequency(915000000);  //Freq in Hz. Must comply with your local radio regulations. Probably should make this optional configuration

  //TODO: Make optional config
  //Configuration presets. Comment/uncomment to observe how long each packet takes to transmit. 
  //radio.configSetPreset(PRESET_DEFAULT);      //Default   - Medium range, medium speed
  //radio.configSetPreset(PRESET_FAST);       //Fast      - Faster, but less reliable at longer distances.  Use when you need fast speed and radios are closer.
  radio.configSetPreset(PRESET_LONGRANGE);  //LongRange - Slower and more reliable.  Good for long distance or when reliability is more important than speed

  //TODO: make tx rx pins configurable  

  if (dump_config) {
    ESP_LOGCONFIG(TAG, "UART %u was reloaded.", this->number_);
    this->dump_config();
  }
}

void ESP32ArduinoLoraUARTComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LoRa UART Bus %d:", this->number_);
  LOG_PIN("  TX Pin: ", tx_pin_);
  LOG_PIN("  RX Pin: ", rx_pin_);
  if (this->rx_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", this->rx_buffer_size_);
  }
  ESP_LOGCONFIG(TAG, "  Baud Rate: %u baud", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "  Data Bits: %u", this->data_bits_);
  ESP_LOGCONFIG(TAG, "  Parity: %s", LOG_STR_ARG(parity_to_str(this->parity_)));
  ESP_LOGCONFIG(TAG, "  Stop bits: %u", this->stop_bits_);
  ESP_LOGCONFIG(TAG, "  Init Success: %u", this->radio_init);
  
  this->check_logger_conflict();
}

void ESP32ArduinoLoraUARTComponent::write_array(const uint8_t *data, size_t len) {
  //this->hw_serial_->write(data, len);
  uint8_t status = this->radio.transmit((byte*)data, len);
  char d_out[100];
  //Parse out the status (see datasheet for what each bit means)
  uint8_t chipMode = (status >> 4) & 0x7;     //Chip mode is bits [6:4] (3-bits)
  uint8_t commandStatus = (status >> 1) & 0x7;//Command status is bits [3:1] (3-bits)
  sprintf(d_out, "LoRa Radio Status Code: %X Chipmode: %X Command Status: %X", status, chipMode, commandStatus);
  ESP_LOGD(TAG, d_out);
#ifdef USE_UART_DEBUGGER
  for (size_t i = 0; i < len; i++) {
    this->debug_callback_.call(UART_DIRECTION_TX, data[i]);
  }
#endif
}

bool ESP32ArduinoLoraUARTComponent::peek_byte(uint8_t *data) {
  if (!this->check_read_timeout_())
    return false;
  *data = this->read_buffer_[this->buff_read_ptr_];
  return true;
}

void ESP32ArduinoLoraUARTComponent::read_radio()
{
  //try to read from radio
  int bytes_read = this->radio.lora_receive_async(this->temp_buffer_, 256);

  
  //if we got new data from radio need to write to read buffer
  if(bytes_read > 0)
  {
    char d_out[100];
    sprintf(d_out, "LoRa Radio Rcv Async bytes: %d", bytes_read);
    ESP_LOGD(TAG, d_out);
  
    for(int i = 0; i < bytes_read; i++)
    {
      if(this->buff_write_ptr_ > 511) //buff wrapped
        this->buff_write_ptr_ = 0;
      this->read_buffer_[this->buff_write_ptr_] = this->temp_buffer_[i];
      this->buff_write_ptr_++;
    }
  }
}

// @param data Pointer to the array where the read data will be stored.
  // @param len Number of bytes to read.
  // @return True if the specified number of bytes were successfully read, false otherwise.
bool ESP32ArduinoLoraUARTComponent::read_array(uint8_t *data, size_t len) {
  //check radio for data
  this->read_radio();

  //get data available in read buffer
  int buffer_data_avail = this->available();
  
  //determine if we have enough data to fill request
  if(buffer_data_avail >= len)
  {
    for(int i = 0; i < len; i++)
    {
      if(this->buff_read_ptr_ > 511)  //buffer wrapped
        this->buff_read_ptr_ = 0;
      data[i] = this->read_buffer_[this->buff_read_ptr_];
      this->buff_read_ptr_++;
    }
  }
  else
  {
    return false;
  }



#ifdef USE_UART_DEBUGGER
  for (size_t i = 0; i < len; i++) {
    this->debug_callback_.call(UART_DIRECTION_RX, data[i]);
  }
#endif
  return true;
}

int ESP32ArduinoLoraUARTComponent::available() 
{ 
  //check radio for data
  this->read_radio();

  int buffer_data_avail = 0;
  //determine how much buffer data we have
  if(this->buff_write_ptr_ < this->buff_read_ptr_)  //buffer wrapped
    buffer_data_avail = (512 - this->buff_read_ptr_) + this->buff_write_ptr_;
  else
    buffer_data_avail = this->buff_write_ptr_ - this->buff_read_ptr_;

  return buffer_data_avail;
  
}
void ESP32ArduinoLoraUARTComponent::flush() {
  ESP_LOGVV(TAG, "    Flushing...");
  //this->hw_serial_->flush();
}

void ESP32ArduinoLoraUARTComponent::check_logger_conflict() {
  //Since this is not a real uart there cannot be a logger conflict.
}

//}  // namespace lora_uart
}  // namespace uart
}  // namespace esphome
#endif  // USE_ESP32_FRAMEWORK_ARDUINO
