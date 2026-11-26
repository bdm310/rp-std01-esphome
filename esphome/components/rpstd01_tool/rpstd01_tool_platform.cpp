#include "esphome/components/rpstd01_tool/rpstd01_tool_platform.h"

#include "esphome/components/rpstd01_tool/rpstd01_mcu_full_image.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/log.h"

#ifdef ESP_PLATFORM
#include "esp_ota_ops.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_rtc_time.h"
#include "hal/gpio_ll.h"
#include "hal/uart_ll.h"
#include "hal/wdt_hal.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/reset_reasons.h"
#include "soc/soc_caps.h"
#include "soc/uart_pins.h"
#include "esphome/components/rpstd01_tool/rpstd01_tool_nvs.h"
#ifdef USE_NETWORK
#include "esphome/components/network/util.h"
#endif
#endif

namespace rpstd01tool {

static const char *const TAG = "rpstd01tool";

Carried carried_image() {
  using namespace rpstd01;
  return Carried{MCU_IMAGE,         sizeof(MCU_IMAGE), MCU_IMAGE_CRC32,
                 MCU_IMAGE_VERSION, MCU_IMAGE_BUILD,   MCU_IMAGE_BOOT_VERSION};
}

#ifdef ESP_PLATFORM

namespace {

// The RTC watchdog the second-stage bootloader starts is only turned off after the constructors have
// run (init_disable_rtc_wdt), and programming takes several seconds.
void keep_rtc_wdt_fed() {
  static uint32_t last_us = 0;
  const uint32_t now = (uint32_t) esp_rtc_get_time_us();
  if (last_us && (uint32_t) (now - last_us) < 250000)
    return;
  last_us = now;
  wdt_hal_context_t rtc_wdt = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rtc_wdt);
  wdt_hal_feed(&rtc_wdt);
  wdt_hal_write_protect_enable(&rtc_wdt);
}

// UART0 driven directly: this runs before ESPHome's UART driver exists. UART0 is not
// the console here (the logger uses USB_SERIAL_JTAG), so ESP-IDF has switched its clock off and held
// it in reset (esp_perip_clk_init); begin() undoes that and routes the pins. ESPHome's uart
// component later takes UART0 over and resets it again.
static_assert(RPSTD01TOOL_RX_PIN == U0RXD_GPIO_NUM && RPSTD01TOOL_TX_PIN == U0TXD_GPIO_NUM,
              "the pins the code generation requires of the UART are UART0's IO MUX pins");

class McuUartIo : public PowerOnIo {
 public:
  void begin() override {
    {
      int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));  // single-threaded: before the scheduler
      uart_ll_enable_bus_clock(UART_NUM_0, true);
      uart_ll_reset_register(UART_NUM_0);
    }
    uart_ll_sclk_enable(&UART0);
    uart_ll_set_mode_normal(&UART0);
    uart_ll_set_data_bit_num(&UART0, UART_DATA_8_BITS);
    uart_ll_set_stop_bits(&UART0, UART_STOP_BITS_1);
    uart_ll_set_hw_flow_ctrl(&UART0, UART_HW_FLOWCTRL_DISABLE, 0);
    uart_ll_txfifo_rst(&UART0);
    uart_ll_rxfifo_rst(&UART0);
    // U0TXD and U0RXD through the IO MUX (their pins' reset functions), RX with pull-up.
    gpio_ll_func_sel(&GPIO, U0TXD_GPIO_NUM, FUNC_U0TXD_U0TXD);
    gpio_ll_func_sel(&GPIO, U0RXD_GPIO_NUM, FUNC_U0RXD_U0RXD);
    gpio_ll_input_enable(&GPIO, U0RXD_GPIO_NUM);
    gpio_ll_pullup_en(&GPIO, U0RXD_GPIO_NUM);
    GPIO.func_in_sel_cfg[U0RXD_IN_IDX].sig_in_sel = 0;  // from the IO MUX, not the GPIO matrix
  }
  void set_baud(uint32_t baud) override {
    this->wait_tx_();
    uart_ll_set_sclk(&UART0, SOC_MOD_CLK_XTAL);
    uart_ll_set_baudrate(&UART0, baud, 40000000);
    uart_ll_set_parity(&UART0, UART_PARITY_EVEN);
    uart_ll_rxfifo_rst(&UART0);
  }
  void set_no_parity() override { uart_ll_set_parity(&UART0, UART_PARITY_DISABLE); }
  void write(const uint8_t *data, size_t len) override {
    while (len > 0) {
      uint32_t room = uart_ll_get_txfifo_len(&UART0);
      if (room > len)
        room = len;
      if (room) {
        uart_ll_write_txfifo(&UART0, data, room);
        data += room;
        len -= room;
      }
    }
    this->wait_tx_();
  }
  bool available() override { return uart_ll_get_rxfifo_len(&UART0) > 0; }
  int read(uint32_t timeout_us) override {
    const uint32_t start = this->now_us();
    while (!this->available()) {
      if ((uint32_t) (this->now_us() - start) >= timeout_us)
        return -1;
    }
    uint8_t b;
    uart_ll_read_rxfifo(&UART0, &b, 1);
    return b;
  }
  // Every wait loop asks for the time, so this is where the watchdog is kept fed.
  uint32_t now_us() override {
    keep_rtc_wdt_fed();
    return (uint32_t) esp_rtc_get_time_us();
  }
  void delay_us(uint32_t us) override { esp_rom_delay_us(us); }

 protected:
  // A full TX FIFO at the slowest rate, twice over.
  static constexpr uint32_t TX_DRAIN_US = 2 * SOC_UART_FIFO_LEN * stcisp::HANDSHAKE_CHAR_US;

  void wait_tx_() {
    const uint32_t start = this->now_us();
    while (!uart_ll_is_tx_idle(&UART0) && (uint32_t) (this->now_us() - start) < TX_DRAIN_US) {
    }
  }
};

IspRecord power_on_record;
bool power_on_ran = false;

// Runs before ESPHome (and before the other constructors), as early after the power-on as
// possible, while STC's bootloader still listens.
void __attribute__((constructor(101))) power_on_ctor() {
  if (esp_rom_get_reset_reason(0) != RESET_REASON_CHIP_POWER_ON)
    return;
  if (nvs_flash_init() != ESP_OK)
    return;  // ESPHome repairs NVS later if needed
  NvsToolStore store(true);
  if (!store.is_open())
    return;
  McuUartIo io;
  power_on_ran = power_on(store, io, carried_image(), &power_on_record);
}

bool running_image_pending_verify() {
  esp_ota_img_states_t state;
  return esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
         state == ESP_OTA_IMG_PENDING_VERIFY;
}

}

void confirm_image_if_ready(Tool &tool) {
  if (!tool.image_pending())
    return;
#if defined(USE_OTA) && defined(USE_NETWORK)
  static bool failed = false;
  if (!failed && esphome::network::is_connected() && esphome::App.is_setup_complete() && tool.decided() &&
      esp_ota_get_running_partition() == esp_ota_get_boot_partition()) {
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Marked this image valid, so a power cycle cannot roll it back");
    } else {
      failed = true;
      ESP_LOGW(TAG, "Could not mark this image valid (%s); safe_mode tries after %u s", esp_err_to_name(err),
               (unsigned) SAFE_MODE_GOOD_AFTER_S);
    }
  }
#endif
  tool.set_image_pending(running_image_pending_verify());
}

void setup_tool(Tool &tool) {
  nvs_flash_init();
  static NvsToolStore store(true);
  tool.begin(&store, carried_image(), power_on_ran ? &power_on_record : nullptr);
  tool.set_image_pending(running_image_pending_verify());
  if (power_on_ran) {
    char text[200];
    tool.last_install_text(text, sizeof(text));
    ESP_LOGI(TAG, "%s", text);
  }
}

#elif defined(USE_HOST)

namespace {

class RamToolStore final : public ToolStore {
 public:
  void load(mcuflash::Store *s) override { *s = this->store_; }
  bool set(Key key, uint32_t value) override {
    switch (key) {
      case KEY_ARM:
        this->store_.arm = value;
        break;
      case KEY_USER:
        this->store_.user = (uint8_t) value;
        break;
      case KEY_FAILS:
        this->store_.fails = (uint8_t) value;
        break;
      case KEY_PARTIAL:
        this->store_.partial = (uint8_t) value;
        break;
    }
    return true;
  }
  bool commit() override { return true; }
  bool load_record(IspRecord *) override { return false; }
  bool save_record(const IspRecord &) override { return true; }

 protected:
  mcuflash::Store store_{0, mcuflash::USER_NONE, 0, 0};
};

}

void confirm_image_if_ready(Tool &) {}

void setup_tool(Tool &tool) {
  static RamToolStore store;
  tool.begin(&store, carried_image(), nullptr);
}

#endif

}
