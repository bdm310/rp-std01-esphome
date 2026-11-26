#include "mcu_sim.h"

#include <string.h>

#include "board.h"
#include "flashmap.h"
#include "hal.h"
#include "link.h"
#include "protocol.h"
#include "update.h"
#include "window.h"

mcu_sim_t mcu_sim;
uint8_t mcu_sim_flash[APP_END];
const uint32_t MCU_SIM_BYTE_US = (LINK_BYTE_BITS * 1000000ul + LINK_BAUD / 2u) / LINK_BAUD;

/* An erase or a write stalls the CPU. */
#define ERASE_US 25000u
#define WRITE_BYTE_US 62u

enum { ADC_KS, ADC_WHEEL, ADC_THERM, ADC_NONE };
static const uint16_t KS_LEVEL[2][3] = {{556, 680, 840}, {62, 630, 820}};

static struct {
  uint8_t ring[HAL_RX_RING];
  uint8_t head, tail;
  uint8_t tf0;
  uint64_t wire_free_us;
  uint32_t cpu_us; /* the bootloader's stall in the call now running */
  uint8_t edges, good;
  uint64_t next_edge_us, next_tick_us;
  window_t window;
  avg8_t therm, wheel;
  uint8_t adc_slot, adc_ch;
  link_rx_t split;
  uint8_t split_buf[BOOT_MAX_PAYLOAD];
  struct {
    uint64_t at;
    uint8_t b;
  } in[8192];
  size_t in_head, in_n;
} s;

uint8_t flash_erase(uint16_t addr) {
  if (!in_app_area(addr, 1))
    return 0;
  addr &= ~(SECTOR_SIZE - 1);
  memset(mcu_sim_flash + addr, 0xFF, SECTOR_SIZE);
  s.cpu_us += ERASE_US;
  if (mcu_sim.flash_changed)
    mcu_sim.flash_changed(addr, SECTOR_SIZE);
  return 1;
}

uint8_t flash_write(uint16_t addr, const uint8_t *data, uint8_t n) {
  uint8_t i;
  if (!in_app_area(addr, n))
    return 0;
  for (i = 0; i < n; i++)
    mcu_sim_flash[addr + i] &= data[i];
  s.cpu_us += WRITE_BYTE_US * n;
  if (mcu_sim.flash_changed)
    mcu_sim.flash_changed(addr, n);
  return 1;
}

uint8_t flash_read(uint16_t addr) { return mcu_sim_flash[addr]; }

void wdt_kick(void) {}

static uint16_t flash16(uint16_t a) { return mcu_sim_flash[a] | (uint16_t) mcu_sim_flash[a + 1] << 8; }
uint16_t mcu_sim_app_version(void) { return flash16(TRAILER_ADDR + TR_VERSION); }
uint32_t mcu_sim_app_build(void) {
  return flash16(TRAILER_ADDR + TR_BUILD) | (uint32_t) flash16(TRAILER_ADDR + TR_BUILD + 2) << 16;
}

static uint32_t queued_bytes(void) {
  uint64_t now = mcu_sim.now_us;
  return s.wire_free_us > now ? (uint32_t) ((s.wire_free_us - now + MCU_SIM_BYTE_US - 1) / MCU_SIM_BYTE_US) : 0;
}

static uint8_t send(const uint8_t *buf, uint8_t n, int app) {
  uint8_t frame[BOOT_MAX_PAYLOAD + FRAME_OVERHEAD];
  uint64_t at = mcu_sim.now_us + s.cpu_us;
  memcpy(frame, buf, n);
  if (app && queued_bytes() + n > HAL_TX_RING - 1)
    return 0;
  if (at < s.wire_free_us)
    at = s.wire_free_us;
  s.wire_free_us = at + (uint64_t) n * MCU_SIM_BYTE_US;
  if (mcu_sim.tx)
    mcu_sim.tx(frame, n, at);
  return 1;
}

/* Auxiliary RAM that did not keep its contents: the hand-over bytes and the boot ID. */
static void lose_aux_ram(void) {
  static uint8_t garbage = 0x3D;
  uint8_t old = mcu_sim.boot_id;
  memset((uint8_t *) handover, 0xEE, HO_SIZE);
  if (mcu_sim.lost_ram_fixed) {
    mcu_sim.boot_id = 0xEE;
    return;
  }
  /* The application's new boot ID is garbage + 1; on the chip it would repeat once in 256. */
  do
    garbage = (uint8_t) (garbage * 5u + 0x47u);
  while ((uint8_t) (garbage + 1u) == old);
  mcu_sim.boot_id = garbage;
}

static void stop_device(void) {
  memset(&app_gate, 0, sizeof app_gate);
  memset(&app_status, 0, sizeof app_status);
  mcu_sim.relay_pin = mcu_sim.leds_pin = mcu_sim.night_pin = 0;
  mcu_sim.fire = 0;
  s.head = s.tail = 0;
  s.tf0 = 0;
  s.cpu_us = 0;
  mcu_sim.busy_until_us = 0;
  link_rx_init(&s.split, s.split_buf, BOOT_MAX_PAYLOAD);
}

static void enter_update_mode(uint8_t reason) {
  mcu_sim.mode = SIM_BOOT;
  mcu_sim.boot_entries++;
  boot_hal_init();
  boot_init(reason);
}

static void start(int power_on, int watchdog) {
  uint8_t reason;
  stop_device();
  mcu_sim.start_us = mcu_sim.now_us;
  s.next_tick_us = mcu_sim.now_us + 1000;
  reason = boot_reason((uint8_t) power_on, (uint8_t) watchdog);
  if (reason) {
    enter_update_mode(reason);
    return;
  }
  mcu_sim.mode = SIM_APP;
  mcu_sim.app_starts++;
  hal_init();
  app_init();
}

void mcu_sim_power_on(void) {
  lose_aux_ram();
  start(1, 0);
}

void mcu_sim_reset(int watchdog) {
  if (!mcu_sim.aux_ram_survives_reset)
    lose_aux_ram();
  start(0, watchdog);
}

void mcu_sim_power_cut(uint32_t ms) {
  stop_device();
  mcu_sim.mode = SIM_OFF;
  mcu_sim.off_until_us = mcu_sim.now_us + (uint64_t) ms * 1000u;
}

void mcu_sim_update_mode(uint8_t reason) {
  stop_device();
  mcu_sim.start_us = mcu_sim.now_us;
  s.next_tick_us = mcu_sim.now_us + 1000;
  enter_update_mode(reason);
}

void mcu_sim_init(void) {
  memset(&mcu_sim, 0, sizeof mcu_sim);
  memset(&s, 0, sizeof s);
  mcu_sim.zo_on = 1;
  mcu_sim.half_counts = PCA_HZ / 120u;
  mcu_sim.therm_raw = 933;
  mcu_sim.wheel_raw = 806;
  mcu_sim.ks_level = KS_LEVEL[0][0];
  mcu_sim.buttons = 7;
  mcu_sim.aux_ram_survives_reset = 1;
  mcu_sim.boot_version = BOOT_VERSION;
  boot_idle_return_ms = IDLE_RETURN_MS;
  stop_device();
}

void hal_init(void) {
  mcu_sim.boot_id++;
  s.edges = s.good = 0;
  if (s.next_edge_us < mcu_sim.now_us)
    s.next_edge_us = mcu_sim.now_us;
  memset(&s.window, 0, sizeof s.window);
  window_reset(&s.window);
  avg8_init(&s.therm);
  avg8_init(&s.wheel);
  s.adc_slot = 0;
  s.adc_ch = ADC_NONE;
}

uint16_t hal_ms(void) { return (uint16_t) ((mcu_sim.now_us - mcu_sim.start_us) / 1000u); }
void hal_wdt_clear(void) {}

void hal_take_inputs(hal_in_t *in) {
  in->edges = s.edges;
  in->half = mcu_sim.half_counts;
  in->zo_valid = mcu_sim.zo_on && s.good >= ZO_GOOD_EDGES;
  in->gate_fault = 0;
  in->therm_avg = s.therm.value;
  in->therm_ready = s.therm.ready;
  in->wheel_avg = s.wheel.value;
}

void hal_take_window(window_t *w) {
  *w = s.window;
  window_reset(&s.window);
}

void hal_set_fire(uint16_t delay) { mcu_sim.fire = delay; }
void hal_set_relay(uint8_t on) { mcu_sim.relay_pin = on ? 1 : 0; }

void hal_set_outputs(uint8_t leds, uint8_t night) {
  mcu_sim.leds_pin = leds;
  mcu_sim.night_pin = night;
}

uint8_t hal_uart_read(uint8_t *b) {
  if (s.head == s.tail)
    return 0;
  *b = s.ring[s.tail];
  s.tail = (s.tail + 1) & (HAL_RX_RING - 1);
  return 1;
}

uint8_t hal_uart_send(const uint8_t *buf, uint8_t len) { return send(buf, len, 1); }

void hal_enter_bootloader(void) {
  handover[0] = HO_MAGIC0;
  handover[1] = HO_MAGIC1;
  handover[HO_REQUEST] = HO_REQUEST_UPDATE;
  mcu_sim_reset(0);
}

uint8_t hal_reset_cause(void) { return handover[HO_RESET_CAUSE]; }
uint8_t hal_wdt_count(void) { return handover[HO_WDT_COUNT]; }
void hal_clear_wdt_count(void) { handover[HO_WDT_COUNT] = 0; }
uint8_t hal_boot_id(void) { return mcu_sim.boot_id; }
/* As the installed image says, so that a test can install another release. */
uint16_t hal_fw_version(void) { return mcu_sim_app_version(); }
uint32_t hal_build_id(void) { return mcu_sim_app_build(); }
uint8_t hal_boot_version(void) { return mcu_sim.boot_version; }

/* The 1 ms tick interrupt: ADC slots and buttons. */
static void tick_isr(void) {
  uint8_t next;
  if (s.adc_ch == ADC_KS) {
    if (!mcu_sim.ks_manual)
      mcu_sim.ks_level = KS_LEVEL[mcu_sim.relay_pin][mcu_sim.addon];
    window_add_ks(&s.window, mcu_sim.ks_level, mcu_sim.relay_pin);
  } else if (s.adc_ch == ADC_THERM) {
    avg8_add(&s.therm, mcu_sim.therm_raw);
  } else if (s.adc_ch == ADC_WHEEL) {
    avg8_add(&s.wheel, mcu_sim.wheel_raw);
  }
  if (!(s.adc_slot & 1))
    next = ADC_KS;
  else if (s.adc_slot == 1)
    next = ADC_WHEEL;
  else if (s.adc_slot == 5)
    next = ADC_THERM;
  else
    next = ADC_NONE;
  if (++s.adc_slot == 10)
    s.adc_slot = 0;
  s.adc_ch = next;
  window_add_buttons(&s.window, mcu_sim.buttons);
}

void boot_hal_iap_idle(void) {}

void boot_hal_init(void) {
  s.head = s.tail = 0;
  s.tf0 = 0;
}

void boot_hal_take(void) {}

uint8_t boot_hal_read(uint8_t *b) {
  if (s.head == s.tail)
    return 0;
  *b = s.ring[s.tail];
  s.tail = (s.tail + 1) & (BOOT_RX_RING - 1);
  return 1;
}

void boot_hal_send(const uint8_t *frame, uint8_t n) { send(frame, n, 0); }

uint8_t boot_hal_tick(void) {
  if (!s.tf0)
    return 0;
  s.tf0 = 0;
  return 1;
}

void boot_hal_restart(void) { mcu_sim_reset(0); }

static void run_bootloader(void) {
  while (mcu_sim.mode == SIM_BOOT && (s.tf0 || s.head != s.tail))
    boot_loop();
  if (s.cpu_us) {
    mcu_sim.busy_until_us = mcu_sim.now_us + s.cpu_us;
    s.cpu_us = 0;
  }
}

static void mains(void) {
  uint64_t half_us = (uint64_t) mcu_sim.half_counts * 1000000u / PCA_HZ;
  if (!mcu_sim.zo_on) {
    s.good = 0;
    s.next_edge_us = mcu_sim.now_us + half_us;
    return;
  }
  while (s.next_edge_us <= mcu_sim.now_us) {
    s.edges++;
    if (s.good < 0xFF)
      s.good++;
    s.next_edge_us += half_us;
  }
}

static void tick(void) {
  s.next_tick_us += 1000;
  mains();
  if (mcu_sim.mode == SIM_APP) {
    tick_isr();
    app_loop();
  } else {
    s.tf0 = 1;
    if (mcu_sim.now_us >= mcu_sim.busy_until_us)
      run_bootloader();
  }
}

static void deliver(uint8_t b, uint64_t at) {
  int mode = mcu_sim.mode;
  if (mode == SIM_OFF)
    return;
  if (mode == SIM_BOOT && at < mcu_sim.busy_until_us) {
    mcu_sim.busy_dropped++;
    return;
  }
  if (link_rx_byte(&s.split, b) && mcu_sim.frame_in) {
    mcu_sim.frame_in(s.split.type, s.split_buf, s.split.len);
    if (mcu_sim.mode != mode)
      return;
  }
  if (mode == SIM_APP) {
    uint8_t next = (s.head + 1) & (HAL_RX_RING - 1);
    if (next != s.tail) {
      s.ring[s.head] = b;
      s.head = next;
    }
    app_loop();
  } else {
    uint8_t next = (s.head + 1) & (BOOT_RX_RING - 1);
    if (next != s.tail) {
      s.ring[s.head] = b;
      s.head = next;
    }
    run_bootloader();
  }
}

void mcu_sim_rx(uint8_t b, uint64_t at_us) {
  size_t cap = sizeof s.in / sizeof s.in[0];
  if (s.in_n == cap)
    return;
  s.in[(s.in_head + s.in_n) % cap].at = at_us;
  s.in[(s.in_head + s.in_n) % cap].b = b;
  s.in_n++;
}

void mcu_sim_run(uint64_t to_us) {
  size_t cap = sizeof s.in / sizeof s.in[0];
  for (;;) {
    uint64_t t_in = s.in_n ? s.in[s.in_head].at : UINT64_MAX;
    uint64_t t_tick = mcu_sim.mode != SIM_OFF ? s.next_tick_us : UINT64_MAX;
    uint64_t t_power = mcu_sim.mode == SIM_OFF && mcu_sim.off_until_us ? mcu_sim.off_until_us : UINT64_MAX;
    uint64_t t = t_in < t_tick ? t_in : t_tick;
    if (t_power < t)
      t = t_power;
    if (t > to_us)
      break;
    if (t > mcu_sim.now_us)
      mcu_sim.now_us = t;
    if (t == t_power) {
      mcu_sim.off_until_us = 0;
      mcu_sim_power_on();
    } else if (t == t_tick) {
      tick();
    } else {
      uint8_t b = s.in[s.in_head].b;
      s.in_head = (s.in_head + 1) % cap;
      s.in_n--;
      deliver(b, t);
    }
  }
  if (to_us > mcu_sim.now_us)
    mcu_sim.now_us = to_us;
}
