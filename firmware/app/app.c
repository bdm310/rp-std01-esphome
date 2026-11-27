#include "app.h"
#include "board.h"
#include "hal.h"
#include "link.h"
#include "protocol.h"
#include "safety.h"

XDATA gate_t app_gate;
XDATA status_in_t app_status;
static XDATA safety_t safety;
static XDATA link_rx_t rx;
static XDATA hal_in_t in;
static XDATA window_t w;
static XDATA uint8_t rx_buf[APP_MAX_PAYLOAD];
static XDATA uint8_t frame[STATUS_LEN + FRAME_OVERHEAD];
_Static_assert(STATUS_LEN + INFO_LEN + 2 * FRAME_OVERHEAD <= HAL_TX_RING - 1,
               "a status and an info frame must fit the transmit ring");
static XDATA uint16_t last_tick, last_rx_ms, last_status_ms, last_info_ms;
static XDATA uint8_t last_edges, wdt_count_cleared, info_wanted, gate_fault_seen, coast_seen;

/* The payload is built in place in frame: link_build copies it onto itself. */
static void send(uint8_t type, uint8_t len) {
  uint8_t n = link_build(frame, type, frame + FRAME_HEADER, len);
  hal_uart_send(frame, n);
}

static void send_info(void) {
  info_build(frame + FRAME_HEADER, hal_fw_version(), hal_build_id(), hal_reset_cause(),
             hal_wdt_count(), hal_boot_version(), app_status.boot_id);
  send(MSG_INFO, INFO_LEN);
}

static void handle_frame(void) {
  switch (rx.type) {
    case MSG_GATE:
      if (rx.len == GATE_LEN)
        gate_set_target(&app_gate, rx_buf[0] | ((uint16_t) rx_buf[1] << 8), rx_buf[2], rx_buf[3]);
      break;
    case MSG_OUTPUTS:
      if (rx.len == OUTPUTS_LEN) {
        /* Kept in app_status, which every status echoes. */
        app_status.leds = rx_buf[0];
        app_status.night = rx_buf[1];
        hal_set_outputs(app_status.leds, app_status.night);
      }
      break;
    case MSG_INFO_REQ:
      if (rx.len == 0)
        info_wanted = 1;
      break;
    case MSG_UPDATE:
      if (rx.len == UPDATE_LEN && rx_buf[0] == UPDATE_KEY0 && rx_buf[1] == UPDATE_KEY1 &&
          rx_buf[2] == UPDATE_KEY2 && rx_buf[3] == UPDATE_KEY3)
        gate_request_update(&app_gate);
      break;
    default:
      break;
  }
}

void app_init(void) {
  app_status.boot_id = hal_boot_id();
  app_status.leds = 0;
  app_status.night = 0;
  gate_init(&app_gate);
  safety_init(&safety);
  link_rx_init(&rx, rx_buf, APP_MAX_PAYLOAD);
  link_rx_types(&rx, MSG_GATE, MSG_UPDATE);
  gate_fault_seen = 0;
  coast_seen = 0;
  last_tick = last_rx_ms = last_status_ms = last_info_ms = hal_ms();
  last_edges = 0;
  wdt_count_cleared = 0;
  info_wanted = 0;
  send_info();
}

void app_loop(void) {
  uint16_t now;
  uint8_t stop_faults, faults, n, b;

  /* Received bytes only until the tick advances, so the tick work below (safety, gate, watchdog)
     is late by at most one byte's work. */
  while (hal_ms() == last_tick && hal_uart_read(&b)) {
    last_rx_ms = last_tick;
    if (link_rx_byte(&rx, b))
      handle_frame();
  }

  now = hal_ms();
  if (now == last_tick)
    return;
  last_tick = now;
  /* The watchdog is cleared only while the 1 ms tick advances. */
  hal_wdt_clear();

  if ((uint16_t) (now - last_rx_ms) > APP_RX_TIMEOUT_MS)
    link_rx_drop_partial(&rx);
  if ((uint16_t) (now - last_info_ms) >= INFO_PERIOD_MS) {
    last_info_ms = now;
    info_wanted = 1;
  }

  hal_take_inputs(&in);
  n = in.edges - last_edges;
  if (n) {
    last_edges = in.edges;
    if (n > 4)
      n = 4;
    while (n--)
      gate_half_cycle(&app_gate);
  }
  if (in.gate_fault)
    gate_fault_seen = 1;
  if (in.zo_coasted)
    coast_seen = 1;
  app_status.starting = in.therm_ready ? 0 : 1;
  stop_faults = safety_therm(&safety, in.therm_avg, in.therm_ready) | (in.gate_fault ? F_GATE : 0);
  if (!app_status.starting && app_gate.startup)
    gate_startup_done(&app_gate, stop_faults);
  gate_tick(&app_gate, now, in.edges, in.zo_valid ? in.half : 0, in.zo_valid, stop_faults);
  hal_set_fire(app_gate.fire);
  hal_set_relay(app_gate.relay);

  if (gate_update_ready(&app_gate)) {
    hal_enter_bootloader();
    return;
  }

  if (!wdt_count_cleared && now >= WDT_CLEAR_COUNT_MS) {
    hal_clear_wdt_count();
    wdt_count_cleared = 1;
  }

  if ((uint16_t) (now - last_status_ms) >= STATUS_PERIOD_MS) {
    last_status_ms = now;
    hal_take_window(&w);
    faults = stop_faults | (in.zo_valid ? 0 : F_ZO) | (gate_fault_seen ? F_GATE : 0);
    if (app_gate.clamped_target != app_gate.target)
      faults |= F_CLAMPED;
    app_status.state = app_gate.state;
    app_status.faults = faults;
    app_status.target = app_gate.clamped_target;
    app_status.current = app_gate.state == ST_FIRING ? app_gate.current : 0;
    app_status.half = in.zo_valid ? in.half : 0;
    app_status.relay = app_gate.relay;
    app_status.therm = in.therm_avg;
    app_status.wheel = in.wheel_avg;
    app_status.coasted = coast_seen;
    status_build(frame + FRAME_HEADER, &app_status, &w);
    send(MSG_STATUS, STATUS_LEN);
    gate_fault_seen = 0;
    coast_seen = 0;
    /* Besides the one at start-up, info frames go out only here, after a status, so the transmit
       ring never holds more than these two frames, which fit, and hal_uart_send() never drops a
       status. */
    if (info_wanted) {
      info_wanted = 0;
      send_info();
    }
  }
}
