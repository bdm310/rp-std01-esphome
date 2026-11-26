/*
 * Fake RP-STD01 MCU for the ESPHome host-platform tests of the main configuration and the MCU tool:
 * the simulated MCU (mcu_sim.h) in real time, talking 8N1 raw over a pseudo-terminal it creates,
 * with a file-backed flash, and line commands on a unix socket.
 *
 * --image AREA: a file with an application area (APP_SIZE bytes) to install when FILE is new.
 * --boot-version sets the byte at BOOT_VERSION_ADDR that the application reports in its info frame.
 * The bootloader's own status always reports BOOT_VERSION.
 * --pad N adds N zero bytes to every frame it sends, as a later release with longer frames would.
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "flashmap.h"
#include "link.h"
#include "mcu_sim.h"
#include "protocol.h"

#define MAX_CLIENTS 8
#define LOG_MAX 1024
#define LOG_LINE 96

static const char *const MODE_NAMES[] = {"off", "app", "boot"};

static const char *opt_link, *opt_ctl, *opt_flash, *opt_image;
static int opt_invalid;
static uint8_t opt_pad;

static struct timespec t0;
static uint64_t now_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t) (t.tv_sec - t0.tv_sec) * 1000000u + (uint64_t) ((t.tv_nsec - t0.tv_nsec) / 1000);
}

static int flash_fd = -1;

static void flash_sync(uint16_t addr, uint16_t n) {
  if (pwrite(flash_fd, mcu_sim_flash + addr, n, addr - APP_BASE) != (ssize_t) n)
    perror("flash write");
}

static void flash_open(void) {
  struct stat sb;
  int exists = stat(opt_flash, &sb) == 0 && sb.st_size == APP_SIZE;
  flash_fd = open(opt_flash, O_RDWR | O_CREAT, 0644);
  if (flash_fd < 0) {
    perror(opt_flash);
    exit(1);
  }
  memset(mcu_sim_flash, 0xFF, APP_END);
  if (exists) {
    if (pread(flash_fd, mcu_sim_flash + APP_BASE, APP_SIZE, 0) != APP_SIZE) {
      perror("flash read");
      exit(1);
    }
    return;
  }
  if (opt_image) {
    FILE *f = fopen(opt_image, "rb");
    if (!f || fread(mcu_sim_flash + APP_BASE, 1, APP_SIZE, f) != APP_SIZE) {
      fprintf(stderr, "cannot read %s (need %u bytes)\n", opt_image, (unsigned) APP_SIZE);
      exit(1);
    }
    fclose(f);
    if (!opt_invalid)
      mcu_sim_flash[TRAILER_ADDR + TR_VALID] = 0x00;
  }
  flash_sync(APP_BASE, APP_SIZE);
}

/* The event log, read and cleared by the `log` command. */
static char log_buf[LOG_MAX][LOG_LINE];
static int log_n;

static void ev(const char *fmt, ...) {
  va_list ap;
  int k;
  if (log_n == LOG_MAX)
    return;
  k = snprintf(log_buf[log_n], LOG_LINE, "%llu ", (unsigned long long) (mcu_sim.now_us / 1000));
  va_start(ap, fmt);
  vsnprintf(log_buf[log_n] + k, LOG_LINE - k, fmt, ap);
  va_end(ap);
  log_n++;
}

typedef struct {
  int n;
  int type; /* -1 = any */
} inject_t;
static inject_t drop_rx, corrupt_rx, tamper_rx, drop_tx;
static uint32_t rx_count[256], tx_count[256], rx_dropped;

static int inject_take(inject_t *i, uint8_t type) {
  if (i->n > 0 && (i->type < 0 || i->type == type)) {
    i->n--;
    return 1;
  }
  return 0;
}

/* Bytes to the ESP32, written when they would have finished on the wire. */
static int pty_fd = -1;
static uint8_t out_buf[8192];
static size_t out_n;
static uint64_t out_due_us, hold_until_us;

static void out_write(const uint8_t *b, size_t n) {
  /* Nobody listening (the pty would keep the bytes for the next reader), or the buffer full: the
     bytes are lost, as on a wire. */
  struct pollfd pf = {pty_fd, POLLOUT, 0};
  if (poll(&pf, 1, 0) >= 0 && !(pf.revents & POLLHUP) && write(pty_fd, b, n) != (ssize_t) n)
    perror("pty write");
}

static void out_flush(uint64_t us) {
  if (out_n && us >= out_due_us && us >= hold_until_us) {
    out_write(out_buf, out_n);
    out_n = 0;
  }
}

static void out_queue(const uint8_t *b, size_t n, uint64_t due_us) {
  if (out_n + n > sizeof out_buf)
    return;
  memcpy(out_buf + out_n, b, n);
  out_n += n;
  if (due_us > out_due_us)
    out_due_us = due_us;
}

static void on_tx(const uint8_t *frame, uint8_t n, uint64_t at_us) {
  uint8_t payload[UINT8_MAX] = {0}, padded[UINT8_MAX];
  if (opt_pad) {
    memcpy(payload, frame + 3, frame[2]);
    n = link_build(padded, frame[1], payload, (uint8_t) (frame[2] + opt_pad));
    frame = padded;
  }
  if (inject_take(&drop_tx, frame[1])) {
    ev("dropped tx %02X", frame[1]);
    return;
  }
  tx_count[frame[1]]++;
  out_queue(frame, n, at_us + (uint64_t) n * MCU_SIM_BYTE_US);
}

/* The first bytes of a status frame only, as when a reset cuts one off on the wire. */
static void send_cut_status(void) {
  uint8_t frame[STATUS_LEN + FRAME_OVERHEAD], payload[STATUS_LEN] = {0};
  link_build(frame, MSG_STATUS, payload, STATUS_LEN);
  out_queue(frame, 11, mcu_sim.now_us);
}

static void on_frame_in(uint8_t type, const uint8_t *p, uint8_t len) {
  if (mcu_sim.mode == SIM_BOOT) {
    ev("boot cmd %02X", type);
  } else if (type == MSG_GATE && len == GATE_LEN) {
    uint16_t target = p[0] | (uint16_t) p[1] << 8;
    if (target != app_gate.target)
      ev("gate %u %u %u", target, p[2], p[3]);
  } else if (type == MSG_OUTPUTS && len == OUTPUTS_LEN) {
    if (p[0] != app_status.leds || p[1] != app_status.night)
      ev("outputs %u %u", p[0], p[1]);
  } else if (type == MSG_UPDATE) {
    ev("update requested");
  }
}

static void log_changes(void) {
  static uint32_t app_starts, boot_entries;
  static uint8_t state, relay;
  if (mcu_sim.app_starts != app_starts || mcu_sim.boot_entries != boot_entries) {
    if (mcu_sim.mode == SIM_APP)
      ev("app start cause=%u wdt=%u boot_id=%u", handover[HO_RESET_CAUSE], handover[HO_WDT_COUNT], mcu_sim.boot_id);
    else
      ev("boot start reason=%u", boot_update_reason);
    app_starts = mcu_sim.app_starts;
    boot_entries = mcu_sim.boot_entries;
  }
  if (app_gate.state != state) {
    ev("state %u eff=%u current=%u", app_gate.state, app_gate.clamped_target, app_gate.current);
    state = app_gate.state;
  }
  if (app_gate.relay != relay) {
    ev("relay %u", app_gate.relay);
    relay = app_gate.relay;
  }
}

/* Bytes from the ESP32. A splitter finds whole frames, so faults can be injected per frame. */
static link_rx_t split;
static uint8_t split_buf[BOOT_MAX_PAYLOAD];

static void rx_bytes(const uint8_t *buf, ssize_t n, uint64_t us) {
  uint8_t frame[BOOT_MAX_PAYLOAD + FRAME_OVERHEAD], type, len, i;
  ssize_t k;
  for (k = 0; k < n; k++) {
    if (!link_rx_byte(&split, buf[k]))
      continue;
    type = split.type;
    len = split.len;
    rx_count[type]++;
    link_build(frame, type, split_buf, len);
    if (inject_take(&drop_rx, type)) {
      rx_dropped++;
      ev("dropped rx %02X", type);
      continue;
    }
    if (len && inject_take(&tamper_rx, type)) {
      split_buf[type == MSG_BOOT_WRITE && len > 66 ? 66 : len / 2] ^= 0x01; /* a data byte of a write */
      link_build(frame, type, split_buf, len);
      ev("tampered rx %02X", type);
    } else if (inject_take(&corrupt_rx, type)) {
      frame[3 + len] ^= 0x5A;
      ev("corrupted rx %02X", type);
    }
    for (i = 0; i < len + FRAME_OVERHEAD; i++)
      mcu_sim_rx(frame[i], us);
  }
}

static uint64_t press_until_ms[3];

static void set_buttons(uint64_t ms) {
  uint8_t i, b = 0;
  for (i = 0; i < 3; i++)
    if (press_until_ms[i] <= ms)
      b |= 1 << i;
  mcu_sim.buttons = b;
}

static char *json_counts(char *p, char *end, const char *name, const uint32_t *c) {
  int t, first = 1;
  p += snprintf(p, end - p, "\"%s\":{", name);
  for (t = 0; t < 256 && p < end; t++) {
    if (!c[t])
      continue;
    p += snprintf(p, end - p, "%s\"%02X\":%u", first ? "" : ",", t, (unsigned) c[t]);
    first = 0;
  }
  p += snprintf(p, end - p, "}");
  return p;
}

static void reply_get(char *out, size_t size) {
  char *p = out, *end = out + size - 4;
  const gate_t *g = &app_gate;
  int boot = mcu_sim.mode == SIM_BOOT;
  p += snprintf(p, end - p,
                "{\"mode\":\"%s\",\"uptime_ms\":%llu,\"state\":%u,\"target\":%u,\"current\":%u,\"relay\":%u,"
                "\"limits\":[%u,%u],\"leds\":%u,\"night\":%u,\"flash_valid\":%u,\"build\":\"%08X\","
                "\"boot_version\":%u,\"boot_id\":%u,\"boot_reason\":%u,\"boot_next\":%u,\"receiving\":%u,"
                "\"app_starts\":%u,\"boot_entries\":%u,\"rx_dropped\":%u,\"startup\":%u,\"buttons\":%u,"
                "\"therm_raw\":%u,\"wheel_raw\":%u,",
                MODE_NAMES[mcu_sim.mode], (unsigned long long) ((mcu_sim.now_us - mcu_sim.start_us) / 1000),
                g->state, g->target, g->state == ST_FIRING ? g->current : 0, g->relay, g->limit_brighter,
                g->limit_dimmer, app_status.leds, app_status.night, app_valid(), (unsigned) mcu_sim_app_build(),
                mcu_sim.boot_version, mcu_sim.boot_id, boot ? boot_update_reason : 0, boot ? boot_upd.next : 0,
                boot ? boot_upd.receiving : 0, (unsigned) mcu_sim.app_starts, (unsigned) mcu_sim.boot_entries,
                (unsigned) (rx_dropped + mcu_sim.busy_dropped), g->startup, mcu_sim.buttons, mcu_sim.therm_raw,
                mcu_sim.wheel_raw);
  p = json_counts(p, end, "rx", rx_count);
  p += snprintf(p, end - p, ",");
  p = json_counts(p, end, "tx", tx_count);
  snprintf(p, end - p, "}");
}

static void parse_inject(char *args, inject_t *rx, inject_t *tx, char *out, size_t size) {
  char dir[8] = "", type[16] = "";
  int n = 1;
  inject_t *i;
  int k = sscanf(args, "%7s %d %15s", dir, &n, type);
  i = !strcmp(dir, "rx") ? rx : !strcmp(dir, "tx") ? tx : NULL;
  if (k < 1 || !i) {
    snprintf(out, size, "err usage: %s [n] [type]", tx ? "rx|tx" : "rx");
    return;
  }
  i->n = n;
  i->type = k >= 3 ? (int) strtol(type, NULL, 0) : -1;
}

static void handle_command(char *line, char *out, size_t size, uint64_t ms) {
  char cmd[24] = "", *args;
  unsigned a = 0, b = 0;
  int k;
  while (*line == ' ')
    line++;
  if (sscanf(line, "%23s", cmd) < 1) {
    snprintf(out, size, "err empty");
    return;
  }
  args = line + strlen(cmd);
  k = sscanf(args, "%u %u", &a, &b);
  snprintf(out, size, "ok");
  if (!strcmp(cmd, "get")) {
    reply_get(out, size);
  } else if (!strcmp(cmd, "log")) {
    char *p = out, *end = out + size - 4;
    int i;
    *p++ = '[';
    for (i = 0; i < log_n && end - p > LOG_LINE + 4; i++)
      p += snprintf(p, end - p, "%s\"%s\"", i ? "," : "", log_buf[i]);
    snprintf(p, end - p, "]");
    log_n = 0;
  } else if (!strcmp(cmd, "press") && k >= 2 && a < 3) {
    press_until_ms[a] = ms + b;
  } else if (!strcmp(cmd, "txhold") && k >= 1) {
    /* Holds back what it sends for a while, then sends it at once, as the ESP32 sees it after a stall. */
    hold_until_us = now_us() + (uint64_t) a * 1000u;
  } else if (!strcmp(cmd, "ks") && k == 1) {
    mcu_sim.ks_manual = 1;
    mcu_sim.ks_level = (uint16_t) a;
  } else if (!strcmp(cmd, "addon")) {
    char what[8] = "";
    sscanf(args, "%7s", what);
    mcu_sim.addon = !strcmp(what, "up") ? 1 : !strcmp(what, "down") ? 2 : 0;
    mcu_sim.ks_manual = 0;
  } else if (!strcmp(cmd, "therm") && k >= 1) {
    mcu_sim.therm_raw = (uint16_t) a;
  } else if (!strcmp(cmd, "wheel") && k >= 1) {
    mcu_sim.wheel_raw = (uint16_t) a;
  } else if (!strcmp(cmd, "half") && k >= 1) {
    mcu_sim.half_counts = (uint16_t) a;
  } else if (!strcmp(cmd, "zo")) {
    mcu_sim.zo_on = strstr(args, "on") != NULL;
  } else if (!strcmp(cmd, "drop")) {
    parse_inject(args, &drop_rx, &drop_tx, out, size);
  } else if (!strcmp(cmd, "corrupt")) {
    parse_inject(args, &corrupt_rx, NULL, out, size);
  } else if (!strcmp(cmd, "tamper")) {
    parse_inject(args, &tamper_rx, NULL, out, size);
  } else if (!strcmp(cmd, "powercut")) {
    mcu_sim_power_cut(k >= 1 ? a : 100);
    ev("power cut");
  } else if (!strcmp(cmd, "wdtreset")) {
    if (mcu_sim.mode == SIM_APP && strstr(args, "cut") != NULL)
      send_cut_status();
    if (mcu_sim.mode != SIM_OFF)
      mcu_sim_reset(1);
  } else if (!strcmp(cmd, "updatemode")) {
    /* As if the application had taken an update request (MSG_UPDATE) and reset into update mode. */
    if (mcu_sim.mode != SIM_OFF) {
      handover[HO_REQUEST] = HO_REQUEST_UPDATE;
      mcu_sim_reset(0);
    }
  } else if (!strcmp(cmd, "auxram")) {
    mcu_sim.aux_ram_survives_reset = strstr(args, "false") == NULL;
  } else if (!strcmp(cmd, "idle_return_ms") && k >= 1) {
    boot_idle_return_ms = (uint16_t) a;
  } else if (!strcmp(cmd, "clear")) {
    memset(rx_count, 0, sizeof rx_count);
    memset(tx_count, 0, sizeof tx_count);
    rx_dropped = mcu_sim.busy_dropped = 0;
    log_n = 0;
  } else if (!strcmp(cmd, "help")) {
    snprintf(out, size,
             "get | log | press <0-2> <ms> | txhold <ms> | ks <v> | addon up|down|idle | therm <raw> | "
             "wheel <raw> | half <counts> | zo on|off | drop rx|tx [n] [type] | corrupt|tamper rx [n] [type] | "
             "powercut [ms] | wdtreset [cut] | updatemode | auxram true|false | idle_return_ms <ms> | clear");
  } else {
    snprintf(out, size, "err unknown or bad arguments: %s", cmd);
  }
}

typedef struct {
  int fd;
  char buf[512];
  size_t n;
} client_t;
static client_t clients[MAX_CLIENTS];
static int listen_fd = -1;

static void client_input(client_t *cl, uint64_t ms) {
  static char out[256 * 1024];
  char *nl;
  ssize_t r = read(cl->fd, cl->buf + cl->n, sizeof cl->buf - 1 - cl->n);
  if (r <= 0) {
    close(cl->fd);
    cl->fd = -1;
    return;
  }
  cl->n += r;
  cl->buf[cl->n] = 0;
  while ((nl = strchr(cl->buf, '\n'))) {
    size_t len;
    *nl = 0;
    if (nl > cl->buf && nl[-1] == '\r')
      nl[-1] = 0;
    handle_command(cl->buf, out, sizeof out - 1, ms);
    len = strlen(out);
    out[len++] = '\n';
    if (write(cl->fd, out, len) != (ssize_t) len)
      perror("control write");
    cl->n -= nl + 1 - cl->buf;
    memmove(cl->buf, nl + 1, cl->n + 1);
  }
  if (cl->n == sizeof cl->buf - 1)
    cl->n = 0;
}

static void cleanup(void) {
  if (opt_link)
    unlink(opt_link);
  if (opt_ctl && opt_ctl[0] != '@')
    unlink(opt_ctl);
}

static void on_signal(int sig) {
  (void) sig;
  cleanup();
  _exit(0);
}

static void open_pty(void) {
  struct termios tio;
  char *name;
  int slave;
  pty_fd = posix_openpt(O_RDWR | O_NOCTTY);
  if (pty_fd < 0 || grantpt(pty_fd) || unlockpt(pty_fd) || !(name = ptsname(pty_fd))) {
    perror("pty");
    exit(1);
  }
  /* Raw mode on the slave side now, so nothing is echoed before ESPHome opens it. */
  slave = open(name, O_RDWR | O_NOCTTY);
  if (slave < 0 || tcgetattr(slave, &tio)) {
    perror(name);
    exit(1);
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tcsetattr(slave, TCSANOW, &tio);
  close(slave);
  fcntl(pty_fd, F_SETFL, O_NONBLOCK);
  if (opt_link) {
    unlink(opt_link);
    if (symlink(name, opt_link)) {
      perror(opt_link);
      exit(1);
    }
  }
  printf("PTY %s\n", name);
}

static void open_ctl(void) {
  struct sockaddr_un sa;
  int i;
  for (i = 0; i < MAX_CLIENTS; i++)
    clients[i].fd = -1;
  if (!opt_ctl)
    return;
  listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  memset(&sa, 0, sizeof sa);
  sa.sun_family = AF_UNIX;
  /* "@name" is a Linux abstract socket: no file, and no limit from the directory's path length. */
  if (strlen(opt_ctl) >= sizeof sa.sun_path) {
    fprintf(stderr, "control socket path too long: %s\n", opt_ctl);
    exit(1);
  }
  memcpy(sa.sun_path, opt_ctl, strlen(opt_ctl));
  if (opt_ctl[0] == '@')
    sa.sun_path[0] = 0;
  else
    unlink(opt_ctl);
  if (listen_fd < 0 ||
      bind(listen_fd, (struct sockaddr *) &sa,
           (socklen_t) (offsetof(struct sockaddr_un, sun_path) + strlen(opt_ctl))) ||
      listen(listen_fd, 4)) {
    perror(opt_ctl);
    exit(1);
  }
}

static void usage(void) {
  fprintf(stderr,
          "usage: fake_mcu --flash FILE [--image AREA] [--invalid] [--link PATH] [--ctl SOCKET]\n"
          "                [--idle-return-ms MS] [--boot-version N] [--pad N]\n");
  exit(2);
}

int main(int argc, char **argv) {
  int i;
  uint64_t tick_ms = 0;
  mcu_sim_init();
  for (i = 1; i < argc; i++) {
    const char *a = argv[i];
    const char *v = i + 1 < argc ? argv[i + 1] : NULL;
    if (!strcmp(a, "--invalid")) {
      opt_invalid = 1;
    } else if (!v) {
      usage();
    } else if (!strcmp(a, "--link")) {
      opt_link = argv[++i];
    } else if (!strcmp(a, "--ctl")) {
      opt_ctl = argv[++i];
    } else if (!strcmp(a, "--flash")) {
      opt_flash = argv[++i];
    } else if (!strcmp(a, "--image")) {
      opt_image = argv[++i];
    } else if (!strcmp(a, "--idle-return-ms")) {
      boot_idle_return_ms = (uint16_t) atol(argv[++i]);
    } else if (!strcmp(a, "--boot-version")) {
      mcu_sim.boot_version = (uint8_t) atoi(argv[++i]);
    } else if (!strcmp(a, "--pad")) {
      opt_pad = (uint8_t) atoi(argv[++i]);
    } else {
      usage();
    }
  }
  if (!opt_flash)
    usage();

  clock_gettime(CLOCK_MONOTONIC, &t0);
  signal(SIGTERM, on_signal);
  signal(SIGINT, on_signal);
  signal(SIGPIPE, SIG_IGN);
  flash_open();
  open_pty();
  open_ctl();
  link_rx_init(&split, split_buf, BOOT_MAX_PAYLOAD);
  mcu_sim.tx = on_tx;
  mcu_sim.frame_in = on_frame_in;
  mcu_sim.flash_changed = flash_sync;
  set_buttons(0);
  mcu_sim_power_on();
  log_changes();
  printf("READY\n");
  fflush(stdout);

  for (;;) {
    struct pollfd pf[2 + MAX_CLIENTS];
    int np = 0, ci[MAX_CLIENTS], nc = 0;
    uint64_t us = now_us(), ms;
    struct timespec ts;
    uint64_t wait = 1000 - us % 1000;
    ts.tv_sec = 0;
    ts.tv_nsec = (long) wait * 1000;
    pf[np].fd = pty_fd;
    pf[np++].events = POLLIN;
    if (listen_fd >= 0) {
      pf[np].fd = listen_fd;
      pf[np++].events = POLLIN;
    }
    for (i = 0; i < MAX_CLIENTS; i++) {
      if (clients[i].fd < 0)
        continue;
      ci[nc++] = i;
      pf[np].fd = clients[i].fd;
      pf[np++].events = POLLIN;
    }
    ppoll(pf, np, &ts, NULL);
    us = now_us();
    ms = us / 1000;

    /* The simulation catches up to now before anything new happens. */
    while (tick_ms < ms) {
      tick_ms++;
      set_buttons(tick_ms);
      mcu_sim_run(tick_ms * 1000);
      log_changes();
    }
    mcu_sim_run(us);

    /* Without a reader on the slave side, POLLHUP is set and read gives EIO: nothing to read. */
    if (pf[0].revents & POLLIN) {
      uint8_t buf[1024];
      ssize_t r;
      while ((r = read(pty_fd, buf, sizeof buf)) > 0)
        rx_bytes(buf, r, us);
    } else if (pf[0].revents & POLLHUP) {
      struct timespec nap = {0, 1000000};
      nanosleep(&nap, NULL);
    }
    if (listen_fd >= 0 && (pf[1].revents & POLLIN)) {
      int fd = accept(listen_fd, NULL, NULL);
      for (i = 0; fd >= 0 && i < MAX_CLIENTS; i++) {
        if (clients[i].fd < 0) {
          clients[i].fd = fd;
          clients[i].n = 0;
          fd = -1;
        }
      }
      if (fd >= 0)
        close(fd);
    }
    for (i = 0; i < nc; i++) {
      if (pf[np - nc + i].revents & (POLLIN | POLLHUP))
        client_input(&clients[ci[i]], ms);
    }
    mcu_sim_run(us);
    log_changes();
    out_flush(now_us());
  }
}
