/* The application's hardware layer: registers and interrupt handlers (SDCC only). */
#include "hal.h"
#include "board.h"
#include "flashmap.h"
#include "protocol.h"
#include "sched.h"
#include "stc15.h"

#define ADC_NONE 0xFFu
#define ADC_KS 1u
#define ADC_THERM 2u
#define ADC_WHEEL 3u

#define P1_KS 0x02
#define P1_THERM 0x04
#define P1_WHEEL 0x08
#define P1_ANALOG (P1_KS | P1_THERM | P1_WHEEL)
#define P2_LED5 0x02
#define P2_LED4 0x04
#define P2_LED3 0x08
#define P2_LED2 0x10
#define P2_LED1 0x20
#define P2_B3 0x40
#define P2_LEDS (P2_LED1 | P2_LED2 | P2_LED3 | P2_LED4 | P2_LED5 | P2_B3)
#define P3_RELAY 0x10
#define P3_SCR 0x20
#define P3_NIGHT_LIGHT 0x80
#define P3_OUTPUTS (P3_RELAY | P3_SCR | P3_NIGHT_LIGHT)

/* Module 0 first: while it is on, its next match could toggle the pin back high. */
#define GATE_FORCE_OFF() \
  do { \
    CCAPM0 = 0; \
    PIN_SCR = 0; \
  } while (0)

static IDATA sched_t hal_sched;
static IDATA window_t hal_window;
static IDATA avg8_t hal_therm;
static IDATA avg8_t hal_wheel;

static volatile uint16_t fire_delay;
static volatile uint16_t tick_ms;
static uint8_t adc_ch;
static uint8_t adc_slot;

static XDATA uint8_t rx_ring[HAL_RX_RING];
static volatile uint8_t rx_head, rx_tail;
static XDATA uint8_t tx_ring[HAL_TX_RING];
static volatile uint8_t tx_head, tx_tail;
static volatile uint8_t tx_busy;

static volatile __xdata __at(HO_ADDR) uint8_t handover[HO_SIZE];
static __xdata __at(BOOT_ID_ADDR) uint8_t boot_id;

static uint16_t pca_now(void) REENTRANT {
  uint8_t h, l;
  do {
    h = CH;
    l = CL;
  } while (h != CH);
  return ((uint16_t) h << 8) | l;
}

void hal_init(void) {
  boot_id++;

  /* The bootloader already drove these low; repeat in case the application is entered some
     other way. */
  GATE_FORCE_OFF();
  PIN_RELAY = 0;
  PORT_PUSH_PULL(P3M1, P3M0, P3_OUTPUTS);

  P2 = 0xFF;    /* LEDs off; button latches high */
  P2M1 = 0x00;
  P2M0 = P2_LEDS; /* push-pull; the buttons stay quasi-bidirectional */

  PORT_INPUT_ONLY(P1M1, P1M0, P1_ANALOG);
  P1ASF = P1_ANALOG;
  ADC_CONTR = ADC_POWER;
  adc_ch = ADC_NONE;
  adc_slot = 0;

  P_SW1 = P_SW1_UART1_P3 | P_SW1_CCP_P3;
  INT_CLKO = 0x00; /* T0CLKO on P3.5 must stay off */

  sched_init(&hal_sched);
  window_reset(&hal_window);
  avg8_init(&hal_therm);
  avg8_init(&hal_wheel);
  fire_delay = 0;
  tick_ms = 0;
  rx_head = rx_tail = tx_head = tx_tail = 0;
  tx_busy = 0;

  CCON = 0;
  CMOD = CMOD_SYSCLK_12;
  CL = 0;
  CH = 0;
  CCAPM0 = 0;
  CCAPM1 = 0;
  PCA_PWM2 = 0;
  CCAP2L = 0;
  CCAP2H = 0;
  CCAPM2 = CCAPM_PWM;
  CCON = CCON_CR;

  SCON = SCON_8BIT_RX;
  T2L = (uint8_t) T2_RELOAD_BAUD;
  T2H = (uint8_t) (T2_RELOAD_BAUD >> 8);
  TMOD = TMOD_T0_RELOAD;
  TL0 = (uint8_t) T0_RELOAD_1MS;
  TH0 = (uint8_t) (T0_RELOAD_1MS >> 8);
  AUXR = AUXR_TIMERS;
  TR0 = 1;

  IT0 = 0;     /* INT0 on both edges */
  IP = IP_PPCA | IP_PX0;
  EX0 = 1;
  ET0 = 1;
  ES = 1;
  IE |= IE_EA; /* the PCA interrupt is enabled per module by ECCF0 */
}

/* Read again until two reads agree, so that a tick between the two byte reads cannot give a
   value 256 ms out. This does not depend on how quickly ET0 = 0 takes effect. */
uint16_t hal_ms(void) {
  uint16_t t;
  do {
    t = tick_ms;
  } while (t != tick_ms);
  return t;
}

void hal_wdt_clear(void) { WDT_CONTR = WDT_RUN_CLEAR; }

/* The main loop reads what the interrupts write through volatile views, so that the compiler keeps
   the reads between the masking of the interrupts. The masked part is plain copies, kept short:
   the Zo interrupt's timestamp waits for it. */
void hal_take_inputs(hal_in_t XDATA *in) {
  volatile sched_t IDATA *s = &hal_sched;
  const volatile avg8_t IDATA *therm = &hal_therm, *wheel = &hal_wheel;
  uint16_t period;
  uint8_t edges, zo_valid, fault;
  IRQ_OFF_ALL();
  edges = s->edges;
  period = s->phase.period;
  zo_valid = s->zo_valid;
  fault = s->fault;
  s->fault = 0;
  EA = 1;
  in->edges = edges;
  in->half = period >> 1;
  in->zo_valid = zo_valid;
  in->gate_fault = fault;
  IRQ_OFF_TICK();
  in->therm_avg = therm->value;
  in->therm_ready = therm->ready;
  in->wheel_avg = wheel->value;
  ET0 = 1;
}

void hal_take_window(window_t XDATA *w) {
  IRQ_OFF_TICK();
  *w = *(const volatile window_t IDATA *) &hal_window;
  window_reset(&hal_window);
  ET0 = 1;
}

void hal_set_fire(uint16_t delay) {
  IRQ_OFF_ALL();
  fire_delay = delay;
  EA = 1;
}

void hal_set_relay(uint8_t on) { PIN_RELAY = on ? 1 : 0; }

void hal_set_outputs(uint8_t leds, uint8_t night) {
  uint8_t p = 0xFF;
  if (leds & OUTPUTS_LED1) p &= ~P2_LED1;
  if (leds & OUTPUTS_LED2) p &= ~P2_LED2;
  if (leds & OUTPUTS_LED3) p &= ~P2_LED3;
  if (leds & OUTPUTS_LED4) p &= ~P2_LED4;
  if (leds & OUTPUTS_LED5) p &= ~P2_LED5;
  if (leds & OUTPUTS_B3) p &= ~P2_B3;
  P2 = p;
  CCAP2H = night; /* reloaded into CCAP2L at the next PWM period */
}

uint8_t hal_uart_read(uint8_t DATA *b) {
  if (rx_head == rx_tail)
    return 0;
  *b = rx_ring[rx_tail];
  rx_tail = (rx_tail + 1) & (HAL_RX_RING - 1);
  return 1;
}

uint8_t hal_uart_send(const uint8_t XDATA *buf, uint8_t len) {
  uint8_t i, used;
  used = (tx_head - tx_tail) & (HAL_TX_RING - 1);
  if (len > HAL_TX_RING - 1 - used)
    return 0;
  for (i = 0; i < len; i++) {
    tx_ring[tx_head] = buf[i];
    tx_head = (tx_head + 1) & (HAL_TX_RING - 1);
  }
  IRQ_OFF_UART();
  if (!tx_busy) {
    tx_busy = 1;
    SBUF = tx_ring[tx_tail];
    tx_tail = (tx_tail + 1) & (HAL_TX_RING - 1);
  }
  ES = 1;
  return 1;
}

void hal_enter_bootloader(void) {
  IRQ_OFF_ALL();
  GATE_FORCE_OFF();
  PIN_RELAY = 0;
  handover[0] = HO_MAGIC0;
  handover[1] = HO_MAGIC1;
  handover[HO_REQUEST] = HO_REQUEST_UPDATE;
  IAP_CONTR = IAP_SWRST_APP;
  for (;;) {
  }
}

uint8_t hal_reset_cause(void) { return handover[HO_RESET_CAUSE]; }
uint8_t hal_wdt_count(void) { return handover[HO_WDT_COUNT]; }
void hal_clear_wdt_count(void) { handover[HO_WDT_COUNT] = 0; }
uint8_t hal_boot_id(void) { return boot_id; }

uint16_t hal_fw_version(void) { return FW_VERSION; }

uint32_t hal_build_id(void) {
  const uint8_t CODE *p = (const uint8_t CODE *) (TRAILER_ADDR + TR_BUILD);
  return p[0] | ((uint16_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

uint8_t hal_boot_version(void) { return *(const uint8_t CODE *) BOOT_VERSION_ADDR; }

void hal_zo_isr(void) __interrupt(0) {
  uint16_t now = pca_now();
  uint16_t t;
  uint8_t a = sched_edge(&hal_sched, now, PIN_ZO, fire_delay, &t);
  if (a == SA_NONE)
    return;
  GATE_FORCE_OFF();
  CCON &= ~CCF0;
  if (a == SA_ARM) {
    CCAP0L = (uint8_t) t; /* low byte first: it stops comparing, the high byte restarts it */
    CCAP0H = (uint8_t) (t >> 8);
    CCAPM0 = CCAPM_HSO_INT;
  }
}

void hal_pca_isr(void) __interrupt(7) {
  uint16_t t;
  if (!(CCON & CCF0))
    return;
  CCON &= ~CCF0;
  if (sched_match(&hal_sched, PIN_SCR, &t) == SA_RELEASE) {
    CCAP0L = (uint8_t) t;
    CCAP0H = (uint8_t) (t >> 8);
  } else {
    GATE_FORCE_OFF();
  }
}

void hal_tick_isr(void) __interrupt(1) {
  uint16_t v, now;
  uint8_t next;
  tick_ms++;

  if (adc_ch != ADC_NONE) {
    v = ((uint16_t) ADC_RES << 2) | (ADC_RESL & 0x03);
    if (adc_ch == ADC_KS)
      window_add_ks(&hal_window, v, PIN_RELAY);
    else if (adc_ch == ADC_THERM)
      avg8_add(&hal_therm, v);
    else
      avg8_add(&hal_wheel, v);
  }
  if (!(adc_slot & 1))
    next = ADC_KS;
  else if (adc_slot == 1)
    next = ADC_WHEEL;
  else if (adc_slot == 5)
    next = ADC_THERM;
  else
    next = ADC_NONE;
  if (++adc_slot == 10)
    adc_slot = 0;
  adc_ch = next;
  if (next != ADC_NONE)
    ADC_CONTR = ADC_POWER | ADC_START | next; /* 540 clocks; a plain write also clears ADC_FLAG */

  window_add_buttons(&hal_window, (PIN_BTN_ONOFF ? BTN_ONOFF : 0) | (PIN_BTN_UP ? BTN_UP : 0) |
                                      (PIN_BTN_DOWN ? BTN_DOWN : 0));

  if (sched_zo_lost(&hal_sched)) {
    IRQ_OFF_ALL();
    sched_zo_reset(&hal_sched);
    EA = 1;
  }
  /* Read before masking: if an interrupt delays the masking, the check runs as if a little
     earlier, which cannot make a release look overdue. */
  now = pca_now();
  IRQ_OFF_ALL();
  if (sched_tick_check(&hal_sched, PIN_SCR, now) == SA_OFF)
    GATE_FORCE_OFF();
  EA = 1;
}

void hal_uart_isr(void) __interrupt(4) {
  uint8_t next;
  if (RI) {
    RI = 0;
    next = (rx_head + 1) & (HAL_RX_RING - 1);
    if (next != rx_tail) {
      rx_ring[rx_head] = SBUF;
      rx_head = next;
    }
  }
  if (TI) {
    TI = 0;
    if (tx_tail != tx_head) {
      SBUF = tx_ring[tx_tail];
      tx_tail = (tx_tail + 1) & (HAL_TX_RING - 1);
    } else {
      tx_busy = 0;
    }
  }
}
