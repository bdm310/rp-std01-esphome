#ifndef HAL_H
#define HAL_H

#include "compiler.h"
#include "protocol.h"
#include "window.h"

_Static_assert(!(HAL_RX_RING & (HAL_RX_RING - 1)) && !(HAL_TX_RING & (HAL_TX_RING - 1)),
               "UART rings must be powers of 2");

/* Taken by the main loop once per tick. */
typedef struct {
  uint8_t edges;      /* half-cycle starts once locked, Zo edges before; wrapping */
  uint16_t half;      /* half-period, PCA counts */
  uint8_t zo_valid;   /* phase locked */
  uint8_t gate_fault; /* a gate fault since the last call */
  uint8_t zo_coasted; /* for S_ZO_COASTED */
  uint16_t therm_avg;
  uint8_t therm_ready;
  uint16_t wheel_avg;
} hal_in_t;

void hal_init(void);
uint16_t hal_ms(void);
void hal_wdt_clear(void);
void hal_take_inputs(hal_in_t XDATA *in);
/* Copies the input window to w and starts a new one. */
void hal_take_window(window_t XDATA *w);
void hal_set_fire(uint16_t delay);
void hal_set_relay(uint8_t on);
/* leds: OUTPUTS_* LED bits. night: PWM 0-255. */
void hal_set_outputs(uint8_t leds, uint8_t night);
uint8_t hal_uart_read(uint8_t DATA *b);
/* Queues a whole frame; returns 0 (and queues nothing) if there is no room. */
uint8_t hal_uart_send(const uint8_t XDATA *buf, uint8_t len);
/* Restarts into the bootloader's update mode. Returns only in the simulator. */
void hal_enter_bootloader(void);
uint8_t hal_reset_cause(void);
uint8_t hal_wdt_count(void);
void hal_clear_wdt_count(void);
uint8_t hal_boot_id(void);
uint16_t hal_fw_version(void);
/* From the application's own trailer, so it always matches the image ESPHome sent, even after an
   incremental build. */
uint32_t hal_build_id(void);
uint8_t hal_boot_version(void);

#ifdef __SDCC
/*
 * Interrupt masking. On this 1T core an interrupt that is already pending when its enable bit is
 * cleared can still be taken after the next instruction (SiLabs documents the same for its 1T
 * 8051 core). Clearing the bit twice keeps such an interrupt out of the protected code.
 */
#define IRQ_OFF_ALL() \
  do { \
    EA = 0; \
    EA = 0; \
  } while (0)
#define IRQ_OFF_TICK() \
  do { \
    ET0 = 0; \
    ET0 = 0; \
  } while (0)
#define IRQ_OFF_UART() \
  do { \
    ES = 0; \
    ES = 0; \
  } while (0)

/* Interrupt handlers must be declared in the file that holds main(). */
void hal_zo_isr(void) __interrupt(0);
void hal_tick_isr(void) __interrupt(1);
void hal_uart_isr(void) __interrupt(4);
void hal_pca_isr(void) __interrupt(7);
#endif

#endif
