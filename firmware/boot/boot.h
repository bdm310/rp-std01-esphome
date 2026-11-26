/* The bootloader's start-up decision and update mode. */
#ifndef BOOT_H
#define BOOT_H

#include "compiler.h"
#include "update.h"

#define BOOT_RX_RING 16u
#ifdef __SDCC
#define BOOT_IDLE_RETURN_MS IDLE_RETURN_MS
#else
extern uint16_t boot_idle_return_ms; /* the simulator can shorten it */
#define BOOT_IDLE_RETURN_MS boot_idle_return_ms
extern volatile uint8_t handover[];
#endif

extern IDATA upd_t boot_upd;
extern uint8_t boot_update_reason;

/* Updates the hand-over bytes for this reset. Returns why to stay in update mode (BOOT_REASON_*),
   or 0 to start the application. */
uint8_t boot_reason(uint8_t power_on, uint8_t watchdog);
/* Call after boot_hal_init(). */
void boot_init(uint8_t reason);
void boot_loop(void);

void boot_hal_iap_idle(void);
void boot_hal_init(void);
/* Moves a received byte from the UART into the receive ring. */
void boot_hal_take(void);
uint8_t boot_hal_read(uint8_t DATA *b);
/* Returns once the frame is sent, with the bytes received meanwhile in the receive ring. */
void boot_hal_send(const uint8_t XDATA *frame, uint8_t n);
/* 1 once per elapsed millisecond. */
uint8_t boot_hal_tick(void);
/* Software reset. Returns only in the simulator. */
void boot_hal_restart(void);

#endif
