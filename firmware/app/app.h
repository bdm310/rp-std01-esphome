/* The application's main loop. Hardware goes through hal.h, so gcc builds this for the simulator. */
#ifndef APP_H
#define APP_H

#include "compiler.h"
#include "gate.h"
#include "status.h"

extern XDATA gate_t app_gate;
extern XDATA status_in_t app_status;

/* Call after hal_init(). */
void app_init(void);
/* One pass of the main loop: received bytes until the 1 ms tick advances, then that tick's work. */
void app_loop(void);

#endif
