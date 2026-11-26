/*
 * The simulated MCU: the firmware's own application loop (firmware/app/app.c) and bootloader
 * (firmware/boot/boot.c) on simulated time, mains, inputs and flash, with the hardware layers
 * played by mcu_sim.c. One per program.
 *
 * Free of protocol.h, flashmap.h and board.h, whose macros would clash with rpstd01's C++ names.
 */
#ifndef MCU_SIM_H
#define MCU_SIM_H

#include <stddef.h>
#include <stdint.h>

#include "app.h"
#include "boot.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { SIM_OFF, SIM_APP, SIM_BOOT };

typedef struct {
  uint64_t now_us;
  int mode;

  /* The world, which a power cut leaves alone. */
  int zo_on;
  uint16_t half_counts;
  uint16_t therm_raw, wheel_raw;
  uint8_t buttons;    /* pin levels: bit 0 ON/OFF, 1 UP, 2 DOWN; 1 = released */
  uint8_t addon;      /* 0 idle, 1 UP, 2 DOWN: Ks follows the levels measured on a unit */
  int ks_manual;      /* Ks stays at ks_level instead */
  uint16_t ks_level;
  int aux_ram_survives_reset; /* any reset other than a power-on */
  int lost_ram_fixed; /* lost auxiliary RAM comes back as 0xEE every time, not as varying garbage */
  uint8_t boot_version; /* the byte at BOOT_VERSION_ADDR */

  uint64_t start_us;     /* the last reset */
  uint64_t off_until_us; /* SIM_OFF: power comes back */
  uint64_t busy_until_us; /* the bootloader erases, writes or checks: received bytes are lost */
  uint8_t boot_id;
  uint8_t relay_pin, leds_pin, night_pin;
  uint16_t fire;

  uint32_t app_starts, boot_entries;
  uint32_t busy_dropped; /* bytes lost while the bootloader was busy */

  /* Hooks, all optional. */
  /* A frame on its way to the ESP32: its first byte goes on the wire at at_us. */
  void (*tx)(const uint8_t *frame, uint8_t n, uint64_t at_us);
  /* A complete frame the MCU receives, before the firmware handles it. */
  void (*frame_in)(uint8_t type, const uint8_t *payload, uint8_t len);
  void (*flash_changed)(uint16_t addr, uint16_t n);
} mcu_sim_t;

extern mcu_sim_t mcu_sim;
/* Flash up to the end of the application area; the bootloader's own bytes are not used. */
extern uint8_t mcu_sim_flash[];
extern const uint32_t MCU_SIM_BYTE_US; /* one byte on the wire */

/* Defaults for everything but the flash, power off. */
void mcu_sim_init(void);
void mcu_sim_power_on(void);
/* A reset other than a power-on: watchdog, or software. */
void mcu_sim_reset(int watchdog);
void mcu_sim_power_cut(uint32_t ms);
/* Straight into update mode, as after a reset that chose it for this reason. */
void mcu_sim_update_mode(uint8_t reason);
/* A byte from the ESP32 that arrives at at_us, no earlier than those before it. */
void mcu_sim_rx(uint8_t b, uint64_t at_us);
void mcu_sim_run(uint64_t to_us);
uint16_t mcu_sim_app_version(void);
uint32_t mcu_sim_app_build(void);

#ifdef __cplusplus
}
#endif

#endif
