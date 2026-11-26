/*
 * Flash map and the hand-over between bootloader and application. The sector from APP_END is
 * unused, which keeps clear of the ID bytes other STC15 parts keep at the end of flash.
 */
#ifndef FLASHMAP_H
#define FLASHMAP_H

#define APP_BASE 0x0E00u
#define APP_END 0x3200u /* exclusive */
#define APP_SIZE (APP_END - APP_BASE)
#define SECTOR_SIZE 512u
#define BOOT_VERSION_ADDR (APP_BASE - 1u)
/* The bootloader's C code, past its vectors and reset code. */
#define BOOT_CODE_BASE 0x00D0u

#if APP_BASE % SECTOR_SIZE || APP_END % SECTOR_SIZE
#error "the application area must start and end on sector boundaries"
#endif

#define TRAILER_SIZE 16u
#define TRAILER_ADDR (APP_END - TRAILER_SIZE)
#define APP_MAGIC "RPA1"
#define TR_MAGIC 0
#define TR_CODE_LEN 4   /* u16, little-endian */
#define TR_CRC 6        /* u16 */
#define TR_VERSION 8    /* u16 */
#define TR_BUILD 10     /* u32 */
#define TR_VALID 15     /* 0xFF as written, 0x00 once the bootloader has checked the CRC */

/* Auxiliary RAM the programs may use. */
#define XRAM_SIZE 0x00F0u

/*
 * Hand-over bytes in auxiliary RAM, above XRAM_SIZE. SDCC's start-up clears only each program's
 * own XSEG, so these survive. HO_MAGIC0 and HO_MAGIC1 come first, then the offsets below.
 */
#define HO_ADDR 0x00F8u
#define HO_REQUEST 2u     /* HO_REQUEST_UPDATE when the application asks for update mode */
#define HO_WDT_COUNT 3u   /* watchdog resets in a row */
#define HO_RESET_CAUSE 4u /* RESET_* */
#define HO_SIZE (HO_RESET_CAUSE + 1u)
#define HO_MAGIC0 0x5Au
#define HO_MAGIC1 0xA5u
#define HO_REQUEST_UPDATE 0xC3u
/* The bootloader never touches the boot ID. */
#define BOOT_ID_ADDR 0x00FDu

/* The linker keeps the programs below XRAM_SIZE, but not these placements (__at). */
#if HO_ADDR < XRAM_SIZE || HO_ADDR + HO_SIZE > BOOT_ID_ADDR || BOOT_ID_ADDR > 0xFFu
#error "hand-over bytes and boot ID must lie apart, between XRAM_SIZE and 0xFF"
#endif

#define RESET_POWER_ON 1u
#define RESET_WATCHDOG 2u
#define RESET_OTHER 3u

#endif
