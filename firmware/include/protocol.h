/* Link protocol between the ESP32 and the MCU. */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define LINK_BAUD 115200ul /* 8N1 */
#define LINK_BYTE_BITS 10u /* start, 8 data, stop */
#define FRAME_START 0xA5u
#define FRAME_HEADER 3u /* start, type, length */
#define FRAME_OVERHEAD ((unsigned) (FRAME_HEADER + 2u)) /* and the CRC */
#define CRC16_INIT 0xFFFFu
/* The application's UART rings. A ring holds one byte less than its size. */
#define HAL_RX_RING 32u
#define HAL_TX_RING 64u
#define WRITE_CHUNK 128u

#define MAX_LEN(a, b) ((a) > (b) ? (a) : (b))

#define MSG_GATE 0x01u    /* target u16, limit brighter u8, limit dimmer u8 */
#define GATE_LEN 4u
#define MSG_OUTPUTS 0x02u /* OUTPUTS_* LED bits u8, night light u8 */
#define OUTPUTS_LEN 2u
#define MSG_INFO_REQ 0x03u
#define MSG_UPDATE 0x04u
#define UPDATE_LEN 4u
#define APP_MAX_PAYLOAD ((unsigned) MAX_LEN(MAX_LEN(GATE_LEN, OUTPUTS_LEN), UPDATE_LEN))
#define UPDATE_KEY0 0x5Au
#define UPDATE_KEY1 0xA5u
#define UPDATE_KEY2 0x0Fu
#define UPDATE_KEY3 0xF0u

#define MSG_STATUS 0x81u
#define MSG_INFO 0x82u
#define INFO_LEN 10u

/* Status payload offsets. Words are little-endian. */
#define SF_STATE 0u
#define SF_FAULTS 1u
#define SF_BUTTONS 2u
#define SF_TARGET 3u  /* u16 */
#define SF_CURRENT 5u /* u16 */
#define SF_HALF 7u    /* u16 */
#define SF_KS_MIN 9u  /* u16 */
#define SF_FLAGS 11u
#define SF_THERM 12u  /* u16 */
#define SF_WHEEL 14u  /* u16 */
#define SF_BOOT_ID 16u
#define SF_LEDS 17u
#define SF_NIGHT 18u
#define STATUS_LEN (SF_NIGHT + 1u)

#define MSG_BOOT_STATUS 0x90u
#define MSG_BOOT_BEGIN 0x11u
#define BOOT_BEGIN_LEN 2u /* size u16 */
#define MSG_BOOT_WRITE 0x12u
#define BOOT_WRITE_LEN (2u + WRITE_CHUNK) /* offset u16, then the chunk */
#define MSG_BOOT_FINISH 0x13u
#define MSG_BOOT_RUN 0x14u
#define BOOT_MAX_PAYLOAD MAX_LEN(BOOT_BEGIN_LEN, BOOT_WRITE_LEN)
#if WRITE_CHUNK & (WRITE_CHUNK - 1u) || BOOT_MAX_PAYLOAD > 0xFFu
#error "WRITE_CHUNK must be a power of 2, and every frame must fit the length byte"
#endif
#define BOOT_STATUS_LEN 6u
#define BOOT_STATUS_PERIOD_MS 500u
#define BOOT_VERSION 1u
#if BOOT_VERSION >= 0xFFu
#error "BOOT_VERSION 0xFF reads as erased flash"
#endif
/* Update mode entered by MSG_UPDATE returns to a valid application after this long without a
   frame for the bootloader, unless an update has begun and not finished. */
#define IDLE_RETURN_MS 60000u

/* MSG_OUTPUTS LED bits, 1 = lit. */
#define OUTPUTS_LED1 0x01u
#define OUTPUTS_LED2 0x02u
#define OUTPUTS_LED3 0x04u
#define OUTPUTS_LED4 0x08u
#define OUTPUTS_LED5 0x10u
#define OUTPUTS_B3 0x20u

/* Gate states, at SF_STATE. */
#define ST_OFF 0u
#define ST_SETTLE 1u
#define ST_FIRING 2u
#define ST_STOPPING 3u
#define ST_FAULT 4u

/* At SF_BUTTONS: the level of each button (high = released), and from BTN_CHANGED_SHIFT on,
   whether it changed within the window. */
#define BTN_ONOFF 0x01u
#define BTN_UP 0x02u
#define BTN_DOWN 0x04u
#define BTN_ALL (BTN_ONOFF | BTN_UP | BTN_DOWN)
#define BTN_CHANGED_SHIFT 4u

#define KS_NO_SAMPLE 0xFFFFu

/* At SF_FLAGS. */
#define S_RELAY 0x01u    /* relay drive on */
#define S_KS_STALE 0x02u /* the Ks minimum may not belong to that drive state */
#define S_STARTING 0x04u /* the application has just started: thermistor average not ready yet */
#define S_ZO_COASTED 0x08u /* the phase lock ran on without a falling Zo edge near its prediction */

/* At SF_FAULTS. */
#define F_OVERTEMP 0x01u
#define F_THERM 0x02u
#define F_ZO 0x04u      /* Zo not good (informational while the relay is off) */
#define F_GATE 0x08u
#define F_CLAMPED 0x10u

/* Bootloader results. */
#define BR_OK 0u
#define BR_BAD_STATE 1u
#define BR_BAD_SIZE 2u
#define BR_BAD_OFFSET 3u
#define BR_VERIFY 4u
#define BR_MISMATCH 5u
#define BR_BAD_IMAGE 6u
#define BR_BAD_FRAME 7u
#define BR_NO_COMMAND 8u /* no frame handled since update mode started */

/* Why the bootloader is in update mode. */
#define BOOT_REASON_REQUEST 1u
#define BOOT_REASON_INVALID 2u
#define BOOT_REASON_WATCHDOG 3u
#define BOOT_WDT_RESETS 3u /* watchdog resets in a row for BOOT_REASON_WATCHDOG */

#endif
