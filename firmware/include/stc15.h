/*
 * STC15W401AS-series registers used by this firmware (SDCC only). Addresses from STC's STC15
 * datasheet (2016), sections 1.5.3, 4.7, 7.1, 8.1, 9.1, 10.2, 11.1 and 2.2.6.
 */
#ifndef STC15_H
#define STC15_H

#ifndef __SDCC
#error "stc15.h is for the SDCC build only"
#endif

#include "board.h"
#include "protocol.h"

__sfr __at(0x87) PCON;
__sfr __at(0x89) TMOD;
__sfr __at(0x8A) TL0;
__sfr __at(0x8C) TH0;
__sfr __at(0x8E) AUXR;
__sfr __at(0x8F) INT_CLKO;
__sfr __at(0x91) P1M1;
__sfr __at(0x92) P1M0;
__sfr __at(0x95) P2M1;
__sfr __at(0x96) P2M0;
__sfr __at(0x98) SCON;
__sfr __at(0x99) SBUF;
__sfr __at(0x9D) P1ASF;
__sfr __at(0xA0) P2;
__sfr __at(0xA2) P_SW1;
__sfr __at(0xA8) IE;
__sfr __at(0xB0) P3;
__sfr __at(0xB1) P3M1;
__sfr __at(0xB2) P3M0;
__sfr __at(0xB8) IP;
__sfr __at(0xBC) ADC_CONTR;
__sfr __at(0xBD) ADC_RES;
__sfr __at(0xBE) ADC_RESL;
__sfr __at(0xC1) WDT_CONTR;
__sfr __at(0xC2) IAP_DATA;
__sfr __at(0xC3) IAP_ADDRH;
__sfr __at(0xC4) IAP_ADDRL;
__sfr __at(0xC5) IAP_CMD;
__sfr __at(0xC6) IAP_TRIG;
__sfr __at(0xC7) IAP_CONTR;
__sfr __at(0xD6) T2H;
__sfr __at(0xD7) T2L;
__sfr __at(0xD8) CCON;
__sfr __at(0xD9) CMOD;
__sfr __at(0xDA) CCAPM0;
__sfr __at(0xDB) CCAPM1;
__sfr __at(0xDC) CCAPM2;
__sfr __at(0xE9) CL;
__sfr __at(0xEA) CCAP0L;
__sfr __at(0xEC) CCAP2L;
__sfr __at(0xF4) PCA_PWM2;
__sfr __at(0xF9) CH;
__sfr __at(0xFA) CCAP0H;
__sfr __at(0xFC) CCAP2H;

__sbit __at(0x88) IT0;
__sbit __at(0x8C) TR0;
__sbit __at(0x8D) TF0;
__sbit __at(0x98) RI;
__sbit __at(0x99) TI;
__sbit __at(0xA0) PIN_BTN_UP;
__sbit __at(0xA7) PIN_BTN_ONOFF;
__sbit __at(0xA8) EX0;
__sbit __at(0xA9) ET0;
__sbit __at(0xAC) ES;
__sbit __at(0xAF) EA;
__sbit __at(0xB2) PIN_ZO;
__sbit __at(0xB3) PIN_BTN_DOWN;
__sbit __at(0xB4) PIN_RELAY;
__sbit __at(0xB5) PIN_SCR;
__sbit __at(0xDE) CR;

#define SCON_8BIT_RX 0x50u    /* mode 1, receiver on */
#define TMOD_T0_RELOAD 0x00u  /* timer 0: 16-bit auto-reload */
#define AUXR_T0X12 0x80u      /* timer 0 counts SYSclk, not SYSclk / 12 */
#define AUXR_T2R 0x10u
#define AUXR_T2X12 0x04u
#define AUXR_S1ST2 0x01u      /* UART1 baud from timer 2 */
#define AUXR_TIMERS (AUXR_T0X12 | AUXR_T2R | AUXR_T2X12 | AUXR_S1ST2)
/* Timer 2 counting SYSclk; UART1 divides its overflow rate by 4. */
#define T2_RELOAD_BAUD (65536ul - SYSCLK_HZ / 4u / LINK_BAUD)
#if SYSCLK_HZ % (4u * LINK_BAUD)
#error "LINK_BAUD is not exact at SYSCLK_HZ"
#endif
#define T0_RELOAD_1MS (65536ul - SYSCLK_HZ / 1000u)

/* Port modes: PxM1 and PxM0 bits 00 quasi-bidirectional, 01 push-pull, 10 input only. */
#define PORT_PUSH_PULL(m1, m0, bits) \
  do { \
    m1 &= ~(bits); \
    m0 |= (bits); \
  } while (0)
#define PORT_INPUT_ONLY(m1, m0, bits) \
  do { \
    m1 |= (bits); \
    m0 &= ~(bits); \
  } while (0)

#define P_SW1_UART1_P3 0x00u  /* UART1 on P3.0/P3.1 */
#define P_SW1_CCP_P3 0x10u    /* PCA on P3.4 to P3.7 */
#define IP_PPCA 0x80u
#define IP_PX0 0x01u
#define IE_EA 0x80u
#define PCON_POF 0x10u
#define WDT_FLAG 0x80u
#define WDT_EN 0x20u
#define WDT_CLR 0x10u
#define WDT_PS 2u
#define WDT_PRESCALE (2u << WDT_PS)
#define WDT_RUN_CLEAR (WDT_EN | WDT_CLR | WDT_PS)
#define WDT_TIMEOUT_MS (12u * WDT_PRESCALE * 32768ul * 1000u / SYSCLK_HZ)
#define CMOD_SYSCLK_12 0x00u
#define CCF0 0x01u
#define CCON_CR 0x40u
#define CCAPM_ECOM 0x40u
#define CCAPM_MAT 0x08u
#define CCAPM_TOG 0x04u
#define CCAPM_PWM_MODE 0x02u
#define CCAPM_ECCF 0x01u
#define CCAPM_HSO_INT (CCAPM_ECOM | CCAPM_MAT | CCAPM_TOG | CCAPM_ECCF)
#define CCAPM_PWM (CCAPM_ECOM | CCAPM_PWM_MODE)
#define ADC_POWER 0x80u
#define ADC_FLAG 0x10u
#define ADC_START 0x08u
#define IAP_IAPEN 0x80u
#define IAP_WAIT_12MHZ 0x03u
#define IAP_ENABLE (IAP_IAPEN | IAP_WAIT_12MHZ)
#define IAP_CMD_WRITE 2u
#define IAP_CMD_ERASE 3u
#define IAP_CMD_FAIL 0x10u
#define IAP_SWRST_APP 0x20u

#endif
