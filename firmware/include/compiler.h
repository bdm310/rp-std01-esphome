/* Portability between SDCC (the MCU build) and gcc (host tests). */
#ifndef COMPILER_H
#define COMPILER_H

#include <stdint.h>

#ifdef __SDCC
#define XDATA __xdata
#define IDATA __idata
#define DATA __data
#define CODE __code
#define REENTRANT __reentrant
#define AT(addr) __at(addr)
#else
#define XDATA
#define IDATA
#define DATA
#define CODE
#define REENTRANT
#define AT(addr)
#endif

#endif
