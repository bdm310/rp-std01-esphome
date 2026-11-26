#ifndef CRC16_H
#define CRC16_H

#include "compiler.h"

/* CRC-16/CCITT-FALSE (polynomial 0x1021), one byte at a time. */
uint16_t crc16_update(uint16_t crc, uint8_t byte);

#endif
