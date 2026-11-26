#include "crc16.h"

/* A whole byte at once, without a table: with x = (crc >> 8) ^ byte, x ^= x >> 4, the new CRC is
   (crc << 8) ^ (x << 12) ^ (x << 5) ^ x. Written in 8-bit steps, which SDCC compiles to about 70
   clocks instead of about 250 for the bit loop. */
uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  uint8_t x = (uint8_t) (crc >> 8) ^ byte;
  uint8_t lo;
  x ^= x >> 4;
  lo = (uint8_t) (x << 5) ^ x;
  return ((uint16_t) ((uint8_t) crc ^ (uint8_t) (x << 4) ^ (uint8_t) (x >> 3)) << 8) | lo;
}
