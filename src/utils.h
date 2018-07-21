#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>

void write_uint8LE(uint8_t* buffer, const uint8_t value, const uint32_t byte_offset);
void write_uint16(uint8_t* buffer, const uint16_t value, const uint32_t byte_offset);
void write_uint16LE(uint8_t* buffer, const uint16_t value, const uint32_t byte_offset);
void write_uint32LE(uint8_t* buffer, const uint32_t value, const uint32_t byte_offset);
uint16_t read_uint16LE(uint8_t* buffer, const uint32_t byte_offset);
uint32_t read_uint32LE(uint8_t* buffer, const uint32_t byte_offset);
uint16_t crc_16(uint8_t* data, uint32_t length, uint16_t initial_remainder);
void randombytes(uint8_t* buffer, uint64_t length);

#endif