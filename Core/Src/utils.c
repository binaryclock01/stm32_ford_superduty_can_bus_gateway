/*
 * utils.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */

#include <stdint.h>
#include <stdio.h>

uint64_t bytes_to_uint32(uint8_t *bytes)
{
    return __builtin_bswap32(*(uint32_t *)bytes);
}

uint64_t bytes_to_uint64(uint8_t *bytes) {
    uint64_t result = 0;
    for (int i = 0; i < 8; i++) {
        result |= ((uint64_t)bytes[i] << ((7-i) * 8)); // Little-endian construction
    }
    return result;
}

// required because of little-endian
uint16_t bytes_to_uint16(uint8_t high_byte, uint8_t low_byte) {
    return ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
}

int tobinary(uint32_t i)
{
	if (i > 0)
		return 1;
	return 0;
}



