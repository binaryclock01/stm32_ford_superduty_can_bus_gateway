/*
 * utils.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

/**
 * @brief Converts a byte array to an integer (uint16_t, uint32_t, or uint64_t) in big-endian order.
 *
 * @param dest Pointer to the integer to store the result.
 * @param src Pointer to the byte array to be converted.
 * @param size The size of the integer (2 for uint16_t, 4 for uint32_t, 8 for uint64_t).
 */

void bytes_to_big_endian(void *dest, const uint8_t *src, size_t size) {
    uint8_t *dest_bytes = (uint8_t *)dest;  // Treat `dest` as a byte array

    for (size_t i = 0; i < size; i++) {
        dest_bytes[size - i - 1] = src[i];
    }
}
/*
uint64_t bytes_to_uint32(uint8_t *bytes)
{
    return __builtin_bswap32(*(uint32_t *)bytes);
}
*/

uint32_t ms_to_ticks(uint32_t ms) {
    return (ms * osKernelGetTickFreq()) / 1000; // Convert milliseconds to ticks
}

uint32_t bytes_to_uint32(uint8_t *bytes) {
    uint32_t result = 0;
    for (int i = 0; i < 4; i++) {
        result |= ((uint32_t)bytes[i] << ((3-i) * 8)); // Little-endian construction
    }
    return result;
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



