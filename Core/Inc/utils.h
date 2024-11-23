/*
 * utils.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  // For fixed-width integer types

/* ---| TIME CONSTANTS |----------------------------------------------------- */
#define ONE_MILLISECOND 1       /**< One millisecond in timer ticks. */
#define ONE_SECOND (ONE_MILLISECOND * 1000) /**< One second in timer ticks. */

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Simulates an osDelayUntil functionality for CMSIS-RTOS v2.
 * @param lastWakeTime Pointer to the last wake time reference (in ticks).
 * @param period Period in ticks to delay until.
 */
void osDelayUntilCMSIS(uint32_t *lastWakeTime, uint32_t period);

uint32_t ms_to_ticks(uint32_t ms);

void bytes_to_big_endian(void *dest, const uint8_t *src, size_t size);
/**
 * @brief Convert a 4-byte array into a 32-bit unsigned integer (big-endian).
 * @param bytes Pointer to a 4-byte array.
 * @return A 64-bit value representing the converted 32-bit unsigned integer.
 */
uint32_t bytes_to_uint32(uint8_t *bytes);

/**
 * @brief Convert a 4-byte array into a 32-bit unsigned integer (big-endian).
 * @param bytes Pointer to a 4-byte array.
 * @return A 64-bit value representing the converted 32-bit unsigned integer.
 */
uint64_t bytes_to_uint64(uint8_t *bytes);

/**
 * @brief Combine two bytes (high and low) into a 16-bit unsigned integer.
 * @param high_byte The high-order byte.
 * @param low_byte The low-order byte.
 * @return A 16-bit unsigned integer combining the two bytes.
 */
uint16_t bytes_to_uint16(uint8_t high_byte, uint8_t low_byte);

/**
 * @brief Convert a non-zero integer to binary (1) and zero to binary (0).
 * @param i The input integer.
 * @return 1 if the input is non-zero, otherwise 0.
 */
int tobinary(uint32_t i);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
