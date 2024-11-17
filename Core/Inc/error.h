/*
 * error.h
 *
 * Provides error handling utilities for the application.
 */

#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>               // For fixed-width integer types
#include <stdbool.h>              // For boolean support

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define MAX_ERROR_STRING_LENGTH 100 /**< Maximum length for error message strings */

/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

/**
 * @brief Enumeration of error codes used in the application.
 */
typedef enum {
    ERROR_NO_ERROR = 0,             /**< No error occurred */
    ERROR_CAN_MODULE_NOT_FOUND,     /**< CAN module not found */
    ERROR_MODULE_PID_NOT_FOUND,     /**< PID not found in module */
    ERROR_CAN_TRANSMIT_FAILED,      /**< CAN transmission failure */
    ERROR_CAN_INVALID_PAYLOAD,      /**< Invalid payload received */
    ERROR_CAN_INIT_FAILED,          /**< CAN initialization failure */
    ERROR_CAN_NOTIFICATION_FAILED,  /**< CAN notification activation failure */
    ERROR_CAN_FILTER_CONFIG_FAILED  /**< CAN filter configuration failure */
} ErrorCodes;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Handle errors based on the provided error code.
 * @param error_code The error code indicating the type of error.
 * @param additional_info (Optional) Additional context or data about the error.
 */
void user_error_handler(uint8_t error_code, const char *additional_info);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
