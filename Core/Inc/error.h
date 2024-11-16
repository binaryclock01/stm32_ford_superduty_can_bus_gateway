/*
 * error.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 */

#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>               // For fixed-width integer types
#include <stdbool.h>              // For boolean support
#include <device_configs.h>       // Include device configurations

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define MAX_ERROR_STRING_LENGTH 60  /**< Maximum length for error message strings */

/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

/**
 * @brief Enumeration of error codes used in the application.
 */
typedef enum {
    ERROR_NO_ERROR = 0,           /**< No error occurred */
    ERROR_CAN_MODULE_NOT_FOUND,   /**< CAN module not found */
    ERROR_MODULE_PID_NOT_FOUND    /**< PID not found in module */
} ErrorCodes;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Handle errors based on the provided error code.
 * @param error_code The error code indicating the type of error.
 */
void User_Error_Handler(uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
