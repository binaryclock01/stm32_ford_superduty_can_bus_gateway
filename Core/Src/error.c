/*
 * error.c
 *
 * Implements error handling utilities for the application.
 */

#include <string.h>  // For string manipulation
#include <stdio.h>   // For snprintf
#include <stdbool.h> // For boolean support
#include "error.h"
#include "ui.h"      // For sending console messages

/**
 * @brief Handle errors based on the provided error code and optional additional information.
 *
 * This function generates a detailed error message string using a combination of
 * the error code, predefined error messages, and optional additional context.
 * It ensures buffer safety and logs the error message to the console. Critical errors
 * invoke the system's main error handler.
 *
 * @param error_code The error code indicating the type of error.
 * @param additional_info Optional additional information to provide more context for the error.
 */
void user_error_handler(uint8_t error_code, const char *additional_info) {
    const char *error_prefix = "ER";              // Prefix for all error messages
    char error_msg[MAX_ERROR_STRING_LENGTH] = ""; // Buffer for the base error message
    char error_msg_final[MAX_ERROR_STRING_LENGTH] = ""; // Buffer for the full error message
    bool call_main_error_handler = true;          // Flag to decide whether to invoke the main handler

    // Determine the base error message based on the error code
    switch (error_code) {
        case ERROR_CAN_MODULE_NOT_FOUND:
            strncpy(error_msg, "CAN module not found", sizeof(error_msg) - 1);
            break;
        case ERROR_MODULE_PID_NOT_FOUND:
            strncpy(error_msg, "PID not found", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_INVALID_PAYLOAD:
            strncpy(error_msg, "Invalid CAN payload", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_INIT_FAILED:
            strncpy(error_msg, "Failed to initialize CAN", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_NOTIFICATION_FAILED:
            strncpy(error_msg, "Failed to activate CAN notifications", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_FILTER_CONFIG_FAILED:
            strncpy(error_msg, "Failed to configure CAN filter", sizeof(error_msg) - 1);
            break;
        case ERROR_NO_ERROR:
            call_main_error_handler = false;  // Do not call the main error handler
            break;
        default:
            strncpy(error_msg, "Unknown error", sizeof(error_msg) - 1);
            break;
    }

    // Construct the full error message string
    int result = snprintf(
        error_msg_final, sizeof(error_msg_final), "%s#%u: %s%s%s",
        error_prefix,
        (uint8_t)error_code,
        error_msg,
        additional_info ? " - " : "",
        additional_info ? additional_info : ""
    );

    // Handle potential truncation
    if (result < 0 || result >= (int)sizeof(error_msg_final)) {
        strncpy(error_msg_final, "Truncated error message", sizeof(error_msg_final) - 1);
        error_msg_final[sizeof(error_msg_final) - 1] = '\0'; // Ensure null-termination
    }

    // Log the error message
    send_console_msg(error_msg_final);

    // Invoke the main error handler if necessary
    if (call_main_error_handler) {
        Error_Handler();
    }
}
