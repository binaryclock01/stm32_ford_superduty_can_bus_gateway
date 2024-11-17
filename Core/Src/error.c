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
 * Generates a detailed error message string and logs it to the console.
 * Critical errors invoke the system's main error handler.
 *
 * @param error_code The error code indicating the type of error.
 * @param additional_info Optional additional information to provide more context for the error.
 */
void user_error_handler(uint8_t error_code, const char *additional_info) {
    // Constants
    const char *error_prefix = "ER";              // Prefix for all error messages
    char error_msg[MAX_ERROR_STRING_LENGTH] = ""; // Buffer for the base error message
    char error_msg_final[MAX_ERROR_STRING_LENGTH] = ""; // Buffer for the full error message
    bool call_main_error_handler = true;          // Flag to decide whether to invoke the main handler

    // Step 1: Determine the base error message based on the error code
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
        case ERROR_CAN_TRANSMIT_FAILED:
            strncpy(error_msg, "CAN transmission failed", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_BUFFER_OVERFLOW:
            strncpy(error_msg, "CAN buffer overflow", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_FILTER_CONFIG_FAILED:
            strncpy(error_msg, "CAN filter configuration failed", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_INIT_FAILED:
            strncpy(error_msg, "CAN initialization failed", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_NOTIFICATION_FAILED:
            strncpy(error_msg, "CAN notification setup failed", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_RETRIEVE_FAILED:
            strncpy(error_msg, "CAN retrieve failed", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_PACKET_NULL:
            strncpy(error_msg, "Null CAN packet", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_DATA_PARSE_FAILED:
            strncpy(error_msg, "Failed to parse CAN data", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_DEVICE_NOT_FOUND:
            strncpy(error_msg, "No matching CAN device", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_PID_NOT_FOUND:
            strncpy(error_msg, "No matching PID for CAN device", sizeof(error_msg) - 1);
            break;
        case ERROR_NO_ERROR:
            call_main_error_handler = false; // No error, do not call main handler
            break;
        case ERROR_CAN_QUEUE_FULL:
            strncpy(error_msg, "CAN message queue full", sizeof(error_msg) - 1);
            break;
        case ERROR_CAN_QUEUE_INIT_FAILED:
        	strncpy(error_msg, "CAN queue init fail", sizeof(error_msg) - 1);
        	break;
        default:
            strncpy(error_msg, "Unknown error", sizeof(error_msg) - 1);
            break;
    }

    // Step 2: Construct the full error message
    int result = snprintf(
        error_msg_final, sizeof(error_msg_final), "%s#%u: %s%s%s",
        error_prefix,                      // Prefix for error messages (e.g., "ER")
        (uint8_t)error_code,               // The numeric error code
        error_msg,                         // The base error message
        additional_info ? " - " : "",      // Separator if additional info is provided
        additional_info ? additional_info : "" // Additional context for the error
    );

    // Handle potential truncation
    if (result < 0 || result >= (int)sizeof(error_msg_final)) {
        strncpy(error_msg_final, "Truncated error message", sizeof(error_msg_final) - 1);
        error_msg_final[sizeof(error_msg_final) - 1] = '\0';
    }

    // Step 3: Log the error message
    send_console_msg(error_msg_final);

    // Step 4: Call the main error handler if necessary
    if (call_main_error_handler) {
        Error_Handler();
    }
}
