/*
 * error.c
 *
 * Implements error handling utilities for the application.
 */


#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "buffers.h"
#include "log.h"
#include "ansi.h"
#include "error.h"


/* -----------------------------------------------------------------------------
   Error Table
   -------------------------------------------------------------------------- */

ErrorEntry error_table[] = {
    // General Errors (0-99)
    {ERROR_NO_ERROR,                "No error occurred",                   ERROR_CATEGORY_GENERAL, ERROR_SEVERITY_INFO,       0},
    {ERROR_INVALID_ARGUMENT,        "Invalid function argument",           ERROR_CATEGORY_GENERAL, ERROR_SEVERITY_WARNING,    0},
    {ERROR_UNKNOWN,                 "Unknown error occurred",              ERROR_CATEGORY_GENERAL, ERROR_SEVERITY_CRITICAL,   0},

    // CAN-Related Errors (100-199)
    {ERROR_CAN_MODULE_NOT_FOUND,    "CAN module not found",                ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_MODULE_PID_NOT_FOUND,    "PID not found",                       ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_TRANSMIT_FAILED,     "CAN transmission failed",             ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_INVALID_PAYLOAD,     "Invalid CAN payload",                 ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_BUFFER_OVERFLOW,     "CAN buffer overflow",                 ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_CAN_RETRIEVE_FAILED,     "CAN retrieve failed",                 ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_QUEUE_FULL,          "CAN message queue full",              ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_PACKET_NULL,         "Null CAN packet",                     ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_DATA_PARSE_FAILED,   "CAN data parsing failed",             ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_DEVICE_NOT_FOUND,    "CAN device not found",                ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_CAN_PID_NOT_FOUND,       "CAN PID not found",                   ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_INIT_FAILED,         "CAN initialization failed",           ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_CAN_NOTIFICATION_FAILED, "CAN notification activation failed",  ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
	{ERROR_CAN_FILTER_CONFIG_FAILED,"Failed to apply CAN filter",          ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},
	{ERROR_CAN_INVALID_PACKET, 		"Invalid CAN packet", 				   ERROR_CATEGORY_CAN, 	   ERROR_SEVERITY_WARNING,    0},
	{ERROR_CAN_PACKET_NOT_FOUND, 	"CAN packet not found", 			   ERROR_CATEGORY_CAN, 	   ERROR_SEVERITY_WARNING,    0},

	// RTOS-Related Errors (200-299)
    {ERROR_RTOS_QUEUE_INIT_FAILED,  "RTOS queue initialization failed",    ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_RTOS_QUEUE_FULL,         "RTOS queue full",                     ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_WARNING,    0},
    {ERROR_RTOS_MUTEX_TIMEOUT,      "RTOS mutex acquisition timed out",    ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_CRITICAL,   0},
    {ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release packet pool mutex", ERROR_CATEGORY_RTOS,  ERROR_SEVERITY_WARNING,    0},
	{ERROR_RTOS_MUTEX_INIT_FAILED,   "Failed to init pool mutex",          ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_WARNING,    0},
	{ERROR_RTOS_QUEUE_INVALID_HANDLE,"Invalid queue handle",			   ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_CRITICAL,   0},
	{ERROR_RTOS_QUEUE_NULL_PACKET,  "Recieved NULL packet from queue",     ERROR_CATEGORY_RTOS,    ERROR_SEVERITY_WARNING, 	  0},
	{ERROR_RTOS_QUEUE_ALLOCATION_FAILED,"Failed to allocate packet on queue.",ERROR_CATEGORY_RTOS, ERROR_SEVERITY_WARNING,    0},

    // UI-Related Errors (300-399)
    {ERROR_UI_RENDER_FAILED,        "UI rendering failed",                 ERROR_CATEGORY_UI,      ERROR_SEVERITY_WARNING,    0},
    {ERROR_UI_LOGGING_FAILED,       "Logging system failed",               ERROR_CATEGORY_UI,      ERROR_SEVERITY_WARNING,    0},

    // Storage-Related Errors (400-499)
    {ERROR_STORAGE_WRITE_FAILED,    "Failed to write to persistent storage", ERROR_CATEGORY_STORAGE, ERROR_SEVERITY_CRITICAL, 0},
    {ERROR_STORAGE_READ_FAILED,     "Failed to read from persistent storage", ERROR_CATEGORY_STORAGE, ERROR_SEVERITY_WARNING,  0},

    // New CAN Context Errors (500-599)
    {ERROR_CAN_QUEUE_UNAVAILABLE,   "Queue unavailable for CAN instance",  ERROR_CATEGORY_CAN,     ERROR_SEVERITY_WARNING,    0},
    {ERROR_CAN_INVALID_CONTEXT,     "Invalid CAN context in callback",     ERROR_CATEGORY_CAN,     ERROR_SEVERITY_CRITICAL,   0},

	// Heap creation errors (600-699)
	{ERROR_HEAP_CREATE_LOG_BUFFER,  "Failed to allocate log buffer in heap",ERROR_CATEGORY_HEAP,   ERROR_SEVERITY_CRITICAL,   0},
	{ERROR_HEAP_LOG_BUFFER_NULL,    "Global log buffer pointer is NULL",    ERROR_CATEGORY_HEAP,   ERROR_SEVERITY_CRITICAL,   0},
};



// Number of entries in the error table
const size_t error_table_size = sizeof(error_table) / sizeof(error_table[0]);




/* -----------------------------------------------------------------------------
   Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Retrieve the error message corresponding to an error code.
 *
 * This function searches the error table for a matching error code
 * and returns the associated error message. If the error code is not found,
 * it returns a default "Unknown error" message.
 *
 * @param error_code The error code to look up.
 * @return const char* Pointer to the error message string.
 */
const char *get_error_message(ErrorCodes error_code) {
    // Search for the error code in the error table
    for (size_t i = 0; i < sizeof(error_table) / sizeof(ErrorEntry); i++) {
        if (error_table[i].code == error_code) {
            return error_table[i].message; // Return the associated message
        }
    }

    // Return a default message if the error code is not found
    return "Unknown error code";
}

/**
 * @brief Handles an error, logs it, and optionally invokes the system error handler.
 */
void user_error_handler(ErrorCodes error_code, const char *format, ...) {
    char error_msg[MAX_ERROR_STRING_LENGTH] = "";  // Buffer for formatted error message
    va_list args;                                  // Variable argument list

    // Retrieve the error entry and increment its occurrence count
    ErrorEntry *entry = NULL;
    for (size_t i = 0; i < sizeof(error_table) / sizeof(ErrorEntry); i++) {
        if (error_table[i].code == error_code) {
            entry = &error_table[i];
            entry->count++; // Increment occurrence counter
            break;
        }
    }

    // Format additional context (if provided)
    if (format) {
        va_start(args, format);
        vsnprintf(error_msg, sizeof(error_msg), format, args);
        va_end(args);
    }

    // ensure to flush any status logs before printing the error, so that the error is not out of sync with
    // what led up to it.
    flush_logs();
    // Log the error to the console with category and severity
    printf(HGRN "* " REDB "\e[1;97mERROR #%u" CRESET "["  // Bright red background and bold white text for ERROR
           HBLU "%s" CRESET "] ("                         // Blue category in brackets
           BYEL "%s" CRESET "): "                         // Yellow severity
           BWHT "%s" CRESET                               // Bold white message
           "%s%s\n\r",
           error_code,
           entry ? (entry->category == ERROR_CATEGORY_CAN ? "CAN" :
                    entry->category == ERROR_CATEGORY_RTOS ? "RTOS" :
                    entry->category == ERROR_CATEGORY_STORAGE ? "Storage" :
                    entry->category == ERROR_CATEGORY_UI ? "UI" : "General") : "Unknown",
           entry ? (entry->severity == ERROR_SEVERITY_CRITICAL ? "CRITICAL" :
                    entry->severity == ERROR_SEVERITY_WARNING ? "WARNING" :
                    "INFO") : "Unknown",
           entry ? entry->message : "Error Unknown",
           format ? " - " : "",
           format ? error_msg: "");


    // Log the error to persistent storage
    //log_error_to_storage(error_code, format ? error_msg : NULL);

    // Invoke the main error handler for critical errors
    if (entry && entry->severity == ERROR_SEVERITY_CRITICAL) {
        Error_Handler();
    }
}

/**
 * @brief Logs the error to persistent storage.
 */
void log_error_to_storage(ErrorCodes error_code, const char *context) {
    // Example: Save the error to persistent storage (e.g., SD card or EEPROM)
    printf("Persisting error: Code=%u, Context=%s\n\r", error_code, context ? context : "None");
    // TODO: Implement actual storage logic here
}
