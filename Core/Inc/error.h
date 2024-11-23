#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

/* -----------------------------------------------------------------------------
   Enumerations for Error Handling
   -------------------------------------------------------------------------- */
#define MAX_ERROR_STRING_LENGTH 255

/**
 * @brief Enum for error categories to classify errors logically.
 */
typedef enum {
    ERROR_CATEGORY_GENERAL = 0,
    ERROR_CATEGORY_CAN,
    ERROR_CATEGORY_RTOS,
    ERROR_CATEGORY_UI,
    ERROR_CATEGORY_STORAGE,
	ERROR_CATEGORY_HEAP,
} ErrorCategory;

/**
 * @brief Enum for error severity levels.
 */
typedef enum {
    ERROR_SEVERITY_INFO = 0,
    ERROR_SEVERITY_WARNING,
    ERROR_SEVERITY_CRITICAL,
} ErrorSeverity;

/**
 * @brief Enum for error codes.
 */
typedef enum {
    // General Errors (0-99)
    ERROR_NO_ERROR = 0,
    ERROR_INVALID_ARGUMENT,
    ERROR_UNKNOWN, // Generic fallback error

    // CAN-Related Errors (100-199)
    ERROR_CAN_MODULE_NOT_FOUND = 100,
    ERROR_MODULE_PID_NOT_FOUND,
    ERROR_CAN_TRANSMIT_FAILED,
    ERROR_CAN_INVALID_PAYLOAD,
    ERROR_CAN_BUFFER_OVERFLOW,
    ERROR_CAN_RETRIEVE_FAILED,
    ERROR_CAN_QUEUE_FULL,
    ERROR_CAN_PACKET_NULL,
    ERROR_CAN_DATA_PARSE_FAILED, // Parsing failure
    ERROR_CAN_DEVICE_NOT_FOUND,
    ERROR_CAN_PID_NOT_FOUND,
    ERROR_CAN_INIT_FAILED,         // New: CAN initialization failure
    ERROR_CAN_NOTIFICATION_FAILED, // New: CAN notification activation failure
	ERROR_CAN_FILTER_CONFIG_FAILED,
	ERROR_CAN_INVALID_PACKET,
	ERROR_CAN_PACKET_NOT_FOUND,

    // RTOS-Related Errors (200-299)
    ERROR_RTOS_QUEUE_INIT_FAILED = 200,
    ERROR_RTOS_QUEUE_FULL,
    ERROR_RTOS_MUTEX_TIMEOUT,
    ERROR_RTOS_MUTEX_RELEASE_FAILED,
	ERROR_RTOS_MUTEX_INIT_FAILED,
	ERROR_RTOS_QUEUE_INVALID_HANDLE,
	ERROR_RTOS_QUEUE_NULL_PACKET,
	ERROR_RTOS_QUEUE_ALLOCATION_FAILED,

    // UI-Related Errors (300-399)
    ERROR_UI_RENDER_FAILED = 300,
    ERROR_UI_LOGGING_FAILED,

    // Storage-Related Errors (400-499)
    ERROR_STORAGE_WRITE_FAILED = 400,
    ERROR_STORAGE_READ_FAILED,

    // New CAN Context Errors (500-599)
    ERROR_CAN_QUEUE_UNAVAILABLE = 500,
    ERROR_CAN_INVALID_CONTEXT,

	// Heap creation errors (600-699)
	ERROR_HEAP_CREATE_LOG_BUFFER,
	ERROR_HEAP_LOG_BUFFER_NULL,

} ErrorCodes;




/**
 * @brief Structure for defining error entries in the error table.
 */
typedef struct {
    ErrorCodes code;                  /**< Error code. */
    const char *message;             /**< Error message string. */
    ErrorCategory category;          /**< Category of the error. */
    ErrorSeverity severity;          /**< Severity of the error. */
    uint32_t count;               /**< How many times has this happened? */
} ErrorEntry;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Handles an error, logs it, and optionally invokes the system error handler.
 */
void log_error_to_storage(ErrorCodes error_code, const char *context);
void user_error_handler(ErrorCodes error_code, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
