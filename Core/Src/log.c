/*
 * log.c
 *
 * Created on: Nov 22, 2024
 * Author: Ryan
 *
 * Description:
 * This file provides a logging system for the project, supporting message buffering,
 * formatted output, and multi-threaded operation with mutex protection.
 */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "log.h"
#include "ansi.h"

/* --------------------------------------------------------------------------
   Global Variables
   -------------------------------------------------------------------------- */

// Circular buffer for storing log messages
LogBuffer log_buffer = { .head = 0, .tail = 0, .count = 0 };

/* --------------------------------------------------------------------------
   Initialization Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Initialize the logging system.
 *
 * This function creates a mutex to protect the log buffer and ensures thread-safe
 * access for logging operations.
 */
void init_log_system(void) {
    log_buffer.mutex_id = osMutexNew(NULL); // Create the mutex

    // Log the initialization status using the helper function
    log_status_message("Initializing logging system", log_buffer.mutex_id != NULL);

    if (log_buffer.mutex_id == NULL) {
        user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to initialize log mutex");
    }
}

/* --------------------------------------------------------------------------
   Logging Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Log a generic message to the buffer.
 *
 * Supports formatted strings, similar to `printf`. Messages are stored in
 * the circular log buffer and can be processed by the logging task.
 *
 * @param format Format string for the log message.
 * @param ... Variable arguments for the format string.
 */
void log_message(const char *format, ...) {
    // Acquire the mutex to protect the buffer
    if (osMutexAcquire(log_buffer.mutex_id, osWaitForever) != osOK) {
        return; // Skip logging if mutex acquisition fails
    }

    // Check if the buffer is full
    if (log_buffer.count >= LOG_BUFFER_SIZE) {
        osMutexRelease(log_buffer.mutex_id); // Release the mutex
        return; // Buffer full, drop the message
    }

    // Format the log message
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer.messages[log_buffer.head], LOG_MESSAGE_MAX_LENGTH, format, args);
    va_end(args);

    // Update the buffer's state
    log_buffer.head = (log_buffer.head + 1) % LOG_BUFFER_SIZE;
    log_buffer.count++;

    // Release the mutex
    osMutexRelease(log_buffer.mutex_id);
}

/**
 * @brief Log a status message with a "SUCCESS" or "FAILED" suffix.
 *
 * Adds either "SUCCESS" (in green) or "FAILED" (in red) to the message and
 * logs it using `log_message`.
 *
 * @param message The base message to log (e.g., "Initializing CAN").
 * @param is_success Indicates success (true) or failure (false).
 */
void log_status_message(const char *message, bool is_success) {
    if (message == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "NULL message passed to log_status_message");
        return;
    }

    char buf[LOG_MESSAGE_MAX_LENGTH];  // Use LOG_MESSAGE_MAX_LENGTH for the buffer size

    // Use snprintf to prevent buffer overflows
    int len = snprintf(buf, sizeof(buf), "%s...%s",
                       message,
                       is_success ? BGRN "SUCCESS" CRESET : BRED "FAILED" CRESET);

    if (len < 0 || len >= (int)sizeof(buf)) {
        user_error_handler(ERROR_UI_LOGGING_FAILED, "Message construction failed in log_status_message");
        return;
    }

    // Log the formatted message
    log_message(buf);
}


/**
 * @brief Log a transmitted CAN message.
 *
 * Formats and logs a CAN message, including the request ID, payload data,
 * and other relevant information for debugging.
 *
 * @param request_id The CAN request ID.
 * @param TxData Pointer to the payload data.
 * @param dlc Data length code.
 */
void log_transmitted_can_message(uint64_t request_id, uint8_t *TxData, uint8_t dlc) {
    char buf[100 + dlc * 2]; // Allocate sufficient space for the message
    int offset = snprintf(buf, sizeof(buf),
                          HGRN "* " BBLU "Tx " UBLU "%X" BWHT "/",
                          (unsigned int)request_id);

    // Append each data byte as a two-character hexadecimal value
    for (uint8_t j = 0; j < dlc; j++) {
        offset += snprintf(&buf[offset], sizeof(buf) - offset, GRN "%02X", TxData[j]);
    }

    // Reset color to prevent bleed
    offset += snprintf(&buf[offset], sizeof(buf) - offset, CRESET);

    // Send the formatted message to the console
    log_message(buf);
}

/**
 * @brief Logs raw CAN packet data for debugging.
 *
 * @param packet Pointer to the CAN packet.
 */
void log_raw_can_packet(const CAN_Packet *packet) {
    if (!packet) return;
    char buf[128];
    snprintf(buf, sizeof(buf), "Raw CAN: ID=0x%" PRIX32 ", DLC=%d, Data=",
             packet->header.id, packet->header.dlc);

    for (uint8_t i = 0; i < packet->header.dlc; i++) {
        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "%02X ", packet->payload[i]);
    }

    log_message(buf);
}

/**
 * @brief Logs validated CAN data.
 *
 * @param parsed_data Pointer to the parsed data structure.
 */
void log_valid_can_data(const Parsed_CAN_Data *parsed_data) {
    char buf[128];
    snprintf(buf, sizeof(buf), "Valid CAN Data: PID=0x%04" PRIX16 ", Payload=0x%08" PRIX32,
             parsed_data->pid, parsed_data->payload);
    log_message(buf);
}

/* --------------------------------------------------------------------------
   Logging Task
   -------------------------------------------------------------------------- */

/**
 * @brief Task to process and output buffered log messages.
 *
 * This task runs in a loop, retrieving messages from the circular log buffer
 * and sending them to the UART for output. The buffer is protected by a mutex.
 */
void __rtos__log_task(void) {
    char batch_buffer[LOG_BUFFER_SIZE * LOG_MESSAGE_MAX_LENGTH];
    size_t batch_length = 0;

    for (;;) {
        batch_length = 0;

        // Acquire the mutex to access the buffer
        if (osMutexAcquire(log_buffer.mutex_id, osWaitForever) == osOK) {
            while (log_buffer.count > 0 && batch_length < sizeof(batch_buffer)) {
                // Append the oldest message to the batch
                size_t len = strlen(log_buffer.messages[log_buffer.tail]);
                memcpy(&batch_buffer[batch_length], log_buffer.messages[log_buffer.tail], len);
                batch_length += len;

                // Add a newline for readability
                batch_buffer[batch_length++] = '\n';
                batch_buffer[batch_length++] = '\r';

                // Update the buffer's state
                log_buffer.tail = (log_buffer.tail + 1) % LOG_BUFFER_SIZE;
                log_buffer.count--;
            }

            osMutexRelease(log_buffer.mutex_id);
        }

        // Transmit the batched messages over UART
        if (batch_length > 0) {
            HAL_UART_Transmit(&huart2, (uint8_t *)batch_buffer, batch_length, HAL_MAX_DELAY);
        }

        // Delay to avoid hogging the CPU
        osDelay(10);
    }
}

/* --------------------------------------------------------------------------
   Utility Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Display a welcome message on system initialization.
 *
 * Outputs a styled welcome message to the console, including system status.
 */
void display_welcome_message(void) {
    log_message(CLR); // Clear screen and reset cursor

    // Header
    log_message(BBLU "====================================================================");
    log_message(BBLU "|" BYEL "                 FORD SUPERDUTY TRUCK CAN BUS GATEWAY               " BBLU "|");
    log_message(BBLU "====================================================================" CRESET);

    // Spacer
    log_message("");

    // Status Messages
    log_message(BGRN "* " CRESET GRN "STM32 initialized" CRESET);
    log_message(BGRN "* " CRESET GRN "Logging system initialized" CRESET);

    // Footer
    log_message("");
    log_message(BBLU "====================================================================" CRESET);
}
