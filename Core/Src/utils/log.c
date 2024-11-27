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
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#include "config.h"
#include "main.h"
#include "buffers.h"
#include "log.h"
#include "ansi.h"
#include "error.h"

/* --------------------------------------------------------------------------
   Global Variables
   -------------------------------------------------------------------------- */

// Global buffer for log buffer is in buffers.c

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
	// log_buffer_global_ptr is a global pointer that is populated with the address of the
	// log buffer that is allocated in the heap from create_log_buffer__heap__

    bool log_buffer_create_success = create_log_buffer__heap__();

    LogBuffer *log_buffer_global_ptr = get_log_buffer_global_ptr();

	if (!log_buffer_create_success || (log_buffer_global_ptr == NULL)) {
		user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to initialize log mutex");
		return;
	}

	log_buffer_global_ptr->mutex_id = osMutexNew(NULL); // Create the mutex

    // Log the initialization status using the helper function
    log_status_message("Initializing logging system", log_buffer_global_ptr->mutex_id != NULL);
    if (log_buffer_global_ptr->mutex_id == NULL) {
        user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to initialize log mutex");
    }
    report_log_buffer_heap_size();
}

// Use pvPortMalloc to create this buffer in the heap instead of static
bool create_log_buffer__heap__(void) {
	// call the function in buffers.c
	return __buffers__create_log_buffer__heap__();
}

void destroy_log_buffer__heap__() {
	// call the function in buffers.c
	__buffers_destroy_log_buffer__heap__();
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
    LogBuffer *log_buffer = get_log_buffer_global_ptr();
    if (log_buffer == NULL) {
        user_error_handler(ERROR_HEAP_LOG_BUFFER_NULL, "Error thrown from: __func__");
        return;
    }

    char buf[LOG_MESSAGE_MAX_LENGTH];
    char final_buf[LOG_MESSAGE_MAX_LENGTH];

    // Calculate the space available in `final_buf` after accounting for ANSI codes
    size_t ansi_overhead = strlen(YEL) + strlen(CRESET) + strlen("SIM: ");
    size_t max_buf_length = LOG_MESSAGE_MAX_LENGTH - ansi_overhead - 1; // -1 for null terminator

    // Prepare the log message
    va_list args;
    va_start(args, format);

#ifdef USE_ANSI
    int buf_len = vsnprintf(buf, max_buf_length, format, args); // Use only the calculated space
#else
    // Strip ANSI codes when USE_ANSI is not defined
    char stripped_buf[LOG_MESSAGE_MAX_LENGTH];
    int buf_len = vsnprintf(stripped_buf, max_buf_length, format, args);
    strip_ansi_codes(buf, stripped_buf, sizeof(buf)); // Function to remove ANSI codes
#endif

    va_end(args);

    // Check for truncation in `buf`
    if (buf_len >= max_buf_length) {
        snprintf(buf + max_buf_length - 20, 20, "...[TRUNCATED]"); // Append a truncation marker
        buf[max_buf_length - 1] = '\0'; // Ensure null-termination
    }

#ifdef IS_SIMULATOR
    int final_len = snprintf(final_buf, sizeof(final_buf), "%sSIM: %s%s", YEL, CRESET, buf);
#else
    int final_len = snprintf(final_buf, sizeof(final_buf), "%s", buf);
#endif

    // Check for truncation in `final_buf`
    if (final_len >= LOG_MESSAGE_MAX_LENGTH) {
        snprintf(final_buf + LOG_MESSAGE_MAX_LENGTH - 20, 20, "...[TRUNCATED]"); // Append a truncation marker
        final_buf[LOG_MESSAGE_MAX_LENGTH - 1] = '\0'; // Ensure null-termination
    }

    // Copy the message into the circular buffer
    strncpy(log_buffer->messages[log_buffer->head], final_buf, LOG_MESSAGE_MAX_LENGTH - 1);
    log_buffer->messages[log_buffer->head][LOG_MESSAGE_MAX_LENGTH - 1] = '\0'; // Null-terminate

    // Update the buffer state
    log_buffer->head = (log_buffer->head + 1) % LOG_BUFFER_SIZE;
    log_buffer->count++;
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
    const int total_width = 60; // Total width for alignment
    const char *suffix = is_success ? BGRN "SUCCESS" CRESET : BRED "FAILED" CRESET;

    // Strip ANSI codes for length calculation
    char stripped_message[LOG_MESSAGE_MAX_LENGTH];
    strip_ansi_codes(stripped_message, message, sizeof(stripped_message));

    size_t message_len = strlen(stripped_message);

    // Strip ANSI codes from suffix
    char stripped_suffix[LOG_MESSAGE_MAX_LENGTH];
    strip_ansi_codes(stripped_suffix, suffix, sizeof(stripped_suffix));
    size_t suffix_len = strlen(stripped_suffix);

    size_t dots_count = total_width - message_len - suffix_len;

    if (dots_count > LOG_MESSAGE_MAX_LENGTH - message_len - suffix_len - 1) {
        dots_count = 0; // Safety to avoid buffer overflow
    }

    // Prepare the formatted message
    char buf[LOG_MESSAGE_MAX_LENGTH];
    snprintf(buf, sizeof(buf), "%s%.*s%s",
             message,
             (int)dots_count,
             "............................................................", // Pre-fill with enough dots
             suffix);

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

void log_transmitted_can_message(Circular_Queue_Types queue_num, uint32_t request_id, uint8_t *TxData, uint8_t dlc) {
    if (queue_num < 0 || queue_num >= TOTAL_QUEUES) {
        log_message("Error: Invalid queue number.");
        return;
    }

    char buf[500]; // Allocate sufficient space for the message
    int pos = snprintf(buf, sizeof(buf),
                       CRESET "* " BBLU "Generated packet SENT to " BWHT "%s" BLU " | " BBLU "Data: " CRESET "%X" BGRN "/",
                       Circular_Queue_Types_Names[queue_num],
                       (unsigned int)request_id);

    if (pos < 0 || pos >= sizeof(buf)) {
        log_message("Error: Buffer overflow detected while formatting CAN message.");
        return;
    }

    // Append data bytes with custom colors
    for (uint8_t j = 0; j < dlc; j++) {
        const char *color;

        // Assign color based on index

        if (j == 1) // highlight the instruction byte in cyan
        	color = CYN;
        else if (j == 2 || j == 3) { // highlight the PID in magenta
            color = MAG; // Magenta for TxData[1] and TxData[2]
        } else {
            color = (j % 2 == 0) ? WHT : BWHT; // Alternate between white and bright white
        }

        // Append the colored data byte
        int bytes_written = snprintf(&buf[pos], sizeof(buf) - pos, "%s%02X", color, TxData[j]);
        if (bytes_written < 0 || pos + bytes_written >= sizeof(buf)) {
            log_message("Error: Buffer overflow detected while appending data bytes.");
            return;
        }
        pos += bytes_written;
    }

    // Null-terminate explicitly
    buf[pos] = '\0';

    // Send the formatted message to the console
    log_message(buf);
}

void log_queue_packet_counts(void) {
    char buf[500];
    int pos = snprintf(buf, sizeof(buf), "* QUEUES: ");
    for (int i = 0; i < TOTAL_QUEUES; i++) {
        pos += snprintf(&buf[pos], sizeof(buf) - pos, CRESET "%s:" BWHT "%d ", Circular_Queue_Types_Names[i], can_circular_buffer[i].count);
        if (pos >= sizeof(buf)) break; // Prevent buffer overflow
    }
    buf[sizeof(buf) - 1] = '\0'; // Ensure null termination
    log_message(buf);
}

void log_circular_buffer_usage(void) {
    char buf[500];
    int pos = snprintf(buf, sizeof(buf), "* BUFFER: ");
    for (int i = 0; i < TOTAL_QUEUES; i++) {
        pos += snprintf(&buf[pos], sizeof(buf) - pos, CRESET "%s:" BWHT "%d ", Circular_Queue_Types_Names[i], can_circular_buffer[i].count);
        if (pos >= sizeof(buf)) break; // Prevent buffer overflow
    }
    buf[sizeof(buf) - 1] = '\0'; // Ensure null termination
    log_message(buf);
}

/**
 * @brief Logs a raw CAN packet in a human-readable format.
 *
 * This function formats the metadata, CAN ID, DLC, and payload of a CAN packet
 * into a single log message for debugging purposes.
 *
 * @param packet Pointer to the `CAN_Packet` containing the received message.
 */
void log_raw_can_packet(const CAN_Packet *packet) {
    // Step 1: Validate the input packet
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Null CAN_Packet provided to log_raw_can_packet.");
        return;
    }

    // Step 2: Extract header information
    uint32_t can_id = packet->header.id;
    uint8_t dlc = packet->header.dlc;

    // Step 3: Validate the DLC to ensure it's within the allowable range
    if (dlc > DLC_MAX) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Invalid DLC=%d in CAN packet", dlc);
        user_error_handler(ERROR_CAN_INVALID_PAYLOAD, error_msg);
        return;
    }

    // Step 4: Initialize the log message buffer
    char log_msg[128] = {0};

    // Step 5: Format the log message with CAN instance, ID, and DLC
    int result = snprintf(
        log_msg, sizeof(log_msg),
        "Raw CAN: Instance=%d, ID=0x%" PRIX32 ", DLC=%d, Data=",
        packet->meta.can_instance, can_id, dlc
    );

    // Step 6: Check for formatting errors
    if (result < 0 || result >= (int)sizeof(log_msg)) {
        user_error_handler(ERROR_UI_LOGGING_FAILED, "Log message construction failed in log_raw_can_packet.");
        return;
    }

    // Step 7: Append payload data to the log message
    size_t offset = strlen(log_msg);
    for (uint8_t i = 0; i < dlc; i++) {
        if (offset >= sizeof(log_msg)) {
            user_error_handler(ERROR_UI_LOGGING_FAILED, "Log message exceeded buffer size in log_raw_can_packet.");
            return;
        }
        int append_result = snprintf(&log_msg[offset], sizeof(log_msg) - offset, "%02X ", packet->payload[i]);
        if (append_result < 0) {
            user_error_handler(ERROR_UI_LOGGING_FAILED, "Failed to append payload data to log message.");
            return;
        }
        offset += append_result;
    }

    // Step 8: Log the complete message
    log_message(log_msg);
}

/**
 * @brief Log validated CAN data for debugging purposes.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the parsed CAN data structure.
 */
void log_valid_can_data(const Parsed_CAN_Data *parsed_data) {
    char log_msg[128] = {0};

    // Use PRIX32 for portable handling of uint32_t in hexadecimal
    snprintf(log_msg, sizeof(log_msg),
             "Valid CAN Data: PID=0x%04" PRIX16 " Payload=0x%08" PRIX32,
             parsed_data->pid, parsed_data->payload);

    // Log the message
    log_message(log_msg);
}


/* --------------------------------------------------------------------------
   Logging Task
   -------------------------------------------------------------------------- */



void _process_one_log_message()
{
	LogBuffer *log_buffer = get_log_buffer_global_ptr();
	if (log_buffer == NULL)
	{
		user_error_handler(ERROR_HEAP_LOG_BUFFER_NULL, "Error thrown from: __func__");
		return;
	}

    char batch_buffer[LOG_BUFFER_SIZE * LOG_MESSAGE_MAX_LENGTH];
    size_t batch_length = 0;

    // Acquire the mutex to access the buffer
    if (osMutexAcquire(log_buffer->mutex_id, osWaitForever) == osOK) {
        while (log_buffer->count > 0 && batch_length < sizeof(batch_buffer)) {
            // Append the oldest message to the batch
            size_t len = strlen(log_buffer->messages[log_buffer->tail]);
            memcpy(&batch_buffer[batch_length], log_buffer->messages[log_buffer->tail], len);
            batch_length += len;

            // Add a newline for readability
            batch_buffer[batch_length++] = '\n';
            batch_buffer[batch_length++] = '\r';

            // Update the buffer's state
            log_buffer->tail = (log_buffer->tail + 1) % LOG_BUFFER_SIZE;
            log_buffer->count--;
        }

        osMutexRelease(log_buffer->mutex_id);
    }

    // Transmit the batched messages over UART
    if (batch_length > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)batch_buffer, batch_length, HAL_MAX_DELAY);
    }

}

/**
 * @brief Task to process and output buffered log messages.
 *
 * This task runs in a loop, retrieving messages from the circular log buffer
 * and sending them to the UART for output. The buffer is protected by a mutex.
 */
void __rtos__log_task(void) {

    _process_one_log_message();
}

/**
 * @brief Flush all log messages from the log buffer.
 *
 * This function loops through the log buffer, processing and sending all log messages
 * until the buffer is empty. The buffer is protected by a mutex for thread-safe access.
 */
void flush_logs(void) {
	LogBuffer *log_buffer = get_log_buffer_global_ptr();
	if (log_buffer == NULL)
	{
		user_error_handler(ERROR_HEAP_LOG_BUFFER_NULL, "Error thrown from: __func__");
		return;
	}

    // Continue processing until the log buffer is empty
    while (1) {
        // Acquire the mutex to check and process the buffer
        if (osMutexAcquire(log_buffer->mutex_id, osWaitForever) == osOK) {
            // If the log buffer is empty, release the mutex and exit
            if (log_buffer->count == 0) {
                osMutexRelease(log_buffer->mutex_id);
                break; // Exit the loop once the buffer is empty
            }

            osMutexRelease(log_buffer->mutex_id);
        }

        // Process one batch of log messages
        _process_one_log_message();
    }
}

/* --------------------------------------------------------------------------
   Utility Functions
   -------------------------------------------------------------------------- */

void send_clear_screen_ansi_code(void) {
#ifdef USE_ANSI
    HAL_UART_Transmit(&huart2, (uint8_t *)CLR, strlen(CLR), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
#endif
}

/**
 * @brief Display a welcome message on system initialization.
 *
 * Outputs a styled welcome message to the console, including system status.
 */
void display_welcome_message(void) {
    // Header
    log_message("");

	log_message(BBLU "====================================================================");
    log_message(BBLU "|" BYEL "                 FORD SUPERDUTY TRUCK CAN BUS GATEWAY             " BBLU "|");
    log_message(BBLU "====================================================================" CRESET);

    // Spacer
    log_message("");

    // Status Messages
    log_status_message("* STM32 initialization", true);
}
