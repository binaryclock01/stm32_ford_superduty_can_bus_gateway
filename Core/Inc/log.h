/*
 * log.h
 *
 * Created on: Nov 22, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares the logging system for managing and displaying messages.
 * Includes real-time logging with a circular buffer and helper functions for
 * status and debugging messages.
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include <stdint.h>
#include "main.h"

/* --------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define LOG_BUFFER_SIZE 256             /**< Maximum number of messages in the log buffer */
#define LOG_MESSAGE_MAX_LENGTH 128     /**< Maximum length of each log message */

/* --------------------------------------------------------------------------
   Data Structures
   -------------------------------------------------------------------------- */

/**
 * @brief Circular buffer structure for log messages.
 */
typedef struct {
    char messages[LOG_BUFFER_SIZE][LOG_MESSAGE_MAX_LENGTH]; /**< Log messages */
    uint16_t head;        /**< Write position in the buffer */
    uint16_t tail;        /**< Read position in the buffer */
    uint16_t count;       /**< Number of messages in the buffer */
    osMutexId_t mutex_id; /**< Mutex for thread-safe access */
} LogBuffer;

/* --------------------------------------------------------------------------
   Extern Variables
   -------------------------------------------------------------------------- */

extern LogBuffer log_buffer;

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup --- */

/**
 * @brief Initialize the logging system.
 */
void init_log_system(void);

/* --- Logging Functions --- */

/**
 * @brief Log a formatted message.
 *
 * @param format Format string for the log message.
 * @param ... Additional arguments for the format string.
 */
void log_message(const char *format, ...);

/**
 * @brief Logs a status message with a "SUCCESS" or "FAILED" suffix.
 *
 * @param message Base message to log.
 * @param is_success Boolean indicating success (true) or failure (false).
 */
void log_status_message(const char *message, bool is_success);

/**
 * @brief Logs raw CAN packet data for debugging.
 *
 * @param packet Pointer to the CAN packet.
 */
void log_raw_can_packet(const CAN_Packet *packet);

/**
 * @brief Logs validated CAN data.
 *
 * @param parsed_data Pointer to the parsed data structure.
 */
void log_valid_can_data(const Parsed_CAN_Data *parsed_data);

/**
 * @brief Log a transmitted CAN message for debugging.
 *
 * @param request_id The request ID.
 * @param TxData Pointer to the payload data.
 * @param dlc Data length code.
 */
void log_transmitted_can_message(uint64_t request_id, uint8_t *TxData, uint8_t dlc);

/* --- Task Functions --- */

/**
 * @brief RTOS task to process and display batched log messages.
 */
void __rtos__log_task(void);

/* --- Display Functions --- */

/**
 * @brief Display the welcome message on the console.
 */
void display_welcome_message(void);

#endif /* INC_LOG_H_ */
