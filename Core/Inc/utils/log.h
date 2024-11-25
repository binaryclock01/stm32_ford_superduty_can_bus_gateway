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

//#include "can_common.h"
#include <stdint.h>

#include "config.h"
#include "main.h"

#include "rtos.h"
#include "buffers.h"
#include "can_packet.h"
//#include "can_common.h"

struct CAN_Packet;
/* --------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */


// execute a function and flush the logs
// otherwise the logs would sit waiting for the housekeeping task to do it
// and maybe the task isn't running yet, like initial initialization of HAL
#define EXECUTE_FUNCTION_AND_FLUSH_LOGS(init_function) \
    do {                              \
        init_function();              \
        flush_logs();                 \
    } while (0)


/* --------------------------------------------------------------------------
   Data Structures
   -------------------------------------------------------------------------- */

/**
 * @brief Circular buffer structure for log messages.
 */
typedef struct {
    char (*messages)[LOG_MESSAGE_MAX_LENGTH]; /**< Pointer to log messages array */
    uint16_t head;        /**< Write position in the buffer */
    uint16_t tail;        /**< Read position in the buffer */
    uint16_t count;       /**< Number of messages in the buffer */
    osMutexId_t mutex_id; /**< Mutex for thread-safe access */
} LogBuffer;

/* --------------------------------------------------------------------------
   Extern Variables
   -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup --- */

void report_log_buffer_heap_size(void);
void report_log_buffer_fullness(void);

LogBuffer *get_log_buffer_global_ptr(void);
bool create_log_buffer__heap__(void);
void destroy_log_buffer__heap__(void);

void log_queue_packet_counts(void);
void log_circular_buffer_usage(void);
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

void flush_logs(void);

/**
 * @brief Log a transmitted CAN message for debugging.
 *
 * @param request_id The request ID.
 * @param TxData Pointer to the payload data.
 * @param dlc Data length code.
 */
// if you uncomment this, you have to try to include buffers.h.  I can't figure out why it gets all messed up. Just let it be implicit for now.
//void log_transmitted_can_message(Circular_Queue_Types queue_num, uint64_t request_id, uint8_t *TxData, uint8_t dlc);

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

void send_clear_screen_ansi_code(void);

#endif /* INC_LOG_H_ */
