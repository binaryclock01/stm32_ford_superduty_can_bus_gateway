/*
 * buffers.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares functions and data structures for managing circular
 * buffers and memory allocation for CAN packets, as well as handling Rx ISR buffers.
 */

#ifndef BUFFERS_H_
#define BUFFERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "config.h"
#include "can_common.h"          // Common CAN utilities
#include "rtos.h"                // RTOS utilities
#include "can_rx.h"              // For Rx processing functions
#include "log.h"                 // Logging utilities


/* --------------------------------------------------------------------------
   Typedefs
   -------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------
   Global Variables
   -------------------------------------------------------------------------- */

extern CAN_Circular_Buffer can_circular_buffer[TOTAL_QUEUES];
extern volatile CAN_Rx_Packet g_isr_rx_buffer[ISR_BUFFER_SIZE];
extern volatile size_t g_isr_rx_buffer_write_index;
extern volatile size_t g_isr_rx_buffer_read_index;

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Initializes all circular buffers for CAN queues.
 *
 * Sets up mutexes and clears all circular buffers.
 */
void init_circular_buffers(void);

/**
 * @brief Allocates a CAN packet from the circular buffer.
 *
 * Thread-safe function that allocates a CAN packet from the specified buffer.
 *
 * @param queue_type The queue type (e.g., QUEUE_RX_CAN1, QUEUE_TX_CAN1).
 * @return CAN_Packet* Pointer to an allocated CAN packet, or NULL if the buffer is full.
 */
CAN_Packet *_allocate_can_packet_on_circular_buffer(Circular_Queue_Types queue_type);

/**
 * @brief Frees a CAN packet using the specified circular buffer.
 *
 * Thread-safe function that marks a packet as free in the specified buffer.
 *
 * @param queue_buffer Pointer to the `CAN_Circular_Buffer` to use.
 * @return true if successful, false otherwise.
 */
bool _free_can_packet_using_queue_buffer_from_circular_buffer(CAN_Circular_Buffer *queue_buffer);

/**
 * @brief Frees a CAN packet using the specified queue type.
 *
 * Frees a CAN packet from the circular buffer associated with the given queue type.
 *
 * @param queue_type The queue type (e.g., QUEUE_RX_CAN1, QUEUE_TX_CAN1).
 * @return true if successful, false otherwise.
 */
bool _free_can_packet_using_queue_type_from_circular_buffer(Circular_Queue_Types queue_type);

/**
 * @brief Retrieves the message queue handle for a given CAN hardware instance.
 *
 * Maps a CAN hardware handle (e.g., `hcan1`) to its corresponding RTOS message queue handle.
 *
 * @param hcan Pointer to the CAN hardware handle.
 * @return osMessageQueueId_t* Pointer to the message queue handle, or NULL if not found.
 */
osMessageQueueId_t *get_rx_queue_handle_from_hcan(CAN_HandleTypeDef *hcan);

/**
 * @brief Processes the Rx ISR buffer for a given CAN instance.
 *
 * Allocates a CAN packet, retrieves the message from FIFO0, and enqueues it into
 * the corresponding message queue.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., `hcan1` or `hcan2`).
 */
void can_process_rx_isr_buffer(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* BUFFERS_H_ */
