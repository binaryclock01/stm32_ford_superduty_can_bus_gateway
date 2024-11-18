/**
 * @file rtos.h
 * @brief RTOS-specific utilities for managing CAN communication.
 *
 * This header defines functions and data structures used to handle
 * RTOS constructs such as mutexes, message queues, and tasks for
 * CAN communication. These utilities include packet pool management,
 * message queuing, and safe access to shared resources.
 */

#ifndef RTOS_H
#define RTOS_H

#include <can_core.h>      // For CAN-related structures
#include "cmsis_os.h" // RTOS CMSIS types and functions


#define CAN_PACKET_POOL_SIZE 10 ///< Unified size for CAN packet pool and circular buffers
#define CAN_BUFFER_SIZE CAN_PACKET_POOL_SIZE ///< Circular buffer size matches packet pool size

#define RETRY_DELAY_MS 100   // Delay between retries in milliseconds

typedef struct {
    CAN_Packet packets[CAN_BUFFER_SIZE]; /**< Circular buffer of TX packets. */
    uint8_t head;                          /**< Index of the next write position. */
    uint8_t tail;                          /**< Index of the next read position. */
    uint8_t count;                         /**< Number of packets currently in the buffer. */
} CAN_Circular_Buffer;

/**
 * @brief Mutex for protecting shared resources in the CAN packet pool.
 */
extern osMutexId_t packet_pool_mutex;

/**
 * @brief Message queue handle for CAN1 receive messages.
 */
extern osMessageQueueId_t CAN1_Rx_QueueHandle;

/**
 * @brief Message queue handle for CAN2 receive messages.
 */
extern osMessageQueueId_t CAN2_Rx_QueueHandle;

/**
 * @brief Message queue handle for CAN transmission messages.
 */
extern osMessageQueueId_t Tx_QueueHandle;

/**
 * @brief Processes a CAN Rx FIFO0 message callback.
 *
 * This function is used to decouple the interrupt callback logic and handle
 * CAN messages received in FIFO0.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 */
void process_can_rx_fifo_callback(CAN_HandleTypeDef *hcan);

/**
 * @brief Initializes the CAN packet pool and its associated mutex.
 *
 * This function must be called during system initialization to ensure
 * that the packet pool is ready for use.
 */
void init_can_packet_pool(void);

/**
 * @brief Retrieves the message queue handle for a given CAN hardware instance.
 *
 * This function maps a hardware CAN handle (e.g., `hcan1`) to the corresponding
 * RTOS message queue handle used for receiving messages.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., `hcan1` or `hcan2`).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t *get_queue_handle_from_hcan(CAN_HandleTypeDef *hcan);

/**
 * @brief Enqueues a CAN packet into the buffer for the specified CAN instance.
 *
 * Safely adds a CAN packet to the circular buffer associated with the given
 * CAN instance. Ensures no data is lost unless the buffer is full.
 *
 * @param can_instance The CAN instance (e.g., `CAN_TRUCK` or `CAN_AUX`).
 * @param packet Pointer to the `CAN_Packet` to enqueue.
 * @return true if the packet was successfully enqueued, false if the buffer is full.
 */
bool _enqueue_can_packet(CANInstance can_instance, const CAN_Packet *packet);

/**
 * @brief Allocates a CAN packet from the circular buffer.
 *
 * This function acquires a mutex to ensure thread-safe access to the shared
 * circular buffer. It iterates through the buffer to find an unused packet,
 * marks it as used, and returns a pointer to the caller. If the buffer is full,
 * the function releases the mutex and returns NULL.
 *
 * Error handling ensures that mutex acquisition and release are verified,
 * and any failures are logged with additional context.
 *
 * @return CAN_Packet* Pointer to an allocated CAN packet, or NULL if the buffer is full.
 */
CAN_Packet *_allocate_can_packet(void);

/**
 * @brief Dequeues a CAN packet from the buffer for the specified CAN instance.
 *
 * Safely retrieves the next available CAN packet from the circular buffer
 * associated with the given CAN instance.
 *
 * @param can_instance The CAN instance (e.g., `CAN_TRUCK` or `CAN_AUX`).
 * @param packet Pointer to a `CAN_Packet` structure where the dequeued packet will be stored.
 * @return true if a packet was successfully dequeued, false if the buffer is empty.
 */
bool _dequeue_can_packet(CAN_Packet *packet);

#endif // RTOS_H
