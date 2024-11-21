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

//#include <can_core.h>      // For CAN-related structures
#include "cmsis_os.h" // RTOS CMSIS types and functions
#include "can_common.h"


#define CAN_PACKET_POOL_SIZE 10 ///< Unified size for CAN packet pool and circular buffers
#define CAN_BUFFER_SIZE CAN_PACKET_POOL_SIZE ///< Circular buffer size matches packet pool size

#define RETRY_DELAY_MS 100   // Delay between retries in milliseconds

typedef enum {
	QUEUE_RX_CAN1,
	QUEUE_RX_CAN2,
	QUEUE_TX_CAN1,
	QUEUE_TX_CAN2,
	TOTAL_QUEUES,
	QUEUE_TYPE_UNKNOWN
} Circular_Queue_Types;

typedef enum {
	QUEUE_TYPE_FLOW_UNKNOWN,
	QUEUE_TYPE_FLOW_TX,
	QUEUE_TYPE_FLOW_RX,
} Queue_Type_Flow;

typedef struct {
	uint32_t total_packets;
} CAN_Circular_Buffer_Meta;

typedef struct {
    CAN_Packet packets[CAN_BUFFER_SIZE]; /**< Circular buffer of TX packets. */
    uint8_t head;                          /**< Index of the next write position. */
    uint8_t tail;                          /**< Index of the next read position. */
    uint8_t count;                        /**< Number of packets currently in the buffer. */
    osMutexId_t mutex_id;
    osMessageQueueId_t queue_handle;
    CAN_Circular_Buffer_Meta meta;
} CAN_Circular_Buffer;


/*
extern osMessageQueueId_t CAN1_Rx_QueueHandle;
extern osMessageQueueId_t CAN2_Rx_QueueHandle;
extern osMessageQueueId_t CAN1_Tx_QueueHandle;
extern osMessageQueueId_t CAN2_Tx_QueueHandle;
*/

extern CAN_Circular_Buffer can_circular_buffer[TOTAL_QUEUES];




/**
 * @brief Main StartCAN_Rx_Task that will call __rtos_StartCAN_Rx#_Task() depending on the can instance
 */
void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance);
void __rtos_StartCAN1_Rx_Task();
void __rtos_StartCAN2_Rx_Task();

/**
 * @brief Main StartCAN_Rx_Task that will call __rtos_StartCAN_Tx#_Task() depending on the can instance
 */
void __rtos__StartCAN_Tx_Task(CANInstance enum_can_instance);
void __rtos_StartCAN1_Tx_Task();
void __rtos_StartCAN2_Tx_Task();

bool __rtos_process_tx_queue_and_send_to_can(CANInstance enum_can_instance, CAN_Packet *packet);

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
void init_circular_buffers(void);

/**
 * @brief Retrieves the message queue handle for a given CAN hardware instance.
 *
 * This function maps a hardware CAN handle (e.g., `hcan1`) to the corresponding
 * RTOS message queue handle used for receiving messages.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., `hcan1` or `hcan2`).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t *get_rx_queue_handle_from_hcan(CAN_HandleTypeDef *hcan);

/**
 * @brief Enqueue a CAN packet into the specified circular buffer with thread safety.
 *
 * @param queue_id The queue ID (e.g., `QUEUE_RX_CAN1`, `QUEUE_RX_CAN2`)
 * @param packet Pointer to the `CAN_Packet` to enqueue.
 * @return true if the packet was successfully enqueued, false if the buffer is full or a mutex error occurred.
 */
//bool _enqueue_can_rx_packet(int queue_id, const CAN_Packet *packet);

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
CAN_Packet *_allocate_can_packet_on_circular_buffer(Circular_Queue_Types queue_type);

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
bool _free_can_packet_using_queue_buffer_from_circular_buffer(CAN_Circular_Buffer *queue_buffer);
bool _free_can_packet_using_queue_type_from_circular_buffer(Circular_Queue_Types queue_type);

#endif // RTOS_H
