/**
 * @file rtos.c
 * @brief RTOS-specific utilities for managing CAN communication.
 *
 * This file contains implementations for handling RTOS constructs
 * such as mutexes, message queues, and tasks. These utilities are
 * designed to manage CAN packet pools, safely enqueue/dequeue packets,
 * and support multi-threaded communication.
 */

#include "rtos.h"
#include "main.h"  // For CAN handles and HAL functions
#include "error.h" // For error handling
#include <string.h> // For memcpy
#include <stdio.h>

// Mutex for protecting the CAN packet pool


// CAN message queues
osMessageQueueId_t CAN1_Rx_QueueHandle;
osMessageQueueId_t CAN2_Rx_QueueHandle;
osMessageQueueId_t Tx_QueueHandle;

// Circular buffer for all queues and CANs

CAN_Circular_Buffer can_circular_buffer[TOTAL_QUEUES];

/**
 * @brief Initialize all circular buffers with mutexes.
 */
void init_circular_buffers(void) {
    for (int i = 0; i < TOTAL_QUEUES; i++) {
        can_circular_buffer[i] = (CAN_Circular_Buffer){0}; // Clear the buffer
        can_circular_buffer[i].mutex_id = osMutexNew(NULL); // Create a mutex for the buffer
        if (can_circular_buffer[i].mutex_id == NULL) {
            user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to initialize mutex for circular buffer %d", i);
        }
    }
}

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
 * @param queue_type The queue type (e.g., QUEUE_RX_CAN1, QUEUE_RX_CAN2, QUEUE_TX).
 * @return CAN_Packet* Pointer to an allocated CAN packet, or NULL if the buffer is full.
 */
CAN_Packet *_allocate_can_packet_on_circular_buffer(Circular_Queue_Types queue_type) {
    // Validate the queue type
    if (queue_type >= TOTAL_QUEUES) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Invalid queue type provided to _allocate_can_packet");
        return NULL;
    }

    // Access the corresponding circular buffer
    CAN_Circular_Buffer *buffer = &can_circular_buffer[queue_type];

    // Acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in allocate");
        return NULL;
    }

    // Check if the buffer is full
    if (buffer->count >= CAN_BUFFER_SIZE) {
        if (osMutexRelease(buffer->mutex_id) != osOK) {
            user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after full buffer check in allocate");
        }
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "No free packets available in the circular buffer");
        return NULL;
    }

    // Allocate the next available packet
    CAN_Packet *packet = &buffer->packets[buffer->head];

    // Update the head index and increment the count
    buffer->head = (buffer->head + 1) % CAN_BUFFER_SIZE;
    buffer->count++;

    // Release the mutex after modifying the buffer
    if (osMutexRelease(buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after allocation");
    }

    return packet; // Return the allocated packet
}


/**
 * @brief Processes a CAN Rx FIFO0 message callback.
 *
 * This function is triggered when a CAN message is received in FIFO0.
 * It identifies the associated CAN instance, allocates a packet from the memory pool,
 * retrieves the CAN message, and enqueues it into the appropriate message queue.
 * The responsibility for processing and freeing the packet lies with the task
 * consuming the queue (e.g., StartCAN1_Rx_Task or StartCAN2_Rx_Task).
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 */
void process_can_rx_fifo_callback(CAN_HandleTypeDef *hcan) {
    // Step 1: Determine the CAN instance based on the provided CAN hardware handle
    CANInstance can_instance = get_can_instance_from_hcan(hcan);
    if (can_instance == CAN_TOTAL) {
        // Log and exit if the CAN instance is invalid
        user_error_handler(ERROR_CAN_MODULE_NOT_FOUND, "Unknown CAN instance in Rx FIFO callback");
        return;
    }

    // Step 2: Retrieve the message queue for the corresponding CAN instance
    osMessageQueueId_t *target_queue = get_queue_handle_from_hcan(hcan);
    if (target_queue == NULL) {
        // Log and exit if the target queue is unavailable
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Message queue unavailable for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_INIT_FAILED, error_msg);
        return;
    }

    Circular_Queue_Types queue_type = CAN_TRUCK;

    if (can_instance == CAN_AUX)
    	queue_type = QUEUE_RX_CAN2;

    // Step 3: Allocate a CAN packet from the memory pool
    CAN_Packet *packet = _allocate_can_packet_on_circular_buffer(queue_type);
    if (packet == NULL) {
        // Log and exit if no free packet is available in the memory pool
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "No free packet in pool for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, error_msg);
        return;
    }

    // Step 4: Retrieve the CAN message and populate the allocated packet
    if (!get_rx_message_from_CAN_RX_FIFO0(hcan, can_instance, packet)) {
        // Free the packet and log the error if message retrieval fails
        _free_can_packet_from_circular_buffer(queue_type, packet);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Failed to retrieve CAN message for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, error_msg);
        return;
    }

    // Step 5: Enqueue the packet into the message queue
    if (osMessageQueuePut(target_queue, &packet, 0, 0) != osOK) {
        // Free the packet and log the error if the queue is full or unavailable
        _free_can_packet_from_circular_buffer(queue_type, packet);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Queue full for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_QUEUE_FULL, error_msg);
        return;
    }

    // Note: Do NOT free the packet here!
    // The osMessageQueuePut function only copies the pointer to the packet,
    // not the packet data itself. The packet must remain valid for the
    // consumer task (e.g., StartCAN1_Rx_Task) to process and free.
}

/**
 * @brief Retrieves the message queue handle for a given CAN hardware instance.
 *
 * Maps a hardware CAN handle (e.g., `hcan1`) to the corresponding RTOS
 * message queue handle for receiving messages.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., `hcan1` or `hcan2`).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t *get_queue_handle_from_hcan(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        return &CAN1_Rx_QueueHandle;
    } else if (hcan == &hcan2) {
        return &CAN2_Rx_QueueHandle;
    } else {
        return NULL; // Unknown CAN instance
    }
}


/**
 * @brief Enqueue a CAN packet into the specified circular buffer with thread safety.
 *
 * @param queue_id The queue ID (e.g., `QUEUE_RX_CAN1`, `QUEUE_RX_CAN2`).
 * @param packet Pointer to the `CAN_Packet` to enqueue.
 * @return true if the packet was successfully enqueued, false if the buffer is full or a mutex error occurred.
 */
bool _enqueue_can_rx_packet(int queue_id, const CAN_Packet *packet) {
    if (queue_id != QUEUE_RX_CAN1 && queue_id != QUEUE_RX_CAN2) {
        user_error_handler(ERROR_CAN_INVALID_CONTEXT, "Invalid rx queue ID for enqueue");
        return false;
    }

    CAN_Circular_Buffer *buffer = &can_circular_buffer[queue_id];

    // Acquire the mutex
    if (osMutexAcquire(buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire mutex for enqueue in buffer %d", queue_id);
        return false;
    }

    // Check if the buffer is full
    if (buffer->count >= CAN_BUFFER_SIZE) {
        osMutexRelease(buffer->mutex_id); // Release the mutex
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "Circular buffer is full for queue ID %d", queue_id);
        return false;
    }

    // Add the packet to the buffer
    buffer->packets[buffer->head] = *packet;
    buffer->head = (buffer->head + 1) % CAN_BUFFER_SIZE;
    buffer->count++;

    // Release the mutex
    if (osMutexRelease(buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after enqueue in buffer %d", queue_id);
        return false;
    }

    return true;
}

/**
 * @brief Enqueue a CAN data packet for transmission using the `CAN_Packet` structure.
 *
 * This function prepares a `CAN_Packet` with the provided header, metadata, and payload,
 * stores it in the circular buffer, and enqueues it in the Tx message queue for
 * asynchronous transmission by the Tx task.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) to use for transmission.
 * @param txheader Pointer to the `CAN_TxHeaderTypeDef` containing header information.
 * @param payload Pointer to the payload data to be transmitted (8 bytes maximum).
 * @param timestamp Timestamp of the message, typically obtained via `HAL_GetTick()`.
 * @return true if the message was successfully enqueued, false otherwise.
 */
/*
bool _enqueue_can_tx_packet(CANInstance can_instance, CAN_TxHeaderTypeDef *txheader, const uint8_t *payload, uint32_t timestamp) {
    // Validate inputs
    if (can_instance >= CAN_TOTAL || txheader == NULL || payload == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Invalid arguments provided to _enqueue_can_tx_packet");
        return false;
    }

    // Acquire the mutex for the Tx circular buffer
    CAN_Circular_Buffer *tx_buffer = &can_circular_buffer[QUEUE_TX];
    if (osMutexAcquire(tx_buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire mutex for Tx circular buffer");
        return false;
    }

    // Check if the buffer is full
    if (tx_buffer->count >= CAN_BUFFER_SIZE) {
        osMutexRelease(tx_buffer->mutex_id); // Release the mutex
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "Tx circular buffer is full");
        return false;
    }

    // Prepare the CAN packet and store it in the buffer
    CAN_Packet *packet = &tx_buffer->packets[tx_buffer->head];
    memset(packet, 0, sizeof(CAN_Packet)); // Clear the packet

    // Populate the packet fields
    packet->flow = PACKET_TX;                       // Mark as a transmission packet
    packet->meta.can_instance = can_instance;       // Set CAN instance
    packet->meta.timestamp = timestamp;            // Set the timestamp
    packet->header.id = txheader->StdId;            // Set the CAN ID
    packet->header.dlc = txheader->DLC;             // Set the data length code
    packet->header.is_extended_id = txheader->IDE;  // Set if the ID is extended
    packet->header.is_remote_frame = txheader->RTR; // Set if the frame is a remote frame
    memcpy(packet->payload, payload, sizeof(packet->payload)); // Copy the payload

    // Update the circular buffer state
    tx_buffer->head = (tx_buffer->head + 1) % CAN_BUFFER_SIZE;
    tx_buffer->count++;

    // Release the mutex for the circular buffer
    if (osMutexRelease(tx_buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after enqueuing Tx packet");
        return false;
    }

    // Enqueue the packet in the Tx message queue
    if (osMessageQueuePut(Tx_QueueHandle, packet, 0, 0) != osOK) {
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, "Failed to enqueue CAN packet in Tx message queue");
        return false;
    }

    return true;
}
*/

/**
 * @brief Dequeues a CAN packet from the circular buffer for a specified queue.
 *
 * Retrieves the next available CAN packet from the specified circular buffer.
 * This function ensures thread-safe access by using a mutex to protect the buffer.
 *
 * @param queue_type The queue type (e.g., QUEUE_RX_CAN1, QUEUE_RX_CAN2, QUEUE_TX).
 * @param packet Pointer to a `CAN_Packet` structure where the dequeued packet will be stored.
 * @return true if a packet was successfully dequeued, false if the buffer is empty.
 */
bool _free_can_packet_from_circular_buffer(uint8_t queue_type, CAN_Packet *packet) {
    // Validate input arguments
    if (queue_type >= TOTAL_QUEUES) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Invalid queue type provided to _dequeue_can_packet");
        return false;
    }

    if (packet == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Null pointer provided to _dequeue_can_packet");
        return false;
    }

    // Access the corresponding circular buffer
    CAN_Circular_Buffer *buffer = &can_circular_buffer[queue_type];

    // Attempt to acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in dequeue");
        return false;
    }

    // Check if the buffer is empty
    if (buffer->count == 0) {
        if (osMutexRelease(buffer->mutex_id) != osOK) {
            user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after empty buffer check in dequeue");
        }
        return false; // Buffer is empty
    }

    // Retrieve the next packet
    *packet = buffer->packets[buffer->tail];

    // Update the tail index and decrement the count
    buffer->tail = (buffer->tail + 1) % CAN_BUFFER_SIZE;
    buffer->count--;

    // Release the mutex after modifying the buffer
    if (osMutexRelease(buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after dequeuing");
    }

    return true; // Successfully dequeued a packet
}


