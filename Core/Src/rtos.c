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
osMutexId_t packet_pool_mutex;

// CAN message queues
osMessageQueueId_t CAN1_Rx_QueueHandle;
osMessageQueueId_t CAN2_Rx_QueueHandle;
osMessageQueueId_t Tx_QueueHandle;

// Circular buffers for each CAN instance
static CAN_Circular_Buffer can_circular_buffer;

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

    // Step 3: Allocate a CAN packet from the memory pool
    CAN_Packet *packet = _allocate_can_packet();
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
        _free_can_packet(packet);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Failed to retrieve CAN message for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, error_msg);
        return;
    }

    // Step 5: Enqueue the packet into the message queue
    if (osMessageQueuePut(target_queue, &packet, 0, 0) != osOK) {
        // Free the packet and log the error if the queue is full or unavailable
        _free_can_packet(packet);
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
 * @brief Initializes the CAN Tx and Rx buffers and their associated mutexes.
 *
 * This function initializes the circular buffers for CAN transmission (Tx)
 * and reception (Rx) for each CAN instance. It also creates mutexes for
 * protecting the buffers during multi-threaded access. If any mutex
 * initialization fails, an error is logged with additional context.
 */
void init_can_buffers(void) {
    // Step 1: Initialize the Tx and Rx buffers for all CAN instances
	can_circular_buffer[i].head = 0;
	can_circular_buffer[i].tail = 0;
	can_circular_buffer[i].count = 0;

    // Step 2: Create the mutexes for protecting the Tx and Rx buffers
    tx_buffer_mutex = osMutexNew(NULL);
    if (tx_buffer_mutex == NULL) {
        // Log a critical error with additional context for Tx buffer mutex
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Tx buffer mutex initialization failed");
        return;
    }

    rx_buffer_mutex = osMutexNew(NULL);
    if (rx_buffer_mutex == NULL) {
        // Log a critical error with additional context for Rx buffer mutex
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Rx buffer mutex initialization failed");
        osMutexDelete(tx_buffer_mutex); // Clean up already-created Tx mutex
        return;
    }

    // Optional success log for debugging
    send_console_msg("CAN buffers and mutexes initialized successfully");
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
 * @return CAN_Packet* Pointer to an allocated CAN packet, or NULL if the buffer is full.
 */
CAN_Packet *_allocate_can_packet(void) {
    // Attempt to acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(packet_pool_mutex, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex");
        return NULL; // Return NULL if mutex acquisition fails
    }

    // Check if the buffer is full
    if (can_circular_buffer.count >= CAN_BUFFER_SIZE) {
        if (osMutexRelease(packet_pool_mutex) != osOK) {
            user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release circular buffer mutex after full buffer check");
        }
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "Circular buffer is full, cannot allocate packet");
        return NULL; // Buffer is full
    }

    // Allocate the packet at the 'head' position
    CAN_Packet *allocated_packet = &can_circular_buffer.packets[can_circular_buffer.head];
    can_circular_buffer.head = (can_circular_buffer.head + 1) % CAN_BUFFER_SIZE; // Move head to next position
    can_circular_buffer.count++; // Increment packet count

    // Release the mutex
    if (osMutexRelease(packet_pool_mutex) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release circular buffer mutex after allocation");
    }

    return allocated_packet; // Return the allocated packet
}


/**
 * @brief Frees a CAN packet back to the packet pool.
 *
 * This function marks a previously allocated CAN packet as unused,
 * making it available for future allocations. It acquires a mutex
 * to ensure thread-safe access to the shared packet pool.
 *
 * @param packet Pointer to the CAN packet to be freed.
 */
void _free_can_packet(CAN_Packet *packet) {
    // Validate the input to ensure a valid pointer is provided
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_INVALID_PACKET, "Attempted to free a NULL packet");
        return;
    }

    // Acquire the mutex to ensure exclusive access to the packet pool
    osMutexAcquire(packet_pool_mutex, osWaitForever);

    // Iterate through the packet pool to locate the packet
    for (int i = 0; i < CAN_PACKET_POOL_SIZE; i++) {
        // Check if the current packet matches the one being freed
        if (&can_circular_buffer[i].packet == packet) {
        	can_circular_buffer[i].used = 0; // Mark the packet as unused
            osMutexRelease(packet_pool_mutex); // Release the mutex immediately after the update
            return; // Exit after successfully freeing the packet
        }
    }

    // If the packet was not found, release the mutex and log an error
    osMutexRelease(packet_pool_mutex);
    user_error_handler(ERROR_CAN_PACKET_NOT_FOUND, "Attempted to free a packet not in the pool");
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
 * @brief Enqueues a CAN packet into the circular buffer for a specified instance.
 *
 * Adds the provided CAN packet to the buffer for the given CAN instance.
 * Handles buffer overflow by returning false and logging an error with context.
 *
 * @param can_instance The CAN instance (e.g., `CAN_TRUCK` or `CAN_AUX`).
 * @param packet Pointer to the `CAN_Packet` to enqueue.
 * @return true if the packet was successfully enqueued, false if the buffer is full or invalid.
 */
bool _enqueue_can_packet(CANInstance can_instance, const CAN_Packet *packet) {
    // Validate the CAN instance
    if (can_instance >= CAN_TOTAL) {
        user_error_handler(ERROR_CAN_INVALID_CONTEXT, "Invalid CAN instance for enqueue: %d", can_instance);
        return false;
    }

    // Access the buffer for the specified CAN instance
    CAN_Buffer *buffer = &can_circular_buffer[can_instance];

    // Check for buffer overflow
    if (buffer->count >= CAN_BUFFER_SIZE) {
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW,
                           "CAN buffer overflow for instance %d (head=%d, tail=%d, count=%d)",
                           can_instance, buffer->head, buffer->tail, buffer->count);
        return false;
    }

    // Add the packet to the circular buffer
    buffer->packets[buffer->head] = *packet; // Copy the packet into the buffer
    buffer->head = (buffer->head + 1) % CAN_BUFFER_SIZE; // Update the head pointer
    buffer->count++; // Increment the count of packets in the buffer

    return true;
}

/**
 * @brief Enqueue a CAN data packet for transmission using the new `CAN_Tx_Packet` structure.
 *
 * This function prepares a `CAN_Tx_Packet` with the provided header, metadata, and payload
 * and enqueues it in the Tx message queue for asynchronous transmission.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) to use for transmission.
 * @param txheader Pointer to the `CAN_TxHeaderTypeDef` containing header information.
 * @param payload Pointer to the payload data to be transmitted (8 bytes maximum).
 * @param timestamp Timestamp of the message, typically obtained via `HAL_GetTick()`.
 * @return true if the message was successfully enqueued, false otherwise.
 */
bool _enqueue_can_tx_packet(CANInstance can_instance, CAN_TxHeaderTypeDef *txheader, const uint8_t *payload, uint32_t timestamp) {
    // Prepare the CAN transmission packet
    CAN_Tx_Packet tx_packet = {0};  // Initialize the structure to zero

    // Populate the packet fields
    tx_packet.txheader = *txheader;                // Copy the Tx header
    tx_packet.meta.can_instance = can_instance;   // Set CAN instance
    tx_packet.meta.timestamp = timestamp;         // Set the timestamp
    memcpy(tx_packet.payload, payload, sizeof(tx_packet.payload));  // Copy the payload

    // Attempt to enqueue the packet in the Tx message queue
    if (osMessageQueuePut(Tx_QueueHandle, &tx_packet, 0, 0) != osOK) {
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, "Failed to enqueue CAN packet");
        return false;
    }

    return true;
}

/**
 * @brief Dequeues a CAN packet from the circular buffer.
 *
 * Retrieves the next available CAN packet from the circular buffer. This function
 * ensures thread-safe access by using a mutex to protect the buffer.
 *
 * @param packet Pointer to a `CAN_Packet` structure where the dequeued packet will be stored.
 * @return true if a packet was successfully dequeued, false if the buffer is empty.
 */
bool _dequeue_can_packet(CAN_Packet *packet) {
    // Validate input arguments
    if (packet == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Null pointer provided to _dequeue_can_packet");
        return false;
    }

    // Attempt to acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(packet_pool_mutex, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in dequeue");
        return false;
    }

    // Check if the buffer is empty
    if (can_circular_buffer.count == 0) {
        if (osMutexRelease(packet_pool_mutex) != osOK) {
            user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after empty buffer check in dequeue");
        }
        return false; // Buffer is empty
    }

    // Retrieve the next packet
    *packet = can_circular_buffer.packets[can_circular_buffer.tail];

    // Update the tail index and decrement the count
    can_circular_buffer.tail = (can_circular_buffer.tail + 1) % CAN_BUFFER_SIZE;
    can_circular_buffer.count--;

    // Release the mutex after modifying the buffer
    if (osMutexRelease(packet_pool_mutex) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after dequeuing");
    }

    return true; // Successfully dequeued a packet
}

