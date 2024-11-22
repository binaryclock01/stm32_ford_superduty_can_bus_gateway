/**
 * @file rtos.c
 * @brief RTOS-specific utilities for managing CAN communication.
 *
 * This file contains implementations for handling RTOS constructs
 * such as mutexes, message queues, and tasks. These utilities are
 * designed to manage CAN packet pools, safely enqueue/dequeue packets,
 * and support multi-threaded communication.
 */
#include <string.h> // For memcpy
#include <stdio.h>

#include "main.h"  // For CAN handles and HAL functions
#include "rtos.h"
#include "can_core.h"
#include "error.h" // For error handling
#include "ui.h"

// Mutex for protecting the CAN packet pool


/*
// CAN message queues
osMessageQueueId_t CAN1_Rx_QueueHandle;
osMessageQueueId_t CAN2_Rx_QueueHandle;
osMessageQueueId_t CAN1_Tx_QueueHandle;
osMessageQueueId_t CAN2_Tx_QueueHandle;
*/

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
    CAN_Packet *packet = &(buffer->packets[buffer->head]);

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
 * @brief Processes a message from the TX queue and sends it to the TRUCK_CAN (hcan1).
 *
 * @return bool Returns true if the message was sent successfully, false otherwise.
 */
/*
bool __rtos_send_tx_packet_to_can_interface(CANInstance enum_can_instance, CAN_Packet *packet) {

	CAN_HandleTypeDef *hcan = get_hcan_from_instance(enum_can_instance);

	osMessageQueueId_t queue_handle = get_queue_handle_by_can_instance(enum_can_instance, QUEUE_TYPE_FLOW_TX);

	if (queue_handle == NULL)
	{
		char error_msg[255]; // a lot more chars just in case the __func__ name changes :)
		snprintf(error_msg, sizeof(error_msg), "%s with enum_can_instance = %d", __func__, enum_can_instance);
		user_error_handler(ERROR_RTOS_QUEUE_INVALID_HANDLE, error_msg);
		return false;
	}

    // Step 1: Validate the packet isn't NULL. we do it here so we have the queue_handle to free the packet if NULL
    if (packet == NULL) {
    	char error_msg[255];
    	snprintf(error_msg, sizeof(error_msg), "%s: NULL packet, TX queue enum: %du **freed packet**", __func__, enum_can_instance);
    	user_error_handler(ERROR_RTOS_QUEUE_NULL_PACKET, error_msg);
    	_free_can_packet_using_queue_handle_from_circular_buffer(queue_handle);
        return false;
    }

    // Step 2: Prepare the HAL CAN Tx header
    CAN_TxHeaderTypeDef hal_tx_header = {
        .StdId = packet->header.id,
        .ExtId = packet->header.id,
        .IDE = packet->header.is_extended_id ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = packet->header.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = packet->header.dlc,
        .TransmitGlobalTime = DISABLE
    };

    // Step 3: Send the message via HAL CAN API
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(hcan, &hal_tx_header, packet->payload, &mailbox) != HAL_OK) {
        // Transmission failed, log the error
        char error_msg[255];
        snprintf(error_msg, sizeof(error_msg), "%s: CAN TX failed: ID=0x%lX", __func__, packet->header.id);
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, error_msg);

        // TODO: Optionally re-enqueue the message or handle failure
        // it probably is a valid packet, just failed xfer for some reason.
        // therefore, do not free the packet.. let the calling function handle the freeing upon error.
        return false;
    }

    // Step 5: Log successful transmission
    char success_msg[64];
    snprintf(success_msg, sizeof(success_msg), "CAN TX success: ID=0x%lX", packet->header.id);
    send_console_msg(success_msg);

    // Step 6: Free the packet after successful transmission
    _free_can_packet_from_circular_buffer(queue_handle);

    return true;
}
*/


void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance, CAN_Packet *packet)
{
	Circular_Queue_Types queue_enum = QUEUE_TYPE_UNKNOWN;

	// TODO: probably a better way to do this with enums and structs, but this is what it is for now.
	// It's more human readable, but it takes more cycles. The STM32F44 is running at 180MHz so I don't think we'll
	// have an issue at this point.
	// Faster would be specific StartCAN_Rx#_Tasks, which I had before, but I hate duplicating functions.
	// Embedded sometimes calls for more code and less cycles, however.
	switch (enum_can_instance)
	{
		case CAN_TRUCK:
			queue_enum = QUEUE_RX_CAN1;
			break;
		case CAN_AUX:
			queue_enum = QUEUE_RX_CAN2;
			break;
		default:
			char error_msg[100]; // a lot more chars just in case the __func__ name changes :)
			snprintf(error_msg, sizeof(error_msg), "%s with enum_can_instance = %d", __func__, enum_can_instance);
			user_error_handler(ERROR_RTOS_QUEUE_INVALID_HANDLE, error_msg);
			return;
	}

	if (packet == NULL) {
		char error_msg[255];
		snprintf(error_msg, sizeof(error_msg), "%s: NULL packet, RX queue enum: %du **freed packet**", __func__, enum_can_instance);
		user_error_handler(ERROR_RTOS_QUEUE_NULL_PACKET, error_msg);
		_free_can_packet_using_queue_type_from_circular_buffer(queue_enum);
		return;
	}


	process_can_rx_packet(queue_enum, enum_can_instance, packet); // Step 3: Process the received packet
	_free_can_packet_using_queue_type_from_circular_buffer(queue_enum);
}

void __rtos__StartCAN_Tx_Task(CANInstance enum_can_instance, CAN_Packet *packet)
{
	Circular_Queue_Types queue_enum = QUEUE_TYPE_UNKNOWN;

	// TODO: probably a better way to do this with enums and structs, but this is what it is for now.
	// It's more human readable, but it takes more cycles. The STM32F44 is running at 180MHz so I don't think we'll
	// have an issue at this point.
	// Faster would be specific StartCAN_Rx#_Tasks, which I had before, but I hate duplicating functions.
	// Embedded sometimes calls for more code and less cycles, however.
	switch (enum_can_instance)
	{
		case CAN_TRUCK:
			queue_enum = QUEUE_TX_CAN1;
			break;
		case CAN_AUX:
			queue_enum = QUEUE_TX_CAN2;
			break;
		default:
			char error_msg[100]; // a lot more chars just in case the __func__ name changes :)
			snprintf(error_msg, sizeof(error_msg), "%s with enum_can_instance = %d", __func__, enum_can_instance);
			user_error_handler(ERROR_RTOS_QUEUE_INVALID_HANDLE, error_msg);
			return;
	}

	if (packet == NULL)
	{
		_free_can_packet_using_queue_type_from_circular_buffer(queue_enum);
		char error_msg[255];
		snprintf(error_msg, sizeof(error_msg), "%s: NULL packet, TX queue enum: %du, **freed packet**", __func__, enum_can_instance);
		user_error_handler(ERROR_RTOS_QUEUE_NULL_PACKET, error_msg);
		return;
	}
	__rtos_send_tx_packet_to_can_interface(packet);
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
    osMessageQueueId_t *target_queue = get_rx_queue_handle_from_hcan(hcan);
    if (target_queue == NULL) {
        // Log and exit if the target queue is unavailable
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Message queue unavailable for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_INIT_FAILED, error_msg);
        return;
    }

    Circular_Queue_Types queue_type = get_queue_num_by_can_instance(can_instance, QUEUE_TYPE_FLOW_RX);

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
        _free_can_packet_using_queue_type_from_circular_buffer(queue_type);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Failed to retrieve CAN message for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, error_msg);
        return;
    }

    // Step 5: Enqueue the packet into the message queue
    if (osMessageQueuePut(target_queue, packet, 0, 0) != osOK) {
        // Free the packet and log the error if the queue is full or unavailable
        _free_can_packet_using_queue_type_from_circular_buffer(queue_type);
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
osMessageQueueId_t *get_rx_queue_handle_from_hcan(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        return &can_circular_buffer[QUEUE_RX_CAN1].queue_handle;
    } else if (hcan == &hcan2) {
        return &can_circular_buffer[QUEUE_RX_CAN2].queue_handle;
    } else {
        return NULL; // Unknown CAN instance
    }
}

bool _free_can_packet_using_queue_buffer_from_circular_buffer(CAN_Circular_Buffer *queue_buffer)
{
    // Attempt to acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(queue_buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in dequeue");
        return false;
    }

    // Check if the buffer is empty
    if (queue_buffer->count == 0) {
        osMutexRelease(queue_buffer->mutex_id);
        return false; // Buffer is empty
    }

    // Update the tail index and decrement the count
    queue_buffer->tail = (queue_buffer->tail + 1) % CAN_BUFFER_SIZE;
    queue_buffer->count--;

    // Release the mutex after modifying the buffer
    if (osMutexRelease(queue_buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after dequeuing");
        return false; // Error releasing mutex
    }

    return true; // Successfully dequeued a packet
}

/**
 * @brief Dequeues a CAN packet from the circular buffer for a specified queue.
 *
 * Retrieves the next available CAN packet from the specified circular buffer.
 * This function ensures thread-safe access by using a mutex to protect the buffer.
 *
 * @param queue_type The queue type (e.g., QUEUE_RX_CAN1, QUEUE_RX_CAN2, QUEUE_TX).
 * @return The dequeued `CAN_Packet *` if successful, or `NULL` if the buffer is empty.
 */
bool _free_can_packet_using_queue_type_from_circular_buffer(Circular_Queue_Types queue_type) {
    // Validate input arguments
    if (queue_type >= TOTAL_QUEUES) {
        char error_msg[255];
        snprintf(error_msg, sizeof(error_msg), "%s: Invalid queue type", __func__);
        user_error_handler(ERROR_INVALID_ARGUMENT, error_msg);
        return NULL;
    }

    // Access the corresponding circular buffer
    CAN_Circular_Buffer *buffer = &can_circular_buffer[queue_type];

    return _free_can_packet_using_queue_buffer_from_circular_buffer(buffer);
}



