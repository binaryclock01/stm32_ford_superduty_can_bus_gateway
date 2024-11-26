/*
 * buffers.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdio.h>
#include "FreeRTOS.h"  // For pvPortMalloc

#include "config.h"
#include "buffers.h"
#include "log.h"
#include "can_helper.h"
#include "ansi.h"
#include "error.h"
#include "can_rx.h"
#include "hid.h"


// sliding buffer variables for HID (touch screen)

/* Implemented queues instead... keeping this for now
uint8_t _g_hid_rx_sliding_buffer[HID_RX_BUFFER_SIZE]; // Circular buffer for receiving data
uint8_t _g_hid_rx_packet_buffer[HID_RXDATA_MAX_LENGTH];       // Buffer to hold a single valid packet
uint8_t _g_hid_rx_sliding_buffer_index = 0;       // Current index in rx_buffer
*/

const char* Circular_Queue_Types_Names[] = {
	    "QUEUE_RX_CAN1",
	    "QUEUE_TX_CAN1",
	    "QUEUE_RX_CAN2",
	    "QUEUE_TX_CAN2",
	    "TOTAL_QUEUES",
	    "QUEUE_TYPE_UNKNOWN"
	};


// Circular buffer for all queues and CANs

CAN_Circular_Buffer can_circular_buffer[TOTAL_QUEUES];

// needed for the FIFO0 Callback function to write to because it is called by an interrupt
// the stm32 library does not allow us to create a MUTEX when an interrupt calls the task because of blocking situations
// therefore this is a temporary holding buffer to hold the rx packets until StartCAN1_Rx_Process_ISR_Buffer can run
// and transfer the packets to the correct Rx packet buffer
volatile CAN_Rx_Packet g_isr_rx_buffer[ISR_BUFFER_SIZE];

volatile size_t g_isr_rx_buffer_write_index = 0;
volatile size_t g_isr_rx_buffer_read_index = 0;

LogBuffer *_g_ptr_log_buffer = NULL; // Global pointer to heap-allocated log buffer

void __buffers_destroy_log_buffer__heap__(void)
{
	// _g_ptr_log_buffer is a global pointer, pointing to the log buffer created in the heap
	if (_g_ptr_log_buffer != NULL) {
		if (_g_ptr_log_buffer->mutex_id != NULL) {
			osMutexDelete(_g_ptr_log_buffer->mutex_id); // Delete the mutex
		}
		if (_g_ptr_log_buffer->messages != NULL) {
			vPortFree(_g_ptr_log_buffer->messages); // Free the messages array
		}
		vPortFree(_g_ptr_log_buffer); // Free the structure itself
	}
}

LogBuffer *get_log_buffer_global_ptr()
{
	return (LogBuffer *)_g_ptr_log_buffer;
}

void report_log_buffer_fullness(void) {
	LogBuffer *buffer = get_log_buffer_global_ptr();
	if (buffer == NULL) {
        printf("Log buffer is not initialized.\n");
        return;
    }

    printf("* Log buffer fullness:\n");
    printf("  - Messages in buffer: " BWHT "%d" CRESET " / " BWHT "%d" CRESET "\r\n", buffer->count, LOG_BUFFER_SIZE);
    printf("  - Percent full: " BWHT "%.2f%%" CRESET "\r\n", (buffer->count / (float)LOG_BUFFER_SIZE) * 100.0);
}

void report_log_buffer_heap_size(void) {
	LogBuffer *buffer = get_log_buffer_global_ptr();
	if (buffer == NULL) {
        printf("Log buffer is not initialized.\n");
        return;
    }

    uint32_t structure_size = sizeof(LogBuffer);
    uint32_t messages_size = LOG_BUFFER_SIZE * LOG_MESSAGE_MAX_LENGTH;
    uint32_t total_size = structure_size + messages_size;

    char buf[255];
    log_message("* Log buffer HEAP usage:");
    sprintf(buf, "  - Structure size: " BWHT "%lu " CRESET "bytes", structure_size);
    log_message(buf);
    sprintf(buf, "  - Messages array size: " BWHT "%lu " CRESET "bytes", messages_size);
    log_message(buf);
    sprintf(buf, "  - Total size: " BWHT "%lu " CRESET "bytes", total_size);
    log_message(buf);
}

bool __buffers__create_log_buffer__heap__(void)
{
    // Allocate memory for the LogBuffer structure
	_g_ptr_log_buffer = (LogBuffer *)pvPortMalloc(sizeof(LogBuffer));
    if (_g_ptr_log_buffer == NULL) {
        user_error_handler(ERROR_HEAP_CREATE_LOG_BUFFER, "Failed to allocate memory for log buffer structure!");
        return false; // Allocation failed
    }

    // Initialize the structure
    _g_ptr_log_buffer->head = 0;
    _g_ptr_log_buffer->tail = 0;
    _g_ptr_log_buffer->count = 0;

    // Allocate memory for the messages array
    _g_ptr_log_buffer->messages = (char (*)[LOG_MESSAGE_MAX_LENGTH])pvPortMalloc(LOG_BUFFER_SIZE * LOG_MESSAGE_MAX_LENGTH);
    if (_g_ptr_log_buffer->messages == NULL) {
        log_message("Failed to allocate memory for log messages array!");
        vPortFree(_g_ptr_log_buffer); // Free the structure if array allocation fails
        return false;
    }

    // Initialize the mutex for thread-safe access
    _g_ptr_log_buffer->mutex_id = osMutexNew(NULL);
    if (_g_ptr_log_buffer->mutex_id == NULL) {
        log_message("Failed to create mutex for log buffer!");
        vPortFree(_g_ptr_log_buffer->messages); // Free the array
        vPortFree(_g_ptr_log_buffer);           // Free the structure
        return false;
    }
    return true;
}

void init_circular_buffers(void) {
    char buf[255];

    log_message("* Initializing CAN circular buffers");
    flush_logs();

    for (int i = 0; i < TOTAL_QUEUES; i++) {
        can_circular_buffer[i] = (CAN_Circular_Buffer){0}; // Clear the buffer
        // can_circular_buffer[i].mutex_id = osMutexNew(NULL); // Create a mutex for the buffer

        // Use the enum name instead of the number
        sprintf(buf, "  - Circular buffer %s%s%s initializing", HWHT, Circular_Queue_Types_Names[i], CRESET);

        /*
        if (can_circular_buffer[i].mutex_id == NULL) {
            log_status_message(buf, false);
            flush_logs();
            user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to initialize mutex for circular buffer %s", Circular_Queue_Types_Names[i]);
        }
        */
        log_status_message(buf, true);
        flush_logs();
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

    /*
    // Acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in allocate");
        return NULL;
    }
*/
    // Check if the buffer is full
    if (buffer->count >= CAN_BUFFER_SIZE) {
    	/*
        if (osMutexRelease(buffer->mutex_id) != osOK) {
            user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after full buffer check in allocate");
        }
        */
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "No free packets available in the circular buffer");
        return NULL;
    }

    // Allocate the next available packet
    CAN_Packet *packet = &(buffer->packets[buffer->head]);

    // Update the head index and increment the count
    buffer->head = (buffer->head + 1) % CAN_BUFFER_SIZE;
    buffer->count++;

    /*
    // Release the mutex after modifying the buffer
    if (osMutexRelease(buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after allocation");
    }
     */

    return packet; // Return the allocated packet
}

bool _free_can_packet_using_queue_buffer_from_circular_buffer(CAN_Circular_Buffer *queue_buffer)
{
	/*
    // Attempt to acquire the mutex for exclusive access to the circular buffer
    if (osMutexAcquire(queue_buffer->mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire circular buffer mutex in dequeue");
        return false;
    }
	*/

    // Check if the buffer is empty
    if (queue_buffer->count == 0) {
        // osMutexRelease(queue_buffer->mutex_id);
        return false; // Buffer is empty
    }

    // Update the tail index and decrement the count
    queue_buffer->tail = (queue_buffer->tail + 1) % CAN_BUFFER_SIZE;
    queue_buffer->count--;

    /*
    // Release the mutex after modifying the buffer
    if (osMutexRelease(queue_buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex after dequeuing");
        return false; // Error releasing mutex
    }
    */

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


/**
 * @brief Retrieves the message queue handle for a given CAN hardware instance.
 *
 * Maps a hardware CAN handle (e.g., `hcan1`) to the corresponding RTOS
 * message queue handle for receiving messages.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., `hcan1` or `hcan2`).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t get_rx_queue_handle_from_hcan(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        return can_circular_buffer[QUEUE_RX_CAN1].queue_handle;
    } else if (hcan == &hcan2) {
        return can_circular_buffer[QUEUE_RX_CAN2].queue_handle;
    } else {
        return NULL; // Unknown CAN instance
    }
}
