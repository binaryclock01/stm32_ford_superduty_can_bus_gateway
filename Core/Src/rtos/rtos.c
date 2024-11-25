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

//#include "main.h"  // For CAN handles and HAL functions
//#include "config.h"
//#include "rtos.h"
//#include "can_core.h"

//#include "ui.h"
#include "config.h"
#include "can_common.h"
#include "log.h"
#include "ansi.h"
#include "buffers.h"
#include "error.h" // For error handling
#include "can_tx.h"


const osMessageQueueAttr_t CAN1_Rx_Queue_attributes = {
  .name = "CAN1_Rx_Queue"
};

const osMessageQueueAttr_t CAN2_Rx_Queue_attributes = {
  .name = "CAN2_Rx_Queue"
};

const osMessageQueueAttr_t CAN1_Tx_Queue_attributes = {
  .name = "CAN1_Tx_Queue"
};

const osMessageQueueAttr_t CAN2_Tx_Queue_attributes = {
  .name = "CAN2_Tx_Queue"
};

osMessageQueueId_t g_can_rx_isr_queue = {0};

void init_rtos_queue_handles(void)
{
	char buf[255];

    log_message("* Initializing RTOS queue handles");
    flush_logs();

    const osMessageQueueAttr_t *queue_attributes[TOTAL_QUEUES] = {
        &CAN1_Rx_Queue_attributes,
        &CAN1_Tx_Queue_attributes,
        &CAN2_Rx_Queue_attributes,
        &CAN2_Tx_Queue_attributes
    };

	for (int i = 0; i < TOTAL_QUEUES; i++) {
		can_circular_buffer[i].queue_handle = osMessageQueueNew(CAN_PACKET_POOL_SIZE, sizeof(CAN_Packet *), queue_attributes[i]);

        sprintf(buf, "  - Queue handle %s%s%s initializing", HWHT, Circular_Queue_Types_Names[i], CRESET);

		if (can_circular_buffer[i].queue_handle == NULL) {
            log_status_message(buf, false);
            flush_logs();
			user_error_handler(ERROR_RTOS_QUEUE_INIT_FAILED, "Failed to initialized queue handle for %s", Circular_Queue_Types_Names[i]);
		}
		log_status_message(buf, true);
		flush_logs();
	}
}

/**
 * @brief Processes and sends a CAN packet from RTOS.
 *
 * Delegates CAN-specific logic to `can_core.c` for separation of concerns.
 *
 * @param packet Pointer to the `CAN_Packet` containing the metadata, header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool __rtos_send_tx_packet_to_can_interface(CAN_Packet *packet) {
    if (packet == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Tx packet pointer is NULL");
        return false;
    }

    // Delegate to CAN-specific function in can_core.c
    bool result = _send_tx_packet_to_can_interface(packet);

    return result;
}




