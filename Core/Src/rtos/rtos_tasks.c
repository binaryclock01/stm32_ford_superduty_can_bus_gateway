/*
 * tasks.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdio.h>
#include "rtos_tasks.h"
#include "can_common.h"
#include "buffers.h"
#include "can_helper.h"
#include "can_rx.h"
#include "log.h"
#include "ansi.h"
#include "error.h"



void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance, CAN_Packet *packet)
{
	log_message(CRESET "* " BYEL "RTOS TASK CALL:" YEL "__rtos__StartCan_Rx_Task" CRESET);
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
	log_message(CRESET "* " BYEL "RTOS TASK CALL:" YEL "__rtos__StartCan_Tx_Task" CRESET);
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
			snprintf(error_msg, sizeof(error_msg), "%s with enum_can_instance = %d\r\n", __func__, enum_can_instance);
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
	_free_can_packet_using_queue_type_from_circular_buffer(queue_enum);
}

void CAN_ProcessingTask(void *argument) {
    for (;;) {
        // Wait for a signal from the ISR
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        while (g_isr_rx_buffer_read_index != g_isr_rx_buffer_write_index) {
            // Retrieve a packet from the ISR buffer
            CAN_Rx_Packet rx_packet = g_isr_rx_buffer[g_isr_rx_buffer_read_index];
            g_isr_rx_buffer_read_index = (g_isr_rx_buffer_read_index + 1) % ISR_BUFFER_SIZE;

            // Acquire the mutex and enqueue the message into the backend buffer
            osMutexAcquire(log_buffer_mutex_id, osWaitForever);
            add_to_circular_buffer(&rx_packet); // Add packet to the backend
            osMutexRelease(log_buffer_mutex_id);
        }
    }
}
