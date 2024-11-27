/*
 * tasks.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdio.h>
#include <string.h>

/* ** RTOS INCLUDES ******************************************************************************** */
#include "FreeRTOS.h"
#include "queue.h" // this is NOT this apps's queues.h. This is RTOS's and is needed for QueueHandle_t
/* ************************************************************************************************* */

#include "rtos_tasks.h"
#include "can_common.h"
#include "buffers.h"
#include "can_helper.h"
#include "can_rx.h"
#include "log.h"
#include "ansi.h"
#include "error.h"
#include "queues.h"
#include "can_helper.h" // for UART task


void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance, CAN_Packet *packet)
{
	//log_message(CRESET "* " BYEL "RTOS TASK CALL:" YEL "__rtos__StartCan_Rx_Task" CRESET);
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

/*
// user code for button bt0
print "$" // start char
print "0" // page number
prints bt0.id,1  // send the ID
prints bt0.val,1 // send the value
print "&" // end char
24 30 02 01 26
*/

typedef enum {
	HID_RX_PACKET_START_CHAR = 0x24, // "$"
	HID_RX_PACKET_END_CHAR = 0x26, // "&"
} HID_Rx_Packet_Delimiters_Enum;

typedef enum {
	HID_RX_PACKET_START_CHAR_POS = 0,
	HID_RX_PACKET_PAGE_NUM_POS,
	HID_RX_PACKET_ID_POS,
	HID_RX_PACKET_VAL_POS,
	HID_RX_PACKET_END_CHAR_POS,
} HID_Rx_Packet_Positions_Enum;

void __rtos__StartHID_RxUART_Task(void *pvParameters, HID_UART_Rx_Packet *packet) {
    uint8_t uart_rx_buffer[HID_RXDATA_MAX_LENGTH * 2] = {0}; // Temporary buffer for resynchronization
    size_t uart_rx_index = 0; // Tracks the current write index in the buffer

    while (1) {
        // Wait for data from the queue
        if (xQueueReceive(UART1_HID_Rx_Queue, packet, portMAX_DELAY) == pdPASS) {
            // Append the received packet data to the UART buffer
            memcpy(&uart_rx_buffer[uart_rx_index], packet->data, HID_RXDATA_MAX_LENGTH);
            uart_rx_index += HID_RXDATA_MAX_LENGTH;

            size_t index = 0;
            while (index + HID_RXDATA_MAX_LENGTH <= uart_rx_index) {
                // Check for valid packet format
                if (uart_rx_buffer[index + HID_RX_PACKET_START_CHAR_POS] == HID_RX_PACKET_START_CHAR &&
                    uart_rx_buffer[index + HID_RX_PACKET_END_CHAR_POS] == HID_RX_PACKET_END_CHAR) {

                    // Copy valid packet into `packet`
                    memcpy(packet->data, &uart_rx_buffer[index], HID_RXDATA_MAX_LENGTH);

                    // Log the received packet
                    char buf[255];
                    size_t len = 0;
                    buf[0] = '\0'; // Initialize the buffer to an empty string

                    for (int i = 0; i < HID_RXDATA_MAX_LENGTH; i++) {
                        int written = snprintf(buf + len, sizeof(buf) - len, "0x%02X ", packet->data[i]);
                        if (written < 0 || written >= (int)(sizeof(buf) - len)) {
                            snprintf(buf + len, sizeof(buf) - len, "..."); // Add ellipsis if truncated
                            break;
                        }
                        len += written;
                    }
                    log_message(buf);

                    // Process packet content
                    uint8_t button_id = packet->data[HID_RX_PACKET_ID_POS];
                    uint8_t button_val = packet->data[HID_RX_PACKET_VAL_POS];

                    CANSignal *signal = NULL;
                    char signal_name[100] = {0};

                    switch (button_id)
                    {
                        case 0x02: sprintf(signal_name, "Brake Pedal"); break;
                        case 0x03: sprintf(signal_name, "Hazard Button"); break;
                        case 0x04: sprintf(signal_name, "Reverse Light"); break;
                        case 0x05: sprintf(signal_name, "Left Turn Signal"); break;
                        case 0x06: sprintf(signal_name, "Left Lane Change"); break;
                        case 0x07: sprintf(signal_name, "Right Turn Signal"); break;
                        case 0x08: sprintf(signal_name, "Right Lane Change"); break;

                        default:
                            log_message("Unknown button ID: 0x%02X\n", button_id);
                            break;
                    }

					signal = get_pid_signal_by_name(signal_name);
					if (signal != NULL) {
					  signal->data = (button_val > 0x00 ? 1 : 0);
					  log_message("%s state: %s", signal_name, signal->data ? "ON" : "OFF");
					} else {
					  log_message("* COULD NOT FIND SIGNAL BY NAME!");
					}

                    // Move to the next possible packet
                    index += HID_RXDATA_MAX_LENGTH;

                } else {
                    // Invalid packet: Find the next start delimiter
                    log_message("Invalid packet format\n");

                    bool found = false;
                    for (size_t i = index + 1; i < uart_rx_index; i++) {
                        if (uart_rx_buffer[i] == HID_RX_PACKET_START_CHAR) {
                            index = i;
                            found = true;
                            break;
                        }
                    }

                    // Reset buffer if no valid start marker found
                    if (!found) {
                        uart_rx_index = 0;
                        break;
                    }
                }
            }

            // Compact the buffer by removing processed bytes
            if (index > 0) {
                memmove(uart_rx_buffer, &uart_rx_buffer[index], uart_rx_index - index);
                uart_rx_index -= index;
            }
        }
    }
}
