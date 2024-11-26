/*
 * callback.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "main.h"
#include "cmsis_os.h"

#include "callbacks.h"
#include "can_common.h"
#include "buffers.h"
#include "rtos_tasks.h"
#include "ansi.h"
#include "main.h"
#include "hid.h"
#include "buffers.h"
#include "queues.h"

#ifdef IS_SIMULATOR
#include "sim.h"
#endif


#ifdef IS_SIMULATOR

/*
 * extern uint8_t _g_hid_rx_sliding_buffer[HID_RX_BUFFER_SIZE]; // Circular buffer for receiving data
* extern uint8_t _g_hid_rx_packet_buffer[HID_RXDATA_MAX_LENGTH];       // Buffer to hold a single valid packet
* extern uint8_t _g_hid_rx_sliding_buffer_index;
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == HID_DEVICE_UART) {
    	HID_UART_Rx_Packet packet;

        // Copy received data into the packet structure
        memcpy(packet.data, _g_hid_rxdata, HID_RXDATA_MAX_LENGTH);

        // Send the packet to the queue (from ISR)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(UART1_HID_Rx_Queue, &packet, &xHigherPriorityTaskWoken);

        // Restart DMA reception
        HAL_UART_Receive_DMA(huart, _g_hid_rxdata, HID_RXDATA_MAX_LENGTH);

        // Request a context switch if necessary
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
	/*
	uint8_t *sliding_buf = _g_hid_rx_sliding_buffer;
	uint8_t *packet_buf = _g_hid_rx_packet_buffer;
	uint8_t *sliding_buf_index = &_g_hid_rx_sliding_buffer_index;

	if (huart->Instance == USART1) {
		// Add new data to the buffer
		for (int i = 0; i < 5; i++) {
			sliding_buf[*sliding_buf_index] = _g_sim_nextion_rxdata[i];
			*sliding_buf_index = (*sliding_buf_index + 1) % HID_RX_BUFFER_SIZE; // Circular buffer logic
		}

		// Try to extract valid packets
		while (1) {
			int start_index = -1;

			// Search for the start marker (`0x24`)
			for (int i = 0; i < HID_RX_BUFFER_SIZE; i++) {
				if (sliding_buf[i] == 0x24) { // Found start marker
					start_index = i;
					break;
				}
			}

			// No valid start marker found, stop processing
			if (start_index == -1) {
				break;
			}

			// Check if there are enough bytes for a full packet
			int end_index = (start_index + 4) % HID_RX_BUFFER_SIZE; // End marker index
			if (sliding_buf[end_index] == 0x26) { // Found a valid end marker
				// Extract the packet
				for (int i = 0; i < 5; i++) {
					packet_buf[i] = sliding_buf[(start_index + i) % HID_RX_BUFFER_SIZE];
				}

				// Process the valid packet
				process_uart_packet(packet_buffer);

				// Remove the processed packet from the buffer
				buffer_index = (end_index + 1) % BUFFER_SIZE; // Move index past the processed packet
			} else {
				// No valid end marker, discard bytes up to the start marker
				buffer_index = (start_index + 1) % BUFFER_SIZE;
				break;
			}
		}

		// Restart DMA for the next batch of bytes
		HAL_UART_Receive_DMA(huart, _g_sim_nextion_rxdata, 5);
	}
	*/
}
#endif /* IS_SIMULATOR */
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
void __callback__process_can_rx_fifo_callback(CAN_HandleTypeDef *hcan)
{
	_process_can_rx_fifo_callback(hcan);
}

void activate_can_bus_fifo_callbacks()
{
    log_message("* Initializing CAN BUS FIFO Callback Functions");
#ifdef USE_CAN_1
	activate_single_can_bus_fifo_callback(&hcan1, "  - " HWHT "TruckNet " CRESET "[" HWHT "CAN1" CRESET "]");
#endif

#ifdef USE_CAN_2
	activate_single_can_bus_fifo_callback(&hcan2, "  - " HWHT "AuxNet " CRESET "[" HWHT "CAN2" CRESET "]");
#endif
}


void activate_single_can_bus_fifo_callback(CAN_HandleTypeDef *hcan, const char *hcan_human_string)
{
	  // Activate notification for TRUCK CAN
	  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
	  {
		  log_status_message(hcan_human_string, true);
	  }
	  else
	  {
		  log_status_message(hcan_human_string, true);
	      user_error_handler(ERROR_CAN_NOTIFICATION_FAILED, "Failed to activate CAN_IT_RX_FIFO0_MSG_PENDING notification on ", hcan_human_string);
	  }
}

