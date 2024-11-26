/*
 * queues.c
 *
 *  Created on: Nov 26, 2024
 *      Author: Ryan
 */

#include <stdio.h>

/* ** RTOS INCLUDES ******************************************************************************** */
#include "FreeRTOS.h"
#include "queue.h" // this is NOT this apps's queues.h. This is RTOS's and is needed for QueueHandle_t
/* ************************************************************************************************* */


#include "config.h"
#include "hid.h"
#include "queues.h" // this is this app's queues.h

QueueHandle_t UART1_HID_Rx_Queue; // Handle for the UART queue

void init_hid_uart_rx_queue(void) {
    // Create a queue to hold up to 10 packets
    UART1_HID_Rx_Queue = xQueueCreate(HID_QUEUE_RX_QUEUE_MAX, sizeof(HID_UART_Rx_Packet));

    if (UART1_HID_Rx_Queue == NULL) {
        // Handle queue creation failure
        printf("Failed to create UART queue!\n");
        while (1);
    }
}

void uartTask(void *pvParameters) {
	HID_UART_Rx_Packet packet;

    while (1) {
        // Wait for data from the queue
        if (xQueueReceive(UART1_HID_Rx_Queue, &packet, portMAX_DELAY) == pdPASS) {
            // Process the received packet
            printf("Received Packet: ");
            for (int i = 0; i < 5; i++) {
                printf("0x%02X ", packet.data[i]);
            }
            printf("\n");

            // Example: Handle packet contents
            if (packet.data[0] == 0x24 && packet.data[4] == 0x26) {
                uint8_t button_id = packet.data[2];
                uint8_t button_val = packet.data[3];

                switch (button_id) {
                    case 0x02: // Brake button
                        _g_sim_brake_on = (button_val == 0x01);
                        printf("Brake state: %s\n", _g_sim_brake_on ? "ON" : "OFF");
                        break;

                    default:
                        printf("Unknown button ID: 0x%02X\n", button_id);
                        break;
                }
            } else {
                printf("Invalid packet format\n");
            }
        }
    }
}
