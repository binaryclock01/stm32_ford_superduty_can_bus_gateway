/*
 * queues.h
 *
 *  Created on: Nov 26, 2024
 *      Author: Ryan
 */

#ifndef INC_APP_QUEUES_H_
#define INC_APP_QUEUES_H_

#include <stdint.h>

/* ** RTOS INCLUDES ******************************************************************************** */
#include "FreeRTOS.h"
#include "queue.h" // this is NOT this apps's queues.h. This is RTOS's and is needed for QueueHandle_t
/* ************************************************************************************************* */

#include "config.h"
#include "main.h"
#include "cmsis_os.h"

#include "buffers.h"
#include "sim.h"
#include "hid.h"

extern QueueHandle_t UART1_HID_Rx_Queue; // Handle for the UART queue

// Define a packet structure
typedef struct {
    uint8_t data[HID_RXDATA_MAX_LENGTH]; // The 5-byte packet
} HID_UART_Rx_Packet;

void init_hid_uart_rx_queue(void);

#endif /* INC_APP_QUEUES_H_ */
