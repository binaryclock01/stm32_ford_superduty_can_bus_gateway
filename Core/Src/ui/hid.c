/*
 * hid.c
 *
 *  Created on: Nov 26, 2024
 *      Author: Ryan
 */

/* ** RTOS INCLUDES ******************************************************************************** */
#include "FreeRTOS.h"
#include "queue.h" // this is NOT this apps's queues.h. This is RTOS's and is needed for QueueHandle_t
/* ************************************************************************************************* */

#include "config.h"
#include "hid.h"


uint8_t _g_hid_rxdata[HID_RXDATA_MAX_LENGTH];


