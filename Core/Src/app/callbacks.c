/*
 * callback.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdint.h>

#include "callbacks.h"
#include "main.h"
#include "can_common.h"
#include "buffers.h"
#include "rtos_tasks.h"
#include "ansi.h"

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

