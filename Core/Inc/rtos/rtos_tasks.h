/*
 * rtos_tasks.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares RTOS task functions for handling CAN Rx and Tx operations,
 * as well as processing CAN messages from an ISR.
 */

#ifndef RTOS_TASKS_H_
#define RTOS_TASKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include "rtos_tasks.h"
#include "can_common.h"          // Common CAN utilities


/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief RTOS task to process a received CAN packet.
 *
 * Validates and processes the received CAN packet based on the CAN instance,
 * then frees the packet from the circular buffer.
 *
 * @param enum_can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to the received CAN packet.
 */
void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance, CAN_Packet *packet);

/**
 * @brief RTOS task to process and transmit a CAN packet.
 *
 * Validates the packet, sends it via the CAN interface, and then frees
 * the packet from the circular buffer.
 *
 * @param enum_can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to the CAN packet to transmit.
 */
void __rtos__StartCAN_Tx_Task(CANInstance enum_can_instance, CAN_Packet *packet);

/**
 * @brief RTOS task for processing CAN packets from an ISR.
 *
 * Waits for a signal from the ISR, retrieves packets from the ISR buffer,
 * and enqueues them into the backend circular buffer for further processing.
 *
 * @param argument Pointer to any arguments for the task (unused in this implementation).
 */
void CAN_ProcessingTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_TASKS_H_ */
