/*
 * rtos.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares RTOS-specific utilities for managing CAN communication,
 * including initialization of message queues, handling packet pools, and transmitting CAN packets.
 */

#ifndef RTOS_H_
#define RTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "cmsis_os2.h"           // For RTOS constructs

#include "buffers.h"
#include "can_common.h"          // For CAN utilities
#include "log.h"                 // For logging utilities


/* --------------------------------------------------------------------------
   Global Variables
   -------------------------------------------------------------------------- */

/**
 * @brief Attributes for CAN1 Rx Queue.
 */
extern const osMessageQueueAttr_t CAN1_Rx_Queue_attributes;

/**
 * @brief Attributes for CAN2 Rx Queue.
 */
extern const osMessageQueueAttr_t CAN2_Rx_Queue_attributes;

/**
 * @brief Attributes for CAN1 Tx Queue.
 */
extern const osMessageQueueAttr_t CAN1_Tx_Queue_attributes;

/**
 * @brief Attributes for CAN2 Tx Queue.
 */
extern const osMessageQueueAttr_t CAN2_Tx_Queue_attributes;


/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Initializes RTOS message queue handles.
 *
 * Creates message queues for CAN communication and associates them with circular buffers.
 */
void init_rtos_queue_handles(void);

/**
 * @brief Sends a CAN packet through RTOS to the CAN interface.
 *
 * Delegates the actual transmission to the core CAN logic.
 *
 * @param packet Pointer to the `CAN_Packet` containing metadata, header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool __rtos_send_tx_packet_to_can_interface(CAN_Packet *packet);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_H_ */
