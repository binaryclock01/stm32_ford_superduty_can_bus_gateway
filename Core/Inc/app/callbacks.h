/*
 * callbacks.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares callback functions for handling CAN Rx FIFO messages
 * and activating CAN bus FIFO notifications.
 */

#ifndef CALLBACKS_H_
#define CALLBACKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include "config.h"
#include "main.h"
#include "cmsis_os.h"

#include "rtos.h"                // RTOS utilities
#include "can_common.h"          // Common CAN utilities
#include "log.h"                 // Logging utilities
#include "error.h"               // Error handling utilities
#include "buffers.h"
#include "can_rx.h"              // CAN Rx processing functions

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Processes a CAN Rx FIFO0 message callback.
 *
 * Triggered when a CAN message is received in FIFO0. Allocates a packet, retrieves
 * the message, and enqueues it into the ISR buffer for further processing.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 */
void __callback__process_can_rx_fifo_callback(CAN_HandleTypeDef *hcan);

/**
 * @brief Activates FIFO callbacks for all CAN buses.
 *
 * Initializes and activates message pending notifications for all enabled CAN buses.
 */
void activate_can_bus_fifo_callbacks(void);

/**
 * @brief Activates the FIFO callback for a single CAN bus.
 *
 * Enables notifications for message pending events on the specified CAN hardware instance.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @param hcan_human_string Human-readable name for logging purposes.
 */
void activate_single_can_bus_fifo_callback(CAN_HandleTypeDef *hcan, const char *hcan_human_string);

#ifdef __cplusplus
}
#endif

#endif /* CALLBACKS_H_ */
