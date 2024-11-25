/*
 * can_tx.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares functions for constructing and transmitting CAN messages,
 * including generating payloads, setting up headers, and managing Tx queues.
 */

#ifndef CAN_TX_H_
#define CAN_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
//#include "can_common.h"          // Common CAN utilities
#include "device_configs.h"      // For CANDeviceConfig, CANDevicePID, etc.
#include "rtos.h"                // For RTOS utilities

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Generate the CAN payload based on the device and PID configuration.
 *
 * Constructs the CAN request payload by combining information from the provided
 * device and PID, optionally converting it to big-endian format for transmission.
 *
 * @param device Pointer to the CANDeviceConfig representing the target device.
 * @param pid Pointer to the CANDevicePID specifying the requested PID.
 * @param TxData Pointer to the data buffer where the payload will be stored.
 */
void generate_can_tx_read_data_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData);

/**
 * @brief Retrieve the appropriate CAN standard ID for the given device and verb.
 *
 * Returns the request or reply ID based on the provided verb.
 *
 * @param device Pointer to the CANDeviceConfig.
 * @param verb The CAN verb type (e.g., request or reply).
 * @return The standard ID for the CAN message.
 */
uint32_t get_can_device_stdid(CANDeviceConfig *device, CAN_Verb_Type verb);

/**
 * @brief Configure the CAN Tx header.
 *
 * Sets up the CAN transmission header with the specified request ID and default values
 * for the identifier type, data length code (DLC), and other metadata.
 *
 * @param tx_header Pointer to the CAN_TxHeaderTypeDef structure to populate.
 * @param std_id The standard identifier for the CAN message.
 */
void create_can_tx_header(CAN_TxHeaderTypeDef *tx_header, uint32_t std_id);

/**
 * @brief Prepare and enqueue a CAN request for a specific device and PID.
 *
 * Prepares a CAN message, stores it in the circular buffer, and enqueues a reference
 * to it in the transmission queue for processing by the Tx task.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2).
 * @param device Pointer to the CANDeviceConfig representing the target device.
 * @param pid Pointer to the CANDevicePID specifying the requested PID.
 * @param verb The CAN verb type (e.g., request or reply).
 */
void send_can_packet_to_tx_queue(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid, CAN_Verb_Type verb);

/**
 * @brief Sends a CAN packet using the HAL CAN API.
 *
 * Handles CAN-specific operations like header conversion, message transmission,
 * and logging.
 *
 * @param packet Pointer to the `CAN_Packet` containing metadata, header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool _send_tx_packet_to_can_interface(CAN_Packet *packet);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_H_ */
