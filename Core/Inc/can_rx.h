/*
 * can_rx.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares functions for receiving and processing CAN messages,
 * including normalization of headers, validation, and populating CAN packets.
 */

#ifndef CAN_RX_H_
#define CAN_RX_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>
#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support

#include "buffers.h"
#include "can_common.h"          // Common CAN utilities
//#include "device_configs.h"      // For CANDeviceConfig, CANDevicePID, etc.
//#include "rtos.h"


/*
#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "buffers.h"
#include "can_common.h"
*/

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */


/**
 * @brief Process a received CAN packet.
 *
 * Validates, parses, and processes a received CAN packet, updating signal states
 * or sending responses as needed.
 *
 * @param queue_enum The circular queue type.
 * @param can_instance_enum The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to the CAN_Packet structure containing the received data.
 */
void process_can_rx_packet(Circular_Queue_Types queue_enum, CANInstance can_instance_enum, CAN_Packet *packet);

/**
 * @brief Translates a HAL CAN Rx header into the unified CAN_Header format.
 *
 * Maps fields from HAL's CAN_RxHeaderTypeDef to the unified CAN_Header structure.
 *
 * @param hal_rx_header Pointer to the HAL CAN Rx header.
 * @param header Pointer to the unified CAN_Header structure to populate.
 */
void normalize_rx_hal_header(const CAN_RxHeaderTypeDef *hal_rx_header, CAN_Header *header);

/**
 * @brief Retrieve a CAN message from FIFO0 and populate a CAN_Packet.
 *
 * Reads a CAN message from FIFO0, validates it, and populates the provided
 * CAN_Packet structure.
 *
 * @param hcan Pointer to the HAL CAN handle (e.g., CAN1 or CAN2).
 * @param packet Pointer to the CAN_Packet to store the received message.
 * @return true if the message was successfully retrieved, false otherwise.
 */
bool get_rx_message_from_CAN_RX_FIFO0(CAN_HandleTypeDef *hcan, CAN_Rx_Packet *packet);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RX_H_ */
