/*
 * can_helper.h
 *
 * Created on: Nov 24, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares helper functions for CAN communication, including
 * retrieving CAN instances, managing queue mappings, and building request payloads.
 */

#ifndef CAN_HELPER_H_
#define CAN_HELPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "device_configs.h"      // For CANDeviceConfig, CANDevicePID, etc.
#include "rtos.h"                // For RTOS utilities
//#include "can_common.h"          // Common CAN utilities

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Retrieves the CAN instance for the given hardware instance.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return CANInstance Enum value representing the CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 */
CANInstance get_can_instance_enum(CAN_HandleTypeDef *hcan);

/**
 * @brief Build a CAN request payload for a specific device and PID.
 *
 * Constructs a 64-bit CAN request payload by combining the request size,
 * command byte, and PID in big-endian format.
 *
 * @param device Pointer to the CANDeviceConfig representing the device.
 * @param pid Pointer to the CANDevicePID representing the PID to request.
 * @return uint64_t The constructed 64-bit CAN request payload.
 */
uint64_t build_can_tx_read_data_request(CANDeviceConfig *device, CANDevicePID *pid);

/**
 * @brief Retrieve a device configuration by its CAN ID.
 *
 * @param stdid The CAN ID to search for.
 * @return CANDeviceConfig* Pointer to the matching device configuration, or NULL if not found.
 */
CANDeviceConfig *get_device_config_by_id(uint32_t stdid);

/**
 * @brief Retrieve a PID configuration from a device by its PID.
 *
 * @param module Pointer to the CANDeviceConfig containing the PIDs.
 * @param pid The PID to search for.
 * @return CANDevicePID* Pointer to the matching PID configuration, or NULL if not found.
 */
CANDevicePID *get_pid_by_id(CANDeviceConfig *module, uint16_t pid);

/**
 * @brief Retrieve the CAN instance from a hardware instance pointer.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return CANInstance Enum value representing the CAN instance.
 */
CANInstance get_can_instance_from_hcan(CAN_HandleTypeDef *hcan);

/**
 * @brief Retrieve the CAN handle for the given CAN instance.
 *
 * @param instance The CANInstance to retrieve the handle for.
 * @return CAN_HandleTypeDef* Pointer to the corresponding CAN hardware handle, or NULL if invalid.
 */
CAN_HandleTypeDef *get_hcan_from_instance(CANInstance instance);

/**
 * @brief Determine if the CAN message should be ignored.
 *
 * Filters out messages like heartbeat signals or other non-essential data.
 *
 * @param can_id The CAN ID of the received message.
 * @return true if the message should be ignored, false otherwise.
 */
bool should_ignore_message(uint32_t can_id);

/**
 * @brief Retrieve the queue type associated with a CAN instance and flow direction.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param queue_type The flow direction (e.g., QUEUE_TYPE_FLOW_TX or QUEUE_TYPE_FLOW_RX).
 * @return Circular_Queue_Types The corresponding queue type.
 */
Circular_Queue_Types get_queue_num_by_can_instance(CANInstance can_instance, Queue_Type_Flow queue_type);

/**
 * @brief Retrieve the queue handle associated with a CAN instance and flow direction.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param flow_dir The flow direction (e.g., QUEUE_TYPE_FLOW_TX or QUEUE_TYPE_FLOW_RX).
 * @return osMessageQueueId_t The queue handle, or NULL if invalid.
 */
osMessageQueueId_t get_queue_handle_by_can_instance(CANInstance can_instance, Queue_Type_Flow flow_dir);

/**
 * @brief Retrieve the queue handle associated with a specific queue type.
 *
 * @param queue_num The circular queue type.
 * @return osMessageQueueId_t The queue handle.
 */
osMessageQueueId_t get_queue_handle_by_queue_num(Circular_Queue_Types queue_num);

#ifdef __cplusplus
}
#endif

#endif /* CAN_HELPER_H_ */
