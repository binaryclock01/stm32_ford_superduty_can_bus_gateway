/*
 * can_helper.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "can_common.h"
#include "error.h"
#include "buffers.h"
#include "can_packet.h"
#include "device_configs.h"


CANSignal *get_pid_signal_by_name(const char *signal_name) {
    // Iterate over all CAN devices
    for (int i = 0; i < CAN_DEVICE_COUNT; i++) {
        CANDeviceConfig *device = &can_devices[i];

        // Iterate over each PID in the device
        for (int j = 0; j < device->pid_count; j++) {
            CANDevicePID *pid = &device->pids[j];

            // Iterate over signals in the PID
            for (int k = 0; k < pid->num_of_signals; k++) {
                CANSignal *signal = &pid->signals[k];

                // Compare the signal name with the given name
                if (strcmp(signal->name, signal_name) == 0) {
                    return signal; // Found the signal, return it
                }
            }
        }
    }

    // Signal not found
    return NULL;
}

bool get_signal_data(const CANSignal *signal)
{
    // Validate input
    if (signal == NULL) {
        return false; // Signal is invalid
    }

    return signal->data;
}

/** "Unknown CAN"
 * @brief Retrieves the CAN instance for the given hardware instance.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return CANInstance Enum value representing the CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 *         Returns CAN_TOTAL (invalid) if the instance is unknown.
 */
CANInstance get_can_instance_enum(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        return CAN_TRUCK;
    } else if (hcan == &hcan2) {
        return CAN_AUX;
    }
    return CAN_TOTAL; // Invalid CAN instance
}

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
uint64_t build_can_tx_read_data_request(CANDeviceConfig *device, CANDevicePID *pid) {
    uint64_t uint64_payload = 0;

    // Validate inputs
    if (!device || !pid) {
        return 0;  // Return an empty payload on invalid inputs
    }

    // Payload size: Length (1 byte) + Command (1 byte) + PID (2 bytes)
    uint8_t request_size_in_bytes = 0x03;

    // Command byte (e.g., Read command 0x22)
    uint8_t command_byte = can_request_commands[REQ_READ].byte;

    // PID in big-endian format
    uint16_t pid_id = __builtin_bswap16(*(uint16_t *)pid->pid_id);

    // Calculate MSB position for an 8-byte payload
    uint8_t starting_msb = ((MAX_DLC_BYTE_LENGTH - 1) * CHAR_BIT);  // CHAR_BIT ensures portability

    // Construct payload
    uint64_payload |= ((uint64_t)request_size_in_bytes) << (starting_msb - (DATA_LENGTH_START * CHAR_BIT));
    uint64_payload |= ((uint64_t)command_byte) << (starting_msb - (DATA_COMMAND_START * CHAR_BIT));
    uint64_payload |= ((uint64_t)pid_id) << (starting_msb - ((DATA_PID_START + 1) * CHAR_BIT));

    return uint64_payload;
}

/**
 * @brief Retrieve a device configuration by its CAN ID.
 *
 * Searches the list of configured devices to find a matching `CANDeviceConfig`
 * based on the provided CAN ID. Returns a pointer to the configuration if found.
 *
 * @param rx_id The CAN ID to search for.
 * @return CANDeviceConfig* Pointer to the matching device configuration, or NULL if not found.
 */
CANDeviceConfig *get_device_config_by_id(uint32_t stdid) {
    for (uint8_t module_idx = 0; module_idx < CAN_DEVICE_COUNT; module_idx++) {
        CANDeviceConfig *module_config = &can_devices[module_idx];
        if ( (module_config->id.response == stdid ) ||
        	 (module_config->id.request == stdid) )
            return module_config;
    }
    return NULL;
}

/**
 * @brief Retrieve a PID configuration from a device by its PID.
 *
 * Searches the list of PIDs in the given `CANDeviceConfig` to find a matching
 * `CANDevicePID` based on the provided PID. Returns a pointer to the PID configuration if found.
 *
 * @param module Pointer to the `CANDeviceConfig` containing the PIDs.
 * @param pid The PID to search for.
 * @return CANDevicePID* Pointer to the matching PID configuration, or NULL if not found.
 */
CANDevicePID *get_pid_by_id(CANDeviceConfig *module, uint16_t pid) {
    for (uint8_t pid_idx = 0; pid_idx < module->pid_count; pid_idx++) {
        CANDevicePID *device_pid = &module->pids[pid_idx];
        uint16_t module_pid = 0;

        // Construct the module PID from its bytes
        for (int i = 0; i < PID_BYTE_LENGTH; i++) {
            module_pid |= ((uint16_t)(((uint8_t)device_pid->pid_id[i]))) << (8 * (1 - i));
        }

        if (module_pid == pid) {
            return device_pid;
        }
    }
    return NULL;
}

// Define a mapping array to map hardware instances to CANInstance indexes
CANInstance get_can_instance_from_hcan(CAN_HandleTypeDef *hcan) {
    static const CAN_HandleTypeDef *can_handles[CAN_TOTAL] = {&hcan1, &hcan2};

    for (uint8_t i = 0; i < CAN_TOTAL; i++) {
        if (hcan == can_handles[i]) {
            return (CANInstance)i;
        }
    }

    // Fallback in case the instance isn't found (shouldn't happen)
    user_error_handler(ERROR_CAN_MODULE_NOT_FOUND, "Unknown CAN instance");
    return CAN_TOTAL;
}

/**
 * @brief Retrieve the CAN handle for the given CAN instance.
 *
 * This function maps a `CANInstance` enumeration value to its corresponding
 * `CAN_HandleTypeDef`. It provides a way to retrieve the appropriate hardware
 * CAN handle based on the logical instance (e.g., `CAN_TRUCK`, `CAN_AUX`).
 *
 * @param instance The `CANInstance` to retrieve the handle for.
 * @return CAN_HandleTypeDef* Pointer to the corresponding CAN hardware handle,
 *         or NULL if the instance is invalid.
 */
CAN_HandleTypeDef *get_hcan_from_instance(CANInstance instance) {
    switch (instance) {
        case CAN_TRUCK:
            return &hcan1; // Replace with actual CAN handle for CAN_TRUCK
        case CAN_AUX:
            return &hcan2; // Replace with actual CAN handle for CAN_AUX
        default:
            // Log an error for an invalid instance
            char error_msg[255];
            snprintf(error_msg, sizeof(error_msg), "%s: Invalid CAN instance: %d", __func__, instance);
            user_error_handler(ERROR_CAN_INVALID_CONTEXT, error_msg);
            return NULL;
    }
}


/**
 * @brief Determine if the CAN message should be ignored.
 *
 * Filters out messages like heartbeat signals or other non-essential data.
 *
 * @param can_id The CAN ID of the received message.
 * @return true if the message should be ignored, false otherwise.
 */
bool should_ignore_message(uint32_t can_id) {
    return (can_id == CAN_ID_HEARTBEAT);
}

Circular_Queue_Types get_queue_num_by_can_instance(CANInstance can_instance, Queue_Type_Flow queue_type)
{
	if (can_instance >= CAN_TOTAL)
		return QUEUE_TYPE_FLOW_UNKNOWN;

	switch (queue_type)
	{
		case QUEUE_TYPE_FLOW_TX:
			if (can_instance == CAN_TRUCK)
				return QUEUE_TX_CAN1;
			else
				return QUEUE_TX_CAN2;
			break;
		case QUEUE_TYPE_FLOW_RX:
			if (can_instance == CAN_TRUCK)
				return QUEUE_RX_CAN1;
			else
				return QUEUE_RX_CAN2;
			break;
			break;
		default:
			return QUEUE_TYPE_FLOW_UNKNOWN;
	}
}


osMessageQueueId_t get_queue_handle_by_can_instance(CANInstance can_instance, Queue_Type_Flow flow_dir)
{
	if (can_instance == CAN_TRUCK)
	{
		if (flow_dir == QUEUE_TYPE_FLOW_RX)
			return &(can_circular_buffer[QUEUE_RX_CAN1].queue_handle);
		else if (flow_dir == QUEUE_TYPE_FLOW_TX)
			return &(can_circular_buffer[QUEUE_TX_CAN1].queue_handle);
	}
	else if (can_instance == CAN_AUX)
	{
		if (flow_dir == QUEUE_TYPE_FLOW_RX)
			return &(can_circular_buffer[QUEUE_RX_CAN2].queue_handle);
		else if (flow_dir == QUEUE_TYPE_FLOW_TX)
			return &(can_circular_buffer[QUEUE_TX_CAN2].queue_handle);
	}

	// if none matched, return NULL pointer
	return NULL;
}

// osMessageQueueId_t is a pointer that holds the queue_handle so don't return pointers to pointers!
osMessageQueueId_t get_queue_handle_by_queue_num(Circular_Queue_Types queue_num)
{
	// osMessageQueueId_t is a pointer that holds the queue_handle so don't return pointers to pointers!
	return can_circular_buffer[queue_num].queue_handle;
}
