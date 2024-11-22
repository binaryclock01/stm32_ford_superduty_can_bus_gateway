/*
 * can_core.c
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This file contains functions for managing CAN communication, including
 * initialization, message handling, request building, and logging.
 */

#include "can_core.h"
#include "device_configs.h"   // CAN device configurations
#include <string.h>           // For memcpy
#include <stdio.h>            // For snprintf
#include <stdint.h>           // Fixed-width integer types
#include <stdbool.h>          // Boolean support
#include <inttypes.h>         // Portable format specifiers

#include "main.h"             // HAL and core includes
#include "rtos.h"             // RTOS support
#include "error.h"            // Error handling utilities
#include "ansi.h"             // ANSI color macros
#include "log.h"              // Logging system

/* --------------------------------------------------------------------------
   Global Variables
   -------------------------------------------------------------------------- */

// Array of CAN handles
CAN_HandleTypeDef *can_handles[] = {
    &hcan1, // Index 0 for CAN1
    &hcan2  // Index 1 for CAN2
};

/* --------------------------------------------------------------------------
   Initialization Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Starts a specific CAN bus and logs the result.
 *
 * If starting the CAN bus fails, it handles the failure using the error handler.
 *
 * @param hcan Pointer to the CAN_HandleTypeDef for the CAN bus to start.
 * @param hcan_human_string Descriptive name of the CAN bus for logging purposes.
 */
void start_hal_can_bus(CAN_HandleTypeDef *hcan, const char *hcan_human_string) {
    if (HAL_CAN_Start(hcan) == HAL_OK) {
        log_status_message(hcan_human_string, true); // Log success
    } else {
        log_status_message(hcan_human_string, false); // Log failure
        user_error_handler(ERROR_CAN_INIT_FAILED, "Failed to start CAN bus");
    }
}

/**
 * @brief Initializes all CAN buses and logs the process.
 */
void start_hal_can_buses(void) {
    log_message(HGRN "* Initializing CAN BUS Networks" CRESET);
    start_hal_can_bus(&hcan1, "TruckNet [CAN1]");
    start_hal_can_bus(&hcan2, "AuxNet [CAN2]");
}

/* --------------------------------------------------------------------------
   CAN Instance and Handle Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Retrieve the CAN handle for a given instance.
 *
 * Maps a `CANInstance` enum to the corresponding `CAN_HandleTypeDef`.
 *
 * @param instance CANInstance enum value (e.g., `CAN_TRUCK`, `CAN_AUX`).
 * @return Pointer to the corresponding CAN handle, or NULL if invalid.
 */
CAN_HandleTypeDef *get_hcan_from_instance(CANInstance instance) {
    switch (instance) {
        case CAN_TRUCK: return &hcan1;
        case CAN_AUX: return &hcan2;
        default:
            user_error_handler(ERROR_CAN_INVALID_CONTEXT, "Invalid CAN instance");
            return NULL;
    }
}

/**
 * @brief Retrieves the CAN instance for a given CAN handle.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @return Corresponding CANInstance enum value, or `CAN_TOTAL` if unknown.
 */
CANInstance get_can_instance_enum(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) return CAN_TRUCK;
    if (hcan == &hcan2) return CAN_AUX;
    return CAN_TOTAL;
}

/* --------------------------------------------------------------------------
   CAN Message Handling Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Processes a received CAN packet.
 *
 * Logs, validates, and updates states based on the packet's content.
 *
 * @param queue_enum Queue type associated with the packet.
 * @param can_instance_enum CAN instance the packet belongs to.
 * @param packet Pointer to the CAN packet to process.
 */
void process_can_rx_packet(Circular_Queue_Types queue_enum, CANInstance can_instance_enum, CAN_Packet *packet) {
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Received null CAN packet");
        return;
    }

    // Log the raw packet
    log_raw_can_packet(packet);

    // Check if the packet should be ignored
    if (should_ignore_message(packet->header.id)) return;

    // Parse and validate the CAN data
    Parsed_CAN_Data parsed_data;
    if (!parse_raw_can_data(packet->payload, &parsed_data) ||
        !validate_parsed_can_data(&parsed_data)) {
        log_message(BRED "Invalid CAN packet received" CRESET);
        return;
    }

    // Log the validated data
    log_valid_can_data(&parsed_data);

    // Determine the device and PID configurations
    CAN_Packet_Flow flow = (can_instance_enum == CAN_TRUCK) ? PACKET_TX : PACKET_RX;
    CANDeviceConfig *device_config = get_device_config_by_id(packet->header.id, flow);
    if (!device_config) {
        user_error_handler(ERROR_CAN_DEVICE_NOT_FOUND, "Device configuration not found");
        return;
    }

    CANDevicePID *pid_config = get_pid_by_id(device_config, parsed_data.pid);
    if (!pid_config) {
        user_error_handler(ERROR_CAN_PID_NOT_FOUND, "PID configuration not found");
        return;
    }

    // Update signals or send replies based on the instance
    if (can_instance_enum == CAN_TRUCK) {
        process_signal_changes(pid_config, parsed_data.payload);
    } else if (can_instance_enum == CAN_AUX) {
        send_can_packet_to_tx_queue(CAN_TRUCK, device_config, pid_config, CAN_VERB_REPLY);
    }
}

/* --------------------------------------------------------------------------
   Utility Functions
   -------------------------------------------------------------------------- */

/**
 * @brief Checks if a CAN message should be ignored.
 *
 * Filters out non-critical messages like heartbeat signals.
 *
 * @param can_id The CAN message ID.
 * @return true if the message should be ignored, false otherwise.
 */
bool should_ignore_message(uint32_t can_id) {
    return can_id == CAN_ID_HEARTBEAT;
}

/**
 * @brief Parses raw CAN data into a structured format.
 *
 * @param raw_data Pointer to the raw CAN data (8 bytes).
 * @param parsed_data Pointer to the parsed data structure.
 * @return true if parsing succeeds, false otherwise.
 */
bool parse_raw_can_data(const uint8_t *raw_data, Parsed_CAN_Data *parsed_data) {
    if (!raw_data || !parsed_data) return false;

    // Combine raw bytes into a single 64-bit integer
    uint64_t raw_combined = 0;
    memcpy(&raw_combined, raw_data, sizeof(raw_combined));

    // Extract fields from the raw data
    parsed_data->data_length = (raw_combined >> 56) & 0xFF;
    parsed_data->pid = (uint16_t)((raw_combined >> 32) & 0xFFFF);
    parsed_data->payload = (uint32_t)(raw_combined & 0xFFFFFFFF);

    return true;
}

/**
 * @brief Validates parsed CAN data for integrity.
 *
 * @param parsed_data Pointer to the parsed data structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const Parsed_CAN_Data *parsed_data) {
    return parsed_data->data_length >= 4 && parsed_data->data_length <= 8;
}

