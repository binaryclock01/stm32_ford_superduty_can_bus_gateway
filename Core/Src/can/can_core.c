/*
 * can.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 *
 * Description:
 * This file contains functions for handling CAN communication, including
 * building and sending requests, processing received messages, and managing
 * device and PID configurations.
 */

#include <string.h>          // For memcpy
#include <stdio.h>           // For printf
#include <stdint.h>          // For fixed-width integer types
#include <stdbool.h>         // For boolean support
#include <inttypes.h> // Include for portable format specifiers
#include <cmsis_os.h>        // RTOS CMSIS types, such as osMutedId_t

#include "config.h"
#include "buffers.h"
#include "can_core.h"

#include "main.h"            // For HAL_CAN and other core includes
#include "log.h"
#include "error.h"
#include "ansi.h"
#include "can_tx.h"
#include "utils.h"

/* -----------------------------------------------------------------------------
   Function Definitions
   -------------------------------------------------------------------------- */

// Create an array of CAN handles
CAN_HandleTypeDef *can_handles[] = {
#ifdef USE_CAN_1
    &hcan1, // Index 0 for CAN1
#endif

#ifdef USE_CAN_2
    &hcan2,  // Index 1 for CAN2
#endif
};


/**
 * @brief Starts a specific CAN bus and logs the result.
 *
 * If starting the CAN bus fails, it handles the failure using the error handler.
 *
 * @param hcan Pointer to the CAN_HandleTypeDef for the CAN bus to start.
 * @param hcan_human_string Descriptive name of the CAN bus for logging purposes.
 */
void start_hal_can_bus(CAN_HandleTypeDef *hcan, const char *hcan_human_string) {
    // Attempt to start the CAN bus
    if (HAL_CAN_Start(hcan) == HAL_OK) {
        // Log success
        log_status_message(hcan_human_string, true);
    } else {
        // Log failure and handle the error
        log_status_message(hcan_human_string, false);
        user_error_handler(ERROR_CAN_INIT_FAILED, "Failed to start CAN bus");
    }
}

/**
 * @brief Starts all CAN buses and logs the process.
 *
 * Initializes all required CAN bus networks and logs their status.
 */
void start_hal_can_buses(void) {
    log_message("* Initializing CAN BUS Networks");

#ifdef USE_CAN_1
    start_hal_can_bus(&hcan1, "  - " HWHT "TruckNet " CRESET "[" HWHT "CAN1" CRESET "]");
#endif

#ifdef USE_CAN_2
    start_hal_can_bus(&hcan2, "  - " HWHT "AuxNet " CRESET "[" HWHT "CAN2" CRESET "]");
#endif
}

/**
 * @brief Send CAN requests for all devices and their PIDs.
 *
 * Iterates through all configured devices and PIDs and sends a CAN request for each.
 * This function is typically used to initialize communication or request status
 * from all known devices on the CAN network.
 */
void send_all_requests(void) {
    for (uint8_t device_index = 0; device_index < CAN_DEVICE_COUNT; device_index++) {
        CANDeviceConfig *device = &can_devices[device_index];

        for (size_t pid_index = 0; pid_index < device->pid_count; pid_index++) {
            CANDevicePID *pid = &device->pids[pid_index];
            send_can_packet_to_tx_queue(CAN_TRUCK, device, pid, CAN_VERB_REQUEST);

            //osDelay(1000);
        }
    }
//    log_queue_packet_counts();
//    log_circular_buffer_usage();
}

bool is_signal_on(const CANSignal *signal) {
    // Validate input
    if (signal == NULL) {
        return false; // Signal is invalid
    }

    // Get the relevant mask based on relevant_data_bytes
    uint32_t mask = get_relevant_mask(signal->relevant_data_bytes);

    // Apply the mask to both the signal data and the state_on mask
    uint32_t masked_data = signal->data & mask;
    uint32_t masked_state_on = *((uint32_t *)signal->state_on) & mask;

    // Check if the masked data matches the masked state_on
    return masked_data == masked_state_on;
}

bool does_payload_turn_signal_on(const CANSignal *signal, uint32_t payload) {
    uint32_t mask = get_relevant_mask(signal->relevant_data_bytes);

    // Mask both the payload and the state_on signal
    uint32_t masked_payload = payload & mask;
    uint32_t masked_state_on = *((uint32_t *)signal->state_on) & mask;

    // Compare masked values
    return (masked_payload == masked_state_on);
}


/**
 * @brief Process signal changes for a given PID and payload.
 *
 * Iterates through the signals within the specified PID and checks for changes
 * based on the provided payload. Updates the signal state if a change is detected.
 *
 * @param device_pid Pointer to the `CANDevicePID` containing the signals.
 * @param payload The received payload data to compare against.
 */
void process_signal_changes(CANDevicePID *device_pid, uint32_t payload) {
    for (uint8_t signal_idx = 0; signal_idx < device_pid->num_of_signals; signal_idx++) {
        CANSignal *signal = &device_pid->signals[signal_idx];

        switch (signal->change_type) {
            case STATE_BIT:
            	signal->data = (does_payload_turn_signal_on(signal, payload) ? 1 : 0);
                break;
            case STATE_BYTE:
                // TODO: Implement STATE_BYTE condition
                break;
            case STATE_MULTIBYTE:
                // TODO: Implement STATE_MULTIBYTE condition
                break;
            default:
                break;
        }
    }
}

/**
 * @brief Parse raw CAN data into a structured format.
 *
 * This function extracts fields such as data length, PID, and payload
 * from a raw 8-byte CAN data array and populates a Parsed_CAN_Data structure.
 * It performs basic validation on the input pointers and reports errors
 * using the user error handler if necessary.
 *
 * @param raw_data Pointer to the raw CAN data array (8 bytes).
 *                 This data represents a single CAN message payload.
 * @param parsed_data Pointer to the Parsed_CAN_Data structure where the
 *                    parsed fields will be stored.
 * @return true if parsing was successful, false if input pointers were invalid.
 */
bool parse_raw_can_data(const uint8_t *raw_data, Parsed_CAN_Data *parsed_data) {
    // Validate input pointers to prevent null pointer dereference
    if (!raw_data || !parsed_data) {
        // Report the error and return failure
        user_error_handler(ERROR_CAN_DATA_PARSE_FAILED, "Invalid raw data or parsed data pointer");
        return false;
    }

    // Combine the raw bytes into a single 64-bit integer for easier parsing
    uint64_t raw_combined = 0;
    bytes_to_big_endian(&raw_combined, raw_data, 8);

    // Extract the data length (highest byte)
    parsed_data->data_length = (raw_combined >> 56) & 0xFF;

    parsed_data->command = (raw_combined >> 48) & 0xFF;
    // Extract the PID (next 2 bytes)
    parsed_data->pid = (uint16_t)((raw_combined >> 32) & 0xFFFF);

    // Extract the payload (lowest 4 bytes)
    parsed_data->payload = (uint32_t)(raw_combined & 0xFFFFFFFF);

    // Return success
    return true;
}


/**
 * @brief Validate the parsed CAN data fields.
 *
 * Ensures the data length and other fields are within acceptable ranges.
 *
 * @param parsed_data Pointer to the parsed CAN data structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const Parsed_CAN_Data *parsed_data) {
    if (parsed_data->data_length < 4 || parsed_data->data_length > 8) {
        user_error_handler(ERROR_CAN_INVALID_PAYLOAD, "Invalid data length");
        return false;
    }
    return true;
}
