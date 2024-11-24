/*
 * can_core.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares functions and data structures for managing CAN communication,
 * including initialization, message handling, and device-specific configurations.
 */

#ifndef CAN_CORE_H_
#define CAN_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>             // Fixed-width integer types
#include <stdbool.h>            // Boolean support

#include "config.h"
#include "device_configs.h"     // CANDeviceConfig, CANDevicePID, etc.
#include "can_common.h"         // Common CAN utilities
#include "rtos.h"               // RTOS utilities
#include "config.h"

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup Functions --- */

/**
 * @brief Initialize and start all CAN buses.
 */
void start_hal_can_buses(void);

/**
 * @brief Start a specific CAN bus and log the result.
 *
 * @param hcan Pointer to the CAN_HandleTypeDef for the CAN bus.
 * @param hcan_human_string Human-readable description of the CAN bus.
 */
void start_hal_can_bus(CAN_HandleTypeDef *hcan, const char *hcan_human_string);

/* --- CAN Packet Processing Functions --- */

/**
 * @brief Process signal changes for a given PID and payload.
 *
 * Updates the state of signals within the specified PID based on the payload.
 *
 * @param device_pid Pointer to the CANDevicePID containing signals.
 * @param payload The received payload data to compare against.
 */
void process_signal_changes(CANDevicePID *device_pid, uint32_t payload);

/**
 * @brief Parse raw CAN data into a structured format.
 *
 * Extracts fields such as data length, PID, and payload from raw CAN data.
 *
 * @param raw_data Pointer to the raw CAN data array (8 bytes).
 * @param parsed_data Pointer to the Parsed_CAN_Data structure for storing results.
 * @return true if parsing was successful, false otherwise.
 */
bool parse_raw_can_data(const uint8_t *raw_data, Parsed_CAN_Data *parsed_data);

/**
 * @brief Validate the parsed CAN data fields.
 *
 * Ensures the parsed data meets protocol requirements.
 *
 * @param parsed_data Pointer to the parsed CAN data structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const Parsed_CAN_Data *parsed_data);

/* --- Device and Signal Configuration Functions --- */

/**
 * @brief Determine if a signal is in the "on" state.
 *
 * Checks if the signal's current data matches its "on" state.
 *
 * @param signal Pointer to the CANSignal structure.
 * @return true if the signal is "on," false otherwise.
 */
bool is_signal_on(const CANSignal *signal);

/**
 * @brief Check if a payload turns a signal "on."
 *
 * Compares the payload to the signal's "on" state.
 *
 * @param signal Pointer to the CANSignal structure.
 * @param payload The payload data.
 * @return true if the payload turns the signal "on," false otherwise.
 */
bool does_payload_turn_signal_on(const CANSignal *signal, uint32_t payload);

/**
 * @brief Send CAN requests for all configured devices and PIDs.
 *
 * Iterates through all devices and PIDs to send requests.
 */
void send_all_requests(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CORE_H_ */
