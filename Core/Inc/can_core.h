/*
 * can_core.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This file declares functions and data structures for managing CAN communication,
 * including request building, message handling, and signal processing.
 */

#ifndef CAN_CORE_H_
#define CAN_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>             // Fixed-width integer types
#include <stdbool.h>            // Boolean support
#include "device_configs.h"     // CANDeviceConfig, CANDevicePID, etc.
#include "main.h"               // HAL_CAN definitions
#include "ui.h"                 // Console message utilities
#include "error.h"              // Error handling utilities
#include "utils.h"
#include "cmsis_os.h"        // RTOS CMSIS types, such as osMutedId_t
#include "rtos.h"
#include "can_common.h"

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */
#define DLC_MAX 8 /**< Maximum data length for standard CAN frames */

#define MAX_RETRIES 3        // Maximum retries for CAN message transmission

/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

// Check can_common.h

/* -----------------------------------------------------------------------------
   Structures
   -------------------------------------------------------------------------- */

// Check can_common.h

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup Functions --- */

bool __rtos__process_tx_queue_and_send_to_can1(CAN_Packet *packet);

void create_can_tx_header(CAN_TxHeaderTypeDef *tx_header, uint32_t std_id);

/**
 * @brief Generate a CAN payload for the specified device and PID.
 *
 * Constructs the CAN payload based on the target device and PID configuration.
 *
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid Pointer to the CANDevicePID structure.
 * @param TxData Pointer to the buffer where the payload will be stored.
 */
void generate_can_tx_read_data_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData);

/* --- CAN Message Handling Functions --- */

/**
 * @brief Retrieves the CAN instance for the given hardware instance.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return CANInstance Enum value representing the CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 *         Returns CAN_TOTAL (invalid) if the instance is unknown.
 */
CANInstance get_can_instance_enum(CAN_HandleTypeDef *hcan);

/**
 * @brief Processes a received CAN packet.
 *
 * Handles the processing of a single CAN packet, including logging raw data,
 * checking for ignored messages, parsing, validation, and updating states.
 *
 * @param packet Pointer to the CAN_Packet to be processed.
 */
void process_can_rx_packet(CAN_Packet *packet);

/**
 * @brief Retrieve a CAN message and populate a CAN_Packet for the specified CAN instance.
 *
 * This function attempts to retrieve a message from the CAN FIFO0 buffer and stores
 * the data into a CAN_Packet, which can then be queued or processed further.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to a CAN_Packet where the retrieved data will be stored.
 * @return true if the message was successfully retrieved, false otherwise.
 */
bool get_rx_message_from_CAN_RX_FIFO0(CAN_HandleTypeDef *hcan, CANInstance can_instance, CAN_Packet *packet);

/**
 * @brief Parse and process an incoming CAN message.
 *
 * Handles the complete lifecycle of a received CAN message, including logging,
 * validation, and signal updates.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) receiving the message.
 */
void handle_incoming_can_packet(CANInstance can_instance);

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
bool parse_raw_can_data(const uint8_t *raw_data, Parsed_CAN_Data *parsed_data);

/**
 * @brief Validate the parsed CAN data fields.
 *
 * Ensures the data length and other parsed fields meet expected constraints.
 *
 * @param parsed_data Pointer to the Parsed_CAN_Data structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const Parsed_CAN_Data *parsed_data);

/* --- Device and PID Configuration Functions --- */

/**
 * @brief Retrieve a device configuration by CAN ID.
 *
 * Searches the list of configured devices for a matching CAN ID.
 *
 * @param rx_id The CAN ID to search for.
 * @return Pointer to the matching CANDeviceConfig, or NULL if not found.
 */
CANDeviceConfig *get_device_config_by_id(uint32_t rx_id);

/**
 * @brief Retrieve a PID configuration by PID.
 *
 * Searches the list of PIDs in a device's configuration for a matching PID.
 *
 * @param device Pointer to the CANDeviceConfig containing the PIDs.
 * @param pid The PID to search for.
 * @return Pointer to the matching CANDevicePID, or NULL if not found.
 */
CANDevicePID *get_pid_by_id(CANDeviceConfig *device, uint16_t pid);

/* --- Logging Functions --- */

/**
 * @brief Log the raw incoming CAN message for debugging.
 *
 * Formats and logs the raw CAN ID and data bytes.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 */
void log_raw_can_packet(const CAN_Packet *packet);

/**
 * @brief Log validated CAN data for debugging.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the Parsed_CAN_Data structure.
 */
void log_valid_can_data(const Parsed_CAN_Data *parsed_data);

/* --- Utility Functions --- */

/**
 * @brief Get the CAN instance corresponding to a hardware handle.
 *
 * Maps a HAL CAN handle (e.g., hcan1) to the appropriate CANInstance.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @return The CANInstance (e.g., CAN_TRUCK or CAN_AUX).
 */
CANInstance get_can_instance_from_hcan(CAN_HandleTypeDef *hcan);

void send_can_request_to_tx_queue(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid);

/**
 * @brief Determine if a CAN message should be ignored.
 *
 * Filters out non-critical messages like heartbeats.
 *
 * @param can_id The CAN ID of the message.
 * @return true if the message should be ignored, false otherwise.
 */

/**
 * @brief Send CAN requests for all devices and their PIDs.
 *
 * Iterates through all configured devices and PIDs and sends a CAN request for each.
 * This function is typically used to initialize communication or request status
 * from all known devices on the CAN network.
 */
void send_all_requests(void);

bool should_ignore_message(uint32_t can_id);

extern CAN_HandleTypeDef *can_handles[];

#ifdef __cplusplus
}
#endif

#endif /* CAN_CORE_H_ */
