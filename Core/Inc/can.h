/*
 * can.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This file declares functions and data structures for managing CAN communication,
 * including request building, message handling, and signal processing.
 */

#ifndef CAN_H_
#define CAN_H_

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

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */
#define DLC_MAX 8 /**< Maximum data length for standard CAN frames */

/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

/**
 * @brief Enum to represent CAN instances for modularity and clarity.
 */
typedef enum {
    CAN_TRUCK, /**< Truck CAN bus (CAN1) */
    CAN_AUX,   /**< Auxiliary CAN bus (CAN2) */
    CAN_TOTAL  /**< Total number of CAN instances */
} CANInstance;

/* -----------------------------------------------------------------------------
   Structures
   -------------------------------------------------------------------------- */

/**
 * @brief Structure to manage CAN data and state.
 */
typedef struct {
    char name[100];                      /**< Name of the CAN instance (e.g., "TruckNet"). */
    CAN_TxHeaderTypeDef TxHeader;        /**< CAN transmission header. */
    CAN_RxHeaderTypeDef RxHeader;        /**< CAN reception header. */
    uint32_t TxMailbox;                  /**< CAN transmission mailbox identifier. */
    uint8_t RxData[DLC_MAX];             /**< Buffer for received CAN data. */
    uint8_t TxData[DLC_MAX];             /**< Buffer for transmitted CAN data. */
    uint32_t tx_count;                   /**< Counter for transmitted messages. */
    uint32_t rx_count;                   /**< Counter for received messages. */
} CANData;

/**
 * @brief Structure to hold parsed CAN data.
 */
typedef struct {
    uint8_t data_length; /**< Length of the received CAN data. */
    uint16_t pid;        /**< Parsed PID from the CAN message. */
    uint32_t payload;    /**< Extracted payload from the CAN message. */
} ParsedCANData;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup Functions --- */

/**
 * @brief Setup the CAN Tx header with the given request ID.
 *
 * Initializes the CAN Tx header with standard values like ID, identifier type,
 * frame type, and data length.
 *
 * @param TxHeader Pointer to the CAN_TxHeaderTypeDef structure.
 * @param request_id The request ID to set in the header.
 */
void setup_can_tx_header(CAN_TxHeaderTypeDef *TxHeader, uint64_t request_id);

/**
 * @brief Generate a CAN payload for the specified device and PID.
 *
 * Constructs the CAN payload based on the target device and PID configuration.
 *
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid Pointer to the CANDevicePID structure.
 * @param TxData Pointer to the buffer where the payload will be stored.
 */
void generate_can_tx_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData);

/* --- CAN Message Handling Functions --- */

/**
 * @brief Retrieve a CAN message from FIFO0 for the specified instance.
 *
 * Fetches a message from FIFO0 and stores it in the `can_data` structure.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @return true if the message was successfully retrieved, false otherwise.
 */
bool retrieve_can_message(CAN_HandleTypeDef *hcan, CANInstance can_instance);

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
 * @brief Parse raw CAN data into structured fields.
 *
 * Extracts data length, PID, and payload from raw CAN data.
 *
 * @param raw_data Pointer to the raw CAN data array.
 * @param parsed_data Pointer to the ParsedCANData structure to populate.
 */
void parse_raw_can_data(const uint8_t *raw_data, ParsedCANData *parsed_data);

/**
 * @brief Validate the parsed CAN data fields.
 *
 * Ensures the data length and other parsed fields meet expected constraints.
 *
 * @param parsed_data Pointer to the ParsedCANData structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const ParsedCANData *parsed_data);

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
void log_raw_can_data(CANInstance can_instance);

/**
 * @brief Log validated CAN data for debugging.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the ParsedCANData structure.
 */
void log_valid_can_data(const ParsedCANData *parsed_data);

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

extern CANData can_data[CAN_TOTAL];

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */
