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

#include "device_configs.h"  // Include configurations for CAN devices
#include <string.h>          // For memcpy
#include <stdio.h>           // For printf
#include <stdint.h>          // For fixed-width integer types
#include <stdbool.h>         // For boolean support
#include <inttypes.h> // Include for portable format specifiers

#include "main.h"            // For HAL_CAN and other core includes
#include "ssd1306.h"         // For OLED display functions
#include "ui.h"              // For console message utilities
#include "utils.h"           // For helper functions like bytes_to_uint32
#include "error.h"           // For error handling utilities
#include "can.h"
#include "cmsis_os.h"        // RTOS CMSIS types, such as osMutedId_t


/* -----------------------------------------------------------------------------
   Function Definitions
   -------------------------------------------------------------------------- */

#define CHAR_BIT 8

static CANPacketPoolEntry can_packet_pool[CAN_PACKET_POOL_SIZE];
static osMutexId_t packet_pool_mutex;  // Mutex to protect pool access

static CANBuffer can_buffers[CAN_TOTAL] = {
    {.head = 0, .tail = 0, .count = 0}, // CAN_TRUCK buffer
    {.head = 0, .tail = 0, .count = 0}  // CAN_AUX buffer
};

// Create an array of CAN handles
CAN_HandleTypeDef *can_handles[] = {
    &hcan1, // Index 0 for CAN1
    &hcan2  // Index 1 for CAN2
};

CANData can_data[CAN_TOTAL] = {
	{ .name = "TruckNet", .rx_count = 0, .tx_count = 0 },
    { .name = "AuxNet", .rx_count = 0, .tx_count = 0 }
};

void init_can_packet_pool(void) {
    for (int i = 0; i < CAN_PACKET_POOL_SIZE; i++) {
        can_packet_pool[i].used = 0;
    }
    packet_pool_mutex = osMutexNew(NULL);
}

CANPacket *allocate_can_packet(void) {
    osMutexAcquire(packet_pool_mutex, osWaitForever);
    for (int i = 0; i < CAN_PACKET_POOL_SIZE; i++) {
        if (can_packet_pool[i].used == 0) {
            can_packet_pool[i].used = 1;
            osMutexRelease(packet_pool_mutex);
            return &can_packet_pool[i].packet;
        }
    }
    osMutexRelease(packet_pool_mutex);
    return NULL;  // Pool is full
}

void free_can_packet(CANPacket *packet) {
    osMutexAcquire(packet_pool_mutex, osWaitForever);
    for (int i = 0; i < CAN_PACKET_POOL_SIZE; i++) {
        if (&can_packet_pool[i].packet == packet) {
            can_packet_pool[i].used = 0;
            break;
        }
    }
    osMutexRelease(packet_pool_mutex);
}

/**
 * @brief Get the message queue handle for a given CAN hardware instance.
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t* get_queue_handle_from_hcan(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        return &CAN1_Rx_QueueHandle;
    } else if (hcan == &hcan2) {
        return &CAN2_Rx_QueueHandle;
    } else {
        return NULL; // Unknown CAN instance
    }
}

bool enqueue_can_packet(CANInstance can_instance, const CANPacket *packet) {
    CANBuffer *buffer = &can_buffers[can_instance];

    if (buffer->count >= CAN_BUFFER_SIZE) {
        // Buffer overflow; handle error or discard packet
        return false;
    }

    buffer->packets[buffer->head] = *packet; // Add packet to the buffer
    buffer->head = (buffer->head + 1) % CAN_BUFFER_SIZE;
    buffer->count++;
    return true;
}

bool dequeue_can_packet(CANInstance can_instance, CANPacket *packet) {
    CANBuffer *buffer = &can_buffers[can_instance];

    if (buffer->count == 0) {
        // Buffer is empty
        return false;
    }

    *packet = buffer->packets[buffer->tail]; // Retrieve the next packet
    buffer->tail = (buffer->tail + 1) % CAN_BUFFER_SIZE;
    buffer->count--;
    return true;
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
uint64_t build_can_request(CANDeviceConfig *device, CANDevicePID *pid) {
    uint64_t uint64_payload = 0;

    // Validate inputs
    if (!device || !pid) {
        return 0;  // Return an empty payload on invalid inputs
    }

    // Payload size: Length (1 byte) + Command (1 byte) + PID (2 bytes)
    uint8_t request_size_in_bytes = 0x03;

    // Command byte (e.g., Read command 0x22)
    uint8_t command_byte = can_request_commands[CMD_READ].byte;

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
 * @brief Setup the CAN Tx header with the given request ID.
 *
 * Initializes the CAN Tx header structure with standard values, including the
 * request ID, identifier type, frame type, and data length.
 *
 * @param TxHeader Pointer to the CAN_TxHeaderTypeDef structure to configure.
 * @param request_id The request ID to be set in the header (Standard Identifier).
 */
void setup_can_tx_header(CAN_TxHeaderTypeDef *TxHeader, uint64_t request_id) {
    *TxHeader = (CAN_TxHeaderTypeDef){
        .StdId = request_id,                 // Set the standard identifier for the CAN message
        .IDE = CAN_ID_STD,                   // Use standard identifier (11-bit)
        .RTR = CAN_RTR_DATA,                 // Specify a data frame (not a remote frame)
        .DLC = MAX_DLC_BYTE_LENGTH,          // Set the Data Length Code to the maximum allowed
        .TransmitGlobalTime = DISABLE        // Disable global time transmission
    };
}

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
void generate_can_tx_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData) {
    uint64_t request_payload = build_can_request(device, pid); // Build the payload using custom logic
    request_payload = __builtin_bswap64(request_payload);      // Convert to big-endian if required
    memcpy(TxData, &request_payload, MAX_DLC_BYTE_LENGTH);     // Copy payload into TxData buffer
}

/**
 * @brief Debug and log the transmitted CAN message.
 *
 * Formats the CAN message request ID and data bytes into a human-readable string
 * and sends it to the console for debugging purposes.
 *
 * @param request_id The request ID associated with the CAN message.
 * @param TxData Pointer to the data buffer containing the CAN message payload.
 * @param dlc The Data Length Code specifying the number of data bytes transmitted.
 */
void log_can_message(uint64_t request_id, uint8_t *TxData, uint8_t dlc) {
    char buf[100 + dlc * 2]; // Allocate enough space for the formatted message
    int offset = snprintf(buf, sizeof(buf), "Tx %X/", (unsigned int)request_id); // Write request ID

    // Append each data byte as a two-character hexadecimal value
    for (uint8_t j = 0; j < dlc; j++) {
        offset += snprintf(&buf[offset], sizeof(buf) - offset, "%02X", TxData[j]);
    }
    send_console_msg(buf); // Send the formatted message to the console
}


/**
 * @brief Send a CAN data packet. Typically called by a higher level function.
 *
 * THIS IS A LOWER LEVEL FUNCTION DENOTED BY _function
 *
 * Typically you want to use the function "send_can_request" if you are trying to send a CAN
 * request as that uses the internal can device and PIDs to build the packet correctly.
 *
 * This function sends a CAN message using the specified CAN data structure, which includes
 * the Tx header, data buffer, and mailbox. Handles errors by invoking the
 * error handler and logging an error message.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @param can_data Pointer to the CANData structure containing transmission details.
 * @return true if the message was transmitted successfully, false otherwise.
 */
bool _send_can_packet(CAN_HandleTypeDef *hcan, CANData *can_data) {
    if (HAL_CAN_AddTxMessage(hcan, &can_data->TxHeader, can_data->TxData, &can_data->TxMailbox) != HAL_OK) {
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, "Failed to send CAN packet");
        return false;
    }
    return true;
}
/**
 * @brief Prepare and send a CAN request for a specific device and PID.
 *
 * This function handles the entire process of preparing and sending a CAN message.
 * It retrieves the request ID, configures the CAN header, generates the payload,
 * transmits the message, and logs the operation for debugging purposes.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2).
 * @param device Pointer to the CANDeviceConfig representing the target device.
 * @param pid Pointer to the CANDevicePID specifying the requested PID.
 */
void send_can_request(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid) {
    // Step 1: Retrieve the request ID
    // The request ID uniquely identifies the CAN message for the target device.
    // This is typically derived from the device's CAN ID.
    uint64_t request_id = get_request_id(device->can_id);

    // Step 2: Access the CAN data and hardware handle
    // Retrieve the CANData structure (e.g., TxHeader, TxData) for the specified instance.
    // Also, select the appropriate hardware handle (e.g., hcan1, hcan2) for transmission.
    CANData *selected_can = &can_data[can_instance];
    CAN_HandleTypeDef *selected_hcan_instance = can_handles[can_instance];

    // Step 3: Configure the CAN Tx header
    // The header defines metadata for the CAN message, including the request ID,
    // identifier type, and data length code (DLC).
    setup_can_tx_header(&selected_can->TxHeader, request_id);

    // Step 4: Generate the CAN payload
    // The payload is constructed based on the target device and the requested PID.
    // This includes encoding the payload in a format suitable for transmission.
    generate_can_tx_payload(device, pid, selected_can->TxData);

    // Step 5: Transmit the CAN message
    // Use the hardware CAN handle and the configured CANData structure to send the message.
    // If transmission fails, log an error and exit the function.
    if (!_send_can_packet(selected_hcan_instance, selected_can)) {
        return; // Exit on transmission failure
    }

    // Step 6: Update transmission statistics
    // Increment the transmission counter for the selected CAN instance to track activity.
    selected_can->tx_count++;

    // Step 7: Log the transmitted message
    // Generate a human-readable log of the transmitted message, including the request ID
    // and the raw payload data, for debugging purposes.
    log_can_message(request_id, selected_can->TxData, selected_can->TxHeader.DLC);
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
            send_can_request(CAN_TRUCK, device, pid);
        }
    }
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
CANDeviceConfig *get_device_config_by_id(uint32_t rx_id) {
    for (uint8_t module_idx = 0; module_idx < CAN_DEVICE_COUNT; module_idx++) {
        CANDeviceConfig *module_config = &can_devices[module_idx];
        if (module_config->can_id == rx_id) {
            return module_config;
        }
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
                signal->data = (payload & bytes_to_uint32((uint8_t *)signal->change_data)) ? 1 : 0;
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
 * @brief Retrieve a CAN message and populate a CANPacket for the specified CAN instance.
 *
 * This function attempts to retrieve a message from the CAN FIFO0 buffer and stores
 * the data into a CANPacket, which can then be queued or processed further.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to a CANPacket where the retrieved data will be stored.
 * @return true if the message was successfully retrieved, false otherwise.
 */
bool retrieve_can_message(CAN_HandleTypeDef *hcan, CANInstance can_instance, CANPacket *packet) {
    if (packet == NULL) {
        // Log and return error if the provided packet pointer is NULL
        user_error_handler(ERROR_CAN_PACKET_NULL, "CANPacket pointer is NULL");
        return false;
    }

    // Attempt to retrieve the CAN message from FIFO0
    HAL_StatusTypeDef hal_status = HAL_CAN_GetRxMessage(
        hcan, CAN_RX_FIFO0, &packet->header, packet->data
    );

    if (hal_status != HAL_OK) {
        // Log an error if message retrieval fails
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, "Failed to retrieve CAN message");
        return false; // Indicate failure
    }

    // Populate additional packet fields
    packet->can_instance = can_instance;
    packet->timestamp = HAL_GetTick(); // Add a timestamp for when the packet was received

    return true; // Indicate success
}

/**
 * @brief Log the raw incoming CAN message for debugging purposes.
 *
 * Logs the CAN ID and raw data bytes in a human-readable format.
 *
 * @param can_number The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 */

void log_raw_can_data(CANInstance can_instance) {
    char log_msg[128] = {0};

    snprintf(log_msg, sizeof(log_msg), "Raw CAN%d: ID=0x%" PRIX32 " Data=",
             can_instance, can_data[can_instance].RxHeader.StdId);

    // Append data bytes to the message
    size_t offset = strlen(log_msg);
    for (uint8_t i = 0; i < can_data[can_instance].RxHeader.DLC; i++) {
        snprintf(&log_msg[offset], sizeof(log_msg) - offset, "%02X ",
                 can_data[can_instance].RxData[i]);
        offset = strlen(log_msg);
    }

    // Log the message
    send_console_msg(log_msg);
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

/**
 * @brief Parse raw CAN data into a structured format.
 *
 * This function extracts fields such as data length, PID, and payload
 * from a raw 8-byte CAN data array and populates a ParsedCANData structure.
 * It performs basic validation on the input pointers and reports errors
 * using the user error handler if necessary.
 *
 * @param raw_data Pointer to the raw CAN data array (8 bytes).
 *                 This data represents a single CAN message payload.
 * @param parsed_data Pointer to the ParsedCANData structure where the
 *                    parsed fields will be stored.
 * @return true if parsing was successful, false if input pointers were invalid.
 */
bool parse_raw_can_data(const uint8_t *raw_data, ParsedCANData *parsed_data) {
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
bool validate_parsed_can_data(const ParsedCANData *parsed_data) {
    if (parsed_data->data_length < 4 || parsed_data->data_length > 8) {
        user_error_handler(ERROR_CAN_INVALID_PAYLOAD, "Invalid data length");
        return false;
    }
    return true;
}

/**
 * @brief Log validated CAN data for debugging purposes.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the parsed CAN data structure.
 */
void log_valid_can_data(const ParsedCANData *parsed_data) {
    char log_msg[128] = {0};

    // Use PRIX32 for portable handling of uint32_t in hexadecimal
    snprintf(log_msg, sizeof(log_msg),
             "Valid CAN Data: PID=0x%04" PRIX16 " Payload=0x%08" PRIX32,
             parsed_data->pid, parsed_data->payload);

    // Log the message
    send_console_msg(log_msg);
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
 * @brief Process a received CAN packet.
 *
 * This function handles the following steps for processing a received CAN packet:
 * 1. Logs the raw data for debugging purposes.
 * 2. Checks if the packet should be ignored (e.g., heartbeat or irrelevant messages).
 * 3. Parses the raw CAN data into a structured format.
 * 4. Validates the parsed data to ensure it meets protocol requirements.
 * 5. Updates relevant signal states based on the parsed data.
 * 6. Logs valid packet data for further debugging or record-keeping.
 *
 * @param packet Pointer to the CANPacket structure containing the received data.
 */
void process_can_packet(CANPacket *packet) {
    // Step 1: Validate the input
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Received null CANPacket");
        return; // Exit early to prevent further processing
    }

    // Step 2: Log the raw CAN data for debugging
    log_raw_can_data(packet->can_instance);

    // Step 3: Check if the CAN message should be ignored
    if (should_ignore_message(packet->header.StdId)) {
        return; // Exit early if the message is not relevant
    }

    // Step 4: Parse the raw CAN data into a structured format
    ParsedCANData parsed_data;
    parse_raw_can_data(packet->data, &parsed_data);

    // Step 5: Validate the parsed data
    if (!validate_parsed_can_data(&parsed_data)) {
        user_error_handler(ERROR_CAN_DATA_PARSE_FAILED, "Invalid CAN data");
        return; // Exit early if the data is invalid
    }

    // Step 6: Locate the device configuration for the CAN ID
    CANDeviceConfig *device_config = get_device_config_by_id(packet->header.StdId);
    if (device_config == NULL) {
        user_error_handler(ERROR_CAN_DEVICE_NOT_FOUND, "Device configuration not found for CAN ID");
        return; // Exit if no matching device configuration is found
    }

    // Step 7: Locate the PID configuration for the parsed PID
    CANDevicePID *pid_config = get_pid_by_id(device_config, parsed_data.pid);
    if (pid_config == NULL) {
        user_error_handler(ERROR_CAN_PID_NOT_FOUND, "PID configuration not found for device");
        return; // Exit if no matching PID configuration is found
    }

    // Step 8: Update the signal states based on the parsed data
    process_signal_changes(pid_config, parsed_data.payload);

    // Step 9: Log the validated and processed CAN data for debugging
    log_valid_can_data(&parsed_data);
}



/**
 * @brief Handle and process an incoming CAN message.
 *
 * This function is the main entry point for processing a received CAN message.
 * It performs the following steps:
 * 1. Logs the raw incoming data for debugging.
 * 2. Checks whether the message should be ignored (e.g., heartbeats).
 * 3. Finds the appropriate device configuration for the CAN ID.
 * 4. Parses the raw data into a structured format.
 * 5. Validates the parsed data for correctness.
 * 6. Logs validated data for further debugging or record-keeping.
 * 7. Updates signal states based on the extracted payload.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) where the message was received.
 */
/*
void old_handle_incoming_can_packet(CANInstance can_instance) {
    // Step 1: Set up the selected CAN data
    // Retrieve the CANData structure for the given instance.
    // This holds relevant information like the RxHeader and RxData buffers.
    CANData *selected_can = &can_data[can_instance];

    // Extract the CAN ID from the RxHeader for further processing.
    uint32_t can_id = selected_can->RxHeader.StdId;

    // Step 2: Log raw data
    // Log the raw CAN ID and data bytes in a human-readable format for debugging purposes.
    log_raw_can_data(can_instance);

    // Step 3: Check if the message should be ignored
    // Certain messages, such as heartbeats or non-critical updates, may be skipped.
    if (should_ignore_message(can_id)) {
        return; // Exit early if the message is not relevant.
    }

    // Step 4: Locate the device configuration
    // Find the corresponding device configuration for the CAN ID.
    // If the device is unknown, log an error and terminate further processing.
    CANDeviceConfig *selected_device = get_device_config_by_id(can_id);
    if (!selected_device) {
        user_error_handler(ERROR_CAN_MODULE_NOT_FOUND, "Unknown CAN ID");
        return;
    }

    // Step 5: Parse raw data into a usable format
    // Convert the raw bytes into structured fields: data length, PID, and payload.
    ParsedCANData parsed_data;
    parse_raw_can_data(selected_can->RxData, &parsed_data);

    // Step 6: Validate the parsed data
    // Check if the parsed data adheres to the expected constraints (e.g., valid data length).
    // If invalid, log an error and stop further processing.
    if (!validate_parsed_can_data(&parsed_data)) {
        return;
    }

    // Step 7: Locate the PID configuration
    // Identify the PID configuration for the extracted PID within the device's context.
    // If the PID is unknown, log an error and terminate further processing.
    CANDevicePID *selected_pid = get_pid_by_id(selected_device, parsed_data.pid);
    if (!selected_pid) {
        user_error_handler(ERROR_MODULE_PID_NOT_FOUND, "Unknown PID");
        return;
    }

    // Step 8: Log valid data for debugging
    // Log the parsed and validated data for debugging or record-keeping purposes.
    log_valid_can_data(&parsed_data);

    // Step 9: Process the signals for the valid PID
    // Update the signal states based on the extracted payload. This may involve
    // toggling switches, updating system states, or triggering further actions.
    process_signal_changes(selected_pid, parsed_data.payload);
}
*/
