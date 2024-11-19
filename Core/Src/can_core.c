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

#include "can_core.h"
#include "device_configs.h"  // Include configurations for CAN devices
#include <string.h>          // For memcpy
#include <stdio.h>           // For printf
#include <stdint.h>          // For fixed-width integer types
#include <stdbool.h>         // For boolean support
#include <inttypes.h> // Include for portable format specifiers

#include "main.h"            // For HAL_CAN and other core includes
#include "ssd1306.h"         // For OLED display functions
//#include "ui.h"              // For console message utilities
#include "utils.h"           // For helper functions like bytes_to_uint32
#include "error.h"           // For error handling utilities
#include <cmsis_os.h>        // RTOS CMSIS types, such as osMutedId_t
#include "rtos.h"

/* -----------------------------------------------------------------------------
   Function Definitions
   -------------------------------------------------------------------------- */

#define CHAR_BIT 8

// Create an array of CAN handles
CAN_HandleTypeDef *can_handles[] = {
    &hcan1, // Index 0 for CAN1
    &hcan2  // Index 1 for CAN2
};

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
CAN_HandleTypeDef *get_can_handle_from_instance(CANInstance instance) {
    switch (instance) {
        case CAN_TRUCK:
            return &hcan1; // Replace with actual CAN handle for CAN_TRUCK
        case CAN_AUX:
            return &hcan2; // Replace with actual CAN handle for CAN_AUX
        default:
            // Log an error for an invalid instance
            char error_msg[64];
            snprintf(error_msg, sizeof(error_msg), "Invalid CAN instance: %d", instance);
            user_error_handler(ERROR_CAN_INVALID_CONTEXT, error_msg);
            return NULL;
    }
}

/**
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
 * @brief Configure the CAN Tx header.
 *
 * Sets up the CAN transmission header with the specified request ID and default values
 * for the identifier type, data length code (DLC), and other metadata.
 *
 * @param tx_header Pointer to the CAN_TxHeaderTypeDef structure to populate.
 * @param std_id The standard identifier for the CAN message.
 */
void create_can_tx_header(CAN_TxHeaderTypeDef *tx_header, uint32_t std_id) {
    if (tx_header == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "NULL pointer provided to create_can_tx_header");
        return;
    }

    *tx_header = (CAN_TxHeaderTypeDef){
        .StdId = std_id,                     // Set the standard identifier for the CAN message
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
void generate_can_tx_read_data_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData) {
    uint64_t request_payload = build_can_tx_read_data_request(device, pid); // Build the payload using custom logic
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
 * @brief Prepare and enqueue a CAN request for a specific device and PID.
 *
 * This function prepares a CAN message, stores it in the circular buffer,
 * and enqueues a reference to it in the transmission queue for processing by the Tx task.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2).
 * @param device Pointer to the CANDeviceConfig representing the target device.
 * @param pid Pointer to the CANDevicePID specifying the requested PID.
 */
void send_can_request_to_tx_queue(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid) {
    // Step 1: Validate inputs
    if (device == NULL || pid == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Device or PID is NULL in send_can_request_to_tx_queue");
        return;
    }

    // Step 2: Acquire mutex for thread-safe access to the circular buffer
    if (osMutexAcquire(can_circular_buffer[QUEUE_TX].mutex_id, osWaitForever) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_TIMEOUT, "Failed to acquire mutex for TX circular buffer");
        return;
    }

    // Step 3: Check if the buffer is full
    CAN_Circular_Buffer *tx_buffer = &can_circular_buffer[QUEUE_TX];
    if (tx_buffer->count >= CAN_BUFFER_SIZE) {
        osMutexRelease(tx_buffer->mutex_id); // Ensure mutex is released before returning
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, "Circular buffer is full, cannot enqueue CAN packet");
        return;
    }

    // Step 4: Allocate and prepare the CAN packet
    CAN_Packet *packet = &tx_buffer->packets[tx_buffer->head]; // Get the next write position
    uint32_t request_id = get_request_id(device->can_id);       // Get the unique request ID

    // Create and configure the tx_header
    CAN_TxHeaderTypeDef tx_header;
    //create_can_tx_header(&tx_header, request_id);               // Configure the header

    // Generate payload and set metadata
    generate_can_tx_read_data_payload(device, pid, packet->payload); // Generate the payload
    packet->meta.can_instance = can_instance;                  // Set CAN instance
    packet->meta.timestamp = HAL_GetTick();                    // Capture the timestamp
    packet->flow = PACKET_TX;                                  // Mark as Tx packet

    // Step 5: Store the packet in the circular buffer
    tx_buffer->head = (tx_buffer->head + 1) % CAN_BUFFER_SIZE; // Increment the head index
    tx_buffer->count++;                                       // Increment the count

    // Release the mutex after updating the buffer
    if (osMutexRelease(tx_buffer->mutex_id) != osOK) {
        user_error_handler(ERROR_RTOS_MUTEX_RELEASE_FAILED, "Failed to release mutex for TX circular buffer");
        return;
    }
    packet->header.dlc = 8;
    packet->header.id = request_id;

    /*
    // Step 6: Enqueue the packet and tx_header reference into the Tx queue
    CAN_Tx_Packet tx_packet = {
        .header = tx_header,  // Pass the tx_header
        .meta = packet->meta,
    };

    memcpy(tx_packet.payload, packet->payload, sizeof(tx_packet.payload));
*/
    if (osMessageQueuePut(Tx_QueueHandle, &packet, 0, 0) != osOK) {
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, "Failed to enqueue CAN packet into Tx queue");

        // Rollback the buffer changes
        if (osMutexAcquire(tx_buffer->mutex_id, osWaitForever) == osOK) {
            tx_buffer->head = (tx_buffer->head == 0) ? (CAN_BUFFER_SIZE - 1) : (tx_buffer->head - 1);
            tx_buffer->count--;
            osMutexRelease(tx_buffer->mutex_id);
        }
        return;
    }

    // Step 7: Log the message for debugging
    log_can_message(request_id, packet->payload, packet->header.dlc);
}

/**
 * @brief Processes a message from the TX queue and sends it to the TRUCK_CAN (hcan1).
 *
 * @return bool Returns true if the message was sent successfully, false otherwise.
 */
bool __rtos__process_tx_queue_and_send_to_can1(CAN_Packet *packet) {
    // Step 1: Validate the packet
    if (packet == NULL) {
        send_console_msg("Invalid CAN packet in TX queue.");
        return false;
    }

    // Step 2: Prepare the HAL CAN Tx header
    CAN_TxHeaderTypeDef hal_tx_header = {
        .StdId = packet->header.id,
        .ExtId = packet->header.id,
        .IDE = packet->header.is_extended_id ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = packet->header.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = packet->header.dlc,
        .TransmitGlobalTime = DISABLE
    };

    // Step 3: Send the message via HAL CAN API
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &hal_tx_header, packet->payload, &mailbox) != HAL_OK) {
        // Transmission failed, log the error
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "CAN TX failed: ID=0x%lX", packet->header.id);
        send_console_msg(error_msg);

        // Optionally re-enqueue the message or handle failure
        return false;
    }

    // Step 5: Log successful transmission
    char success_msg[64];
    snprintf(success_msg, sizeof(success_msg), "CAN TX success: ID=0x%lX", packet->header.id);
    send_console_msg(success_msg);

    // Step 6: Free the packet after successful transmission
    _free_can_packet_from_circular_buffer(QUEUE_TX, packet);

    return true;
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
            send_can_request_to_tx_queue(CAN_TRUCK, device, pid);
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
 * @brief Translates a HAL CAN Rx header into the unified CAN_Header format.
 *
 * This function maps fields from the HAL `CAN_RxHeaderTypeDef` structure
 * into the application's unified `CAN_Header` format for consistency across
 * Rx and Tx operations.
 *
 * @param hal_rx_header Pointer to the HAL CAN Rx header.
 * @param header Pointer to the unified `CAN_Header` structure to populate.
 */
void normalize_rx_hal_header(const CAN_RxHeaderTypeDef *hal_rx_header, CAN_Header *header) {
    if (hal_rx_header == NULL || header == NULL) {
        user_error_handler(ERROR_CAN_INVALID_CONTEXT, "NULL pointer passed to translate_rx_header");
        return;
    }

    header->id = (hal_rx_header->IDE == CAN_ID_EXT) ? hal_rx_header->ExtId : hal_rx_header->StdId;
    header->dlc = hal_rx_header->DLC;
    header->is_extended_id = (hal_rx_header->IDE == CAN_ID_EXT);
    header->is_remote_frame = (hal_rx_header->RTR == CAN_RTR_REMOTE);
    header->filter_match = hal_rx_header->FilterMatchIndex;
    header->timestamp = hal_rx_header->Timestamp;
}


/**
 * @brief Retrieve a CAN message from FIFO0 and populate a CAN_Packet.
 *
 * Retrieves a CAN message from FIFO0 using the HAL CAN driver, validates
 * the message, translates the header into a unified format, and populates
 * the provided `CAN_Packet` structure.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to the `CAN_Packet` where the received message will be stored.
 * @return true if the message was successfully retrieved and valid, false otherwise.
 */
bool get_rx_message_from_CAN_RX_FIFO0(CAN_HandleTypeDef *hcan, CANInstance can_instance, CAN_Packet *packet) {
    // Step 1: Validate `hcan`.
    if (hcan == NULL) {
        user_error_handler(ERROR_CAN_INVALID_CONTEXT, "NULL CAN_HandleTypeDef provided to get_rx_message_from_CAN_RX_FIFO0");
        return false;
    }

    // Step 2: Validate `packet`.
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "NULL CAN_Packet provided to get_rx_message_from_CAN_RX_FIFO0");
        return false;
    }

    // Step 3: Declare a temporary HAL CAN Rx header for retrieval.
    CAN_RxHeaderTypeDef hal_rx_header;

    // Step 4: Retrieve the CAN message from FIFO0.
    HAL_StatusTypeDef hal_status = HAL_CAN_GetRxMessage(
        hcan,
        CAN_RX_FIFO0,
        &hal_rx_header, // HAL Rx header
        packet->payload // Data payload buffer
    );

    // Step 5: Check HAL status.
    if (hal_status != HAL_OK) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "HAL_CAN_GetRxMessage failed with status: %d", hal_status);
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, error_msg);
        return false;
    }

    // Step 6: Validate DLC.
    if (hal_rx_header.DLC > DLC_MAX) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Invalid DLC value: %ld in received CAN message", hal_rx_header.DLC);
        user_error_handler(ERROR_CAN_INVALID_PAYLOAD, error_msg);
        return false;
    }

    // Step 7: Translate the HAL Rx header into the unified CAN_Header format.
    normalize_rx_hal_header(&hal_rx_header, &packet->header);

    // Step 8: Populate packet metadata.
    packet->flow = PACKET_RX;
    packet->meta.can_instance = can_instance;
    packet->meta.timestamp = HAL_GetTick();

    // Step 9: Log the raw CAN message for debugging.
    log_raw_can_packet(packet);

    // Step 10: Return success.
    return true;
}

/**
 * @brief Logs a raw CAN packet in a human-readable format.
 *
 * This function formats the metadata, CAN ID, DLC, and payload of a CAN packet
 * into a single log message for debugging purposes.
 *
 * @param packet Pointer to the `CAN_Packet` containing the received message.
 */
void log_raw_can_packet(const CAN_Packet *packet) {
    // Step 1: Validate the input packet
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Null CAN_Packet provided to log_raw_can_packet.");
        return;
    }

    // Step 2: Extract header information
    uint32_t can_id = packet->header.id;
    uint8_t dlc = packet->header.dlc;

    // Step 3: Validate the DLC to ensure it's within the allowable range
    if (dlc > DLC_MAX) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Invalid DLC=%d in CAN packet", dlc);
        user_error_handler(ERROR_CAN_INVALID_PAYLOAD, error_msg);
        return;
    }

    // Step 4: Initialize the log message buffer
    char log_msg[128] = {0};

    // Step 5: Format the log message with CAN instance, ID, and DLC
    int result = snprintf(
        log_msg, sizeof(log_msg),
        "Raw CAN: Instance=%d, ID=0x%" PRIX32 ", DLC=%d, Data=",
        packet->meta.can_instance, can_id, dlc
    );

    // Step 6: Check for formatting errors
    if (result < 0 || result >= (int)sizeof(log_msg)) {
        user_error_handler(ERROR_UI_LOGGING_FAILED, "Log message construction failed in log_raw_can_packet.");
        return;
    }

    // Step 7: Append payload data to the log message
    size_t offset = strlen(log_msg);
    for (uint8_t i = 0; i < dlc; i++) {
        if (offset >= sizeof(log_msg)) {
            user_error_handler(ERROR_UI_LOGGING_FAILED, "Log message exceeded buffer size in log_raw_can_packet.");
            return;
        }
        int append_result = snprintf(&log_msg[offset], sizeof(log_msg) - offset, "%02X ", packet->payload[i]);
        if (append_result < 0) {
            user_error_handler(ERROR_UI_LOGGING_FAILED, "Failed to append payload data to log message.");
            return;
        }
        offset += append_result;
    }

    // Step 8: Log the complete message
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

/**
 * @brief Log validated CAN data for debugging purposes.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the parsed CAN data structure.
 */
void log_valid_can_data(const Parsed_CAN_Data *parsed_data) {
    char log_msg[128] = {0};

    // Use PRIX32 for portable handling of uint32_t in hexadecimal
    snprintf(log_msg, sizeof(log_msg),
             "Valid CAN Data: PID=0x%04" PRIX16 " Payload=0x%08" PRIX32,
             parsed_data->pid, parsed_data->payload);

    // Log the message
    send_console_msg(log_msg);
}

/**
 * @brief Processes and sends a CAN packet.
 *
 * This function handles the following:
 * 1. Validates the provided CAN packet for correctness.
 * 2. Identifies the appropriate CAN hardware interface (e.g., CAN1 or CAN2) based on the packet's metadata.
 * 3. Converts the generalized CAN header to the HAL-specific Tx header format.
 * 4. Transmits the CAN packet using the HAL CAN API.
 * 5. Logs detailed transmission statistics, including transmission time and running average of all transmitted packets.
 *
 * @param packet Pointer to the `CAN_Packet` containing the metadata, unified header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool process_and_send_can_tx_packet(CAN_Packet *packet) {
    if (packet == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Tx packet pointer is NULL");
        return false;
    }

    // Step 1: Validate the CAN instance
    CAN_HandleTypeDef *target_can = get_can_handle_from_instance(packet->meta.can_instance);
    if (target_can == NULL) {
        user_error_handler(ERROR_CAN_INVALID_CONTEXT, "Invalid CAN instance in Tx packet");
        return false;
    }

    // Step 2: Convert the generalized header to the HAL Tx header format
    CAN_TxHeaderTypeDef hal_tx_header = {
        .StdId = packet->header.id,
        .ExtId = packet->header.id,
        .IDE = packet->header.is_extended_id ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = packet->header.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = packet->header.dlc,
        .TransmitGlobalTime = DISABLE // Default value for Tx header
    };

    // Step 3: Attempt to send the packet
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        target_can,               // Target CAN interface
        &hal_tx_header,           // HAL-specific Tx header
        packet->payload,          // Payload data
        NULL                      // Mailbox (NULL since we're not directly tracking mailboxes here)
    );

    if (status != HAL_OK) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "CAN Tx failed on instance: %d", packet->meta.can_instance);
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, error_msg);
        return false;
    }

    // Step 4: Calculate transmission time and update running average
    uint32_t current_time = HAL_GetTick();
    uint32_t transmission_time = current_time - packet->meta.timestamp;

    static uint32_t tx_time_sum = 0;
    static uint32_t tx_count = 0;

    tx_time_sum += transmission_time;
    tx_count++;

    uint32_t avg_tx_time = tx_time_sum / tx_count;

    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "CAN Tx success: Instance=%d, Time=%lu ms, Avg Time=%lu ms",
             packet->meta.can_instance, transmission_time, avg_tx_time);
    send_console_msg(log_msg);

    return true;
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
void process_can_rx_packet(CAN_Packet *packet) {
    // Step 1: Validate the input
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Received null CANPacket");
        return; // Exit early to prevent further processing
    }

    // Step 2: Log the raw CAN data for debugging
    log_raw_can_packet(packet);

    // Step 3: Check if the CAN message should be ignored
    if (should_ignore_message(packet->header.id)) {
        return; // Exit early if the message is not relevant
    }

    // Step 4: Parse the raw CAN data into a structured format
    Parsed_CAN_Data parsed_data;
    parse_raw_can_data(packet->payload, &parsed_data);

    // Step 5: Validate the parsed data
    if (!validate_parsed_can_data(&parsed_data)) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Invalid parsed CAN data: ID=0x%" PRIX32 ", PID=0x%X, DLC=%d",
                 packet->header.id, parsed_data.pid, parsed_data.data_length);
        user_error_handler(ERROR_CAN_DATA_PARSE_FAILED, error_msg);
        return; // Exit early if the data is invalid
    }

    // Step 6: Locate the device configuration for the CAN ID
    CANDeviceConfig *device_config = get_device_config_by_id(packet->header.id);
    if (device_config == NULL) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Device configuration not found for CAN ID: 0x%" PRIX32, packet->header.id);
        user_error_handler(ERROR_CAN_DEVICE_NOT_FOUND, error_msg);
        return; // Exit if no matching device configuration is found
    }

    // Step 7: Locate the PID configuration for the parsed PID
    CANDevicePID *pid_config = get_pid_by_id(device_config, parsed_data.pid);
    if (pid_config == NULL) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "PID configuration not found for device: ID=0x%" PRIX32 ", PID=0x%X",
                 packet->header.id, parsed_data.pid);
        user_error_handler(ERROR_CAN_PID_NOT_FOUND, error_msg);
        return; // Exit if no matching PID configuration is found
    }

    // Step 8: Update the signal states based on the parsed data
    process_signal_changes(pid_config, parsed_data.payload);

    // Step 9: Log the validated and processed CAN data for debugging
    log_valid_can_data(&parsed_data);
}
