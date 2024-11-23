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

#include "can_core.h"
#include "config.h"
#include "main.h"            // For HAL_CAN and other core includes
#include "device_configs.h"  // Include configurations for CAN devices
#include "ssd1306.h"         // For OLED display functions
#include "ui.h"              // For console message utilities
#include "utils.h"           // For helper functions like bytes_to_uint32
#include "error.h"           // For error handling utilities
#include "rtos.h"
#include "can_core.h"
#include "ansi.h"
#include "log.h"
#include "sim.h"

/* -----------------------------------------------------------------------------
   Function Definitions
   -------------------------------------------------------------------------- */

#define USE_CAN_1
//#define USE_CAN_2

#define CHAR_BIT 8

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

void activate_single_can_bus_fifo_callback(CAN_HandleTypeDef *hcan, const char *hcan_human_string)
{
	  // Activate notification for TRUCK CAN
	  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
	  {
		  log_status_message(hcan_human_string, true);
	  }
	  else
	  {
		  log_status_message(hcan_human_string, true);
	      user_error_handler(ERROR_CAN_NOTIFICATION_FAILED, "Failed to activate CAN_IT_RX_FIFO0_MSG_PENDING notification on ", hcan_human_string);
	  }
}

void activate_can_bus_fifo_callbacks()
{
    log_message("* Initializing CAN BUS FIFO Callback Functions");
#ifdef USE_CAN_1
	activate_single_can_bus_fifo_callback(&hcan1, "  - " HWHT "TruckNet " CRESET "[" HWHT "CAN1" CRESET "]");
#endif

#ifdef USE_CAN_2
	activate_single_can_bus_fifo_callback(&hcan2, "  - " HWHT "AuxNet " CRESET "[" HWHT "CAN2" CRESET "]");
#endif
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

uint32_t get_can_device_stdid(CANDeviceConfig *device, CAN_Verb_Type verb)
{
	if (verb == CAN_VERB_REPLY)
		return device->id.response;
	else if (verb == CAN_VERB_REQUEST)
		return device->id.request;
	return 0;
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
void send_can_packet_to_tx_queue(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid, CAN_Verb_Type verb) {
    // Step 1: Validate inputs
    if (device == NULL || pid == NULL || can_instance >= CAN_TOTAL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "Device or PID is NULL or invalid CAN instance in send_can_request_to_tx_queue");
        return;
    }

    // Step 2: Get queue information
    Circular_Queue_Types queue_num = get_queue_num_by_can_instance(can_instance, QUEUE_TYPE_FLOW_TX);

    // osMessageQueueId_t is a pointer that holds the queue_handle so don't return pointers to pointers!
    osMessageQueueId_t queue_handle = get_queue_handle_by_queue_num(queue_num);

    // Step 3: Allocate a CAN packet from the circular buffer
    CAN_Packet *packet = _allocate_can_packet_on_circular_buffer(queue_num);
    if (!packet) {
        user_error_handler(ERROR_RTOS_QUEUE_ALLOCATION_FAILED, "Failed to allocate CAN packet in send_can_request_to_tx_queue");
        return;
    }

    // get request id if verb is CAN_VERB_REQUEST, and reply id if verb is CAN_VERB_REPLY
    uint32_t verb_stdid = get_can_device_stdid(device, verb);

    // Step 5: Populate the CAN packet
    generate_can_tx_read_data_payload(device, pid, packet->payload); // Generate the payload
    packet->meta.can_instance = can_instance;                  // Set CAN instance
    packet->meta.timestamp = HAL_GetTick();                    // Capture the timestamp
    packet->flow = PACKET_TX;                                  // Mark as Tx packet
    packet->header.dlc = 8;                                    // Data length code
    packet->header.id = verb_stdid;                            // Unique request ID

    // Step 6: Attempt to enqueue the packet
    if (osMessageQueuePut(queue_handle, &packet, 0, 0) != osOK) {
        // Log failure to enqueue
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, __func__, "Failed to enqueue CAN packet into Tx queue");

        // Rollback circular buffer changes
        CAN_Circular_Buffer *tx_buffer = &can_circular_buffer[queue_num];
        if (osMutexAcquire(tx_buffer->mutex_id, osWaitForever) == osOK) {
            tx_buffer->head = (tx_buffer->head == 0) ? (CAN_BUFFER_SIZE - 1) : (tx_buffer->head - 1);
            tx_buffer->count--;
            osMutexRelease(tx_buffer->mutex_id);
        } else {
            user_error_handler(ERROR_RTOS_MUTEX_INIT_FAILED, "Failed to acquire mutex for rollback in send_can_request_to_tx_queue");
        }
        return;
    }

//    CAN_Packet *packet_get;
//    osMessageQueueGet(queue_handle, &packet_get, NULL, osWaitForever);
    // Step 7: Log the message for debugging
    log_transmitted_can_message(queue_num, verb_stdid, packet->payload, packet->header.dlc);
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
            	signal->data = (is_signal_on(signal, payload) ? 1 : 0);
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


/**
 * @brief Sends a CAN packet using the HAL CAN API.
 *
 * Handles CAN-specific operations like header conversion, message transmission,
 * and logging.
 *
 * @param packet Pointer to the `CAN_Packet` containing metadata, header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool _send_tx_packet_to_can_interface(CAN_Packet *packet) {
    // Step 1: Validate the CAN instance
    CAN_HandleTypeDef *target_can = get_hcan_from_instance(packet->meta.can_instance);
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
        .TransmitGlobalTime = DISABLE
    };

    // Step 3: Attempt to send the packet
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        target_can,
        &hal_tx_header,
        packet->payload,
        NULL
    );

    if (status != HAL_OK) {
        char error_msg[255];
        snprintf(error_msg, sizeof(error_msg), "%s: CAN Tx failed on instance: %d\r\n", __func__, packet->meta.can_instance);
        user_error_handler(ERROR_CAN_TRANSMIT_FAILED, error_msg);
        return false;
    }

    // Step 4: Calculate and log transmission statistics
    uint32_t current_time = HAL_GetTick();
    uint32_t transmission_time = current_time - packet->meta.timestamp;

    static uint32_t tx_time_sum = 0;
    static uint32_t tx_count = 0;

    tx_time_sum += transmission_time;
    tx_count++;

    uint32_t avg_tx_time = tx_time_sum / tx_count;

    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg),
             "CAN Tx success: Instance=%d, Time=%lu ms, Avg Time=%lu ms",
             packet->meta.can_instance, transmission_time, avg_tx_time);
    log_message(log_msg);

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
 *
 * ** DO NOT FREE PACKET. void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance) is the function that called this.
 * ** the calling function will free the packet after this function returns.
 */

void process_can_rx_packet(Circular_Queue_Types queue_enum, CANInstance can_instance_enum, CAN_Packet *packet) {
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

    log_valid_can_data(&parsed_data);

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

    switch (can_instance_enum)
    {
    	case CAN_TRUCK:
    	    // Step 8: Update the signal states based on the parsed data

// if this a simulator (aka, simulating the truck's responses)
#ifdef IS_SIMULATOR
    		// then generate a new packet to respond to the received packet
    		__sim__generate_packet_response_from_truck(pid_config, parsed_data->command);
#elif
    		// if this is not a simulator, then process the signal change
    	    process_signal_changes(pid_config, parsed_data.payload);
#endif
    		break;
    	case CAN_AUX:
    		CANInstance reply_enum = CAN_TRUCK;
    		send_can_packet_to_tx_queue(reply_enum, device_config, pid_config, CAN_VERB_REPLY);
    		break;
    	default:
    		char error_msg[255];
            snprintf(error_msg, sizeof(error_msg), "%s: Invalid CAN Instance enum: %du", __func__, can_instance_enum);
            user_error_handler(ERROR_CAN_DEVICE_NOT_FOUND, error_msg);
            return; // Exit if no matching PID configuration is found
    }

    // ** DO NOT FREE PACKET. void __rtos__StartCAN_Rx_Task(CANInstance enum_can_instance) is the function that called this.
    // ** the calling function will free the packet after this function returns.
}

