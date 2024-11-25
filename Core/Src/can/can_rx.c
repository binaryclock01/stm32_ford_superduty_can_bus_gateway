/*
 * can_rx.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

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

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "buffers.h"
#include "can_rx.h"

#include "can_packet.h"
#include "can_common.h"
#include "error.h"

#include "can_helper.h"
#include "can_tx.h"
#include "can_core.h"


void process_can_rx_packet(Circular_Queue_Types queue_enum, CANInstance can_instance_enum, CAN_Packet *packet) {
    // Step 1: Validate the input
    if (packet == NULL) {
        user_error_handler(ERROR_CAN_PACKET_NULL, "Received null CANPacket");
        return; // Exit early to prevent further processing
    }

    // Step 2: Log the raw CAN data for debugging
    //log_raw_can_packet(packet);

    // Step 3: Check if the CAN message should be ignored
    if (should_ignore_message(packet->header.id)) {
        return; // Exit early if the message is not relevant
    }

    // Step 4: Parse the raw CAN data into a structured format
    Parsed_CAN_Data parsed_data;
    parse_raw_can_data(packet->payload, &parsed_data);

    /*
    // Step 5: Validate the parsed data
    if (!validate_parsed_can_data(&parsed_data)) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Invalid parsed CAN data: ID=0x%" PRIX32 ", PID=0x%X, DLC=%d",
                 packet->header.id, parsed_data.pid, parsed_data.data_length);
        user_error_handler(ERROR_CAN_DATA_PARSE_FAILED, error_msg);
        return; // Exit early if the data is invalid
    }
    */

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
    		__sim__generate_packet_response_from_truck(pid_config, parsed_data.command);
#else
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
bool _get_rx_message_from_CAN_RX_FIFO0(CAN_HandleTypeDef *hcan, CANInstance can_instance, CAN_Packet *packet) {
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
    //log_raw_can_packet(packet);

    // Step 10: Return success.
    return true;
}

/**
 * @brief Processes a CAN Rx FIFO0 message callback.
 *
 * This function is triggered when a CAN message is received in FIFO0.
 * It identifies the associated CAN instance, allocates a packet from the memory pool,
 * retrieves the CAN message, and enqueues it into the appropriate message queue.
 * The responsibility for processing and freeing the packet lies with the task
 * consuming the queue (e.g., StartCAN1_Rx_Task or StartCAN2_Rx_Task).
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 */
void _process_can_rx_fifo_callback(CAN_HandleTypeDef *hcan) {
    // Step 1: Determine the CAN instance based on the provided CAN hardware handle
    CANInstance can_instance = get_can_instance_from_hcan(hcan);
    if (can_instance == CAN_TOTAL) {
        // Log and exit if the CAN instance is invalid
        user_error_handler(ERROR_CAN_MODULE_NOT_FOUND, "Unknown CAN instance in Rx FIFO callback");
        return;
    }

    // Step 2: Retrieve the message queue for the corresponding CAN instance
    osMessageQueueId_t target_queue = get_rx_queue_handle_from_hcan(hcan);
    if (target_queue == NULL) {
        // Log and exit if the target queue is unavailable
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Message queue unavailable for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_INIT_FAILED, error_msg);
        return;
    }

    Circular_Queue_Types queue_type = get_queue_num_by_can_instance(can_instance, QUEUE_TYPE_FLOW_RX);

    // Step 3: Allocate a CAN packet from the memory pool
    CAN_Packet *packet = _allocate_can_packet_on_circular_buffer(queue_type);

    if (packet == NULL) {
        // Log and exit if no free packet is available in the memory pool
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "No free packet in pool for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_BUFFER_OVERFLOW, error_msg);
        return;
    }

    // Step 4: Retrieve the CAN message and populate the allocated packet
    if (!_get_rx_message_from_CAN_RX_FIFO0(hcan, can_instance, packet)) {
        // Free the packet and log the error if message retrieval fails
        _free_can_packet_using_queue_type_from_circular_buffer(queue_type);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Failed to retrieve CAN message for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_RETRIEVE_FAILED, error_msg);
        return;
    }

    // Step 5: Enqueue the packet into the message queue
    if (osMessageQueuePut(target_queue, &packet, 0, 0) != osOK) {
        // Free the packet and log the error if the queue is full or unavailable
        _free_can_packet_using_queue_type_from_circular_buffer(queue_type);
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Queue full for CAN instance %d", can_instance);
        user_error_handler(ERROR_CAN_QUEUE_FULL, error_msg);
        return;
    }

    // Note: Do NOT free the packet here!
    // The osMessageQueuePut function only copies the pointer to the packet,
    // not the packet data itself. The packet must remain valid for the
    // consumer task (e.g., StartCAN1_Rx_Task) to process and free.
}
