/*
 * can_tx.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ryan
 */

/*
 * @brief Generate the CAN payload based on the device and PID configuration.
 *
 * Constructs the CAN request payload by combining information from the provided
 * device and PID, optionally converting it to big-endian format for transmission.
 *
 * @param device Pointer to the CANDeviceConfig representing the target device.
 * @param pid Pointer to the CANDevicePID specifying the requested PID.
 * @param TxData Pointer to the data buffer where the payload will be stored.
 */

#include <stdio.h>
#include <string.h>

#include "can_tx.h"
#include "can_common.h"
#include "can_helper.h"
#include "error.h"
#include "log.h"
#include "can_rx.h"
#include "can_packet.h"


void generate_can_tx_read_data_payload(CANDeviceConfig *device, CANDevicePID *pid, CAN_Payload_Array *TxData) {
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
    generate_can_tx_read_data_payload(device, pid, &(packet->payload)); // Generate the payload
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
