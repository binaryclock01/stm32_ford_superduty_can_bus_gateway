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

#include "main.h"            // For HAL_CAN and other core includes
#include "ssd1306.h"         // For OLED display functions
#include "ui.h"              // For console message utilities
#include "utils.h"           // For helper functions like bytes_to_uint32
#include "error.h"           // For error handling utilities

/* -----------------------------------------------------------------------------
   Function Definitions
   -------------------------------------------------------------------------- */

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
#define CHAR_BIT 8
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
 * @brief Send a CAN request for a specific device and PID.
 *
 * Prepares and transmits a CAN message for a specified device and PID.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2).
 * @param device Pointer to the CANDeviceConfig representing the device.
 * @param pid Pointer to the CANDevicePID representing the PID to request.
 */
void send_can_request(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid) {
    uint64_t request_id = get_request_id(device->can_id);

    // Configure CAN transmission header
    TxHeader[can_instance].StdId = request_id;
    TxHeader[can_instance].IDE = CAN_ID_STD;
    TxHeader[can_instance].RTR = CAN_RTR_DATA;
    TxHeader[can_instance].DLC = MAX_DLC_BYTE_LENGTH;
    TxHeader[can_instance].TransmitGlobalTime = DISABLE;

    uint64_t request_data = build_can_request(device, pid);
    uint64_t reversed_bytes_request_data = __builtin_bswap64(request_data);  // Convert to big-endian

    // Copy request data into TxData buffer
    memcpy(TxData[can_instance], &reversed_bytes_request_data, TxHeader[can_instance].DLC);

    // Transmit the CAN message
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader[can_instance], TxData[can_instance], &TxMailbox[can_instance]) != HAL_OK) {
        char error_msg[255];
        snprintf(error_msg, sizeof(error_msg), "Err Tx on CAN%d", can_instance);
        send_Console_Msg(error_msg);
        Error_Handler();
    } else {
        tx_count[can_instance]++;
        char sent_msg[255];
        int offset = snprintf(sent_msg, sizeof(sent_msg), "Tx %X/", (unsigned int)request_id);

        for (int j = 0; j < TxHeader[can_instance].DLC; j++) {
            offset += snprintf(&sent_msg[offset], sizeof(sent_msg) - offset, "%02X", TxData[can_instance][j]);
        }

        send_Console_Msg(sent_msg);
    }
}

/**
 * @brief Send CAN requests for all devices and their PIDs.
 *
 * Iterates through all configured devices and PIDs to send CAN requests.
 */
void send_all_can_requests(void) {

#ifdef LOOPBACK_MODE
	// Send BCM Brake Pedal for test in loopback mode
	send_can_request(CAN_TRUCK, &can_devices[0], &(can_devices[0].pids[1]));
#else
    for (uint8_t device_index = 0; device_index < CAN_DEVICE_COUNT; device_index++) {
        CANDeviceConfig *device = &can_devices[device_index];

        for (size_t pid_index = 0; pid_index < device->pid_count; pid_index++) {
            CANDevicePID *pid = &device->pids[pid_index];
            send_can_request(CAN_TRUCK, device, pid);
        }
    }
#endif
}

/**
 * @brief Retrieve a CANDeviceConfig by its CAN ID.
 *
 * Searches for a device configuration matching the given CAN ID.
 *
 * @param rx_id The received CAN ID.
 * @return CANDeviceConfig* Pointer to the matching device configuration, or NULL if not found.
 */
CANDeviceConfig *get_CANDeviceConfig_by_canid(uint32_t rx_id) {
    for (uint8_t module_idx = 0; module_idx < CAN_DEVICE_COUNT; module_idx++) {
        CANDeviceConfig *module_config = &can_devices[module_idx];
        if (module_config->can_id == rx_id) {
            return module_config;
        }
    }
    return NULL;
}

/**
 * @brief Retrieve a CANDevicePID by its PID.
 *
 * Searches for a PID within a specific device configuration.
 *
 * @param module Pointer to the CANDeviceConfig representing the device.
 * @param pid The PID to search for.
 * @return CANDevicePID* Pointer to the matching PID configuration, or NULL if not found.
 */
CANDevicePID *get_CANDevicePID_by_pid(CANDeviceConfig *module, uint16_t pid) {
    for (uint8_t pid_idx = 0; pid_idx < module->pid_count; pid_idx++) {
        CANDevicePID *device_pid = &module->pids[pid_idx];
        uint16_t module_pid = 0;
        for (int i = 0; i < PID_BYTE_LENGTH; i++)
        {
            module_pid |= ((uint16_t)(((uint8_t)device_pid->pid_id[i]))) << (8 * (1 - i));
        }
        if (module_pid == pid) {
            return device_pid;
        }
    }
    return NULL;
}

/**
 * @brief Compare signals of a PID with received payload for changes.
 *
 * Iterates over the signals within a PID and checks for state changes based on the payload.
 *
 * @param device_pid Pointer to the CANDevicePID containing the signals.
 * @param payload The received payload data to compare against.
 */
void iterate_signals_for_changes(CANDevicePID *device_pid, uint32_t payload) {
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
 * @brief Parse a received CAN message and process it.
 *
 * Parses a received CAN message, checks the corresponding device and PID,
 * and processes the signals based on the payload.
 *
 * @param RAW_rx_id The received CAN ID.
 * @param RAW_rx_data_as_byte_array The received CAN data as a byte array.
 */
void parse_rx_CAN_message(uint32_t RAW_rx_id, uint8_t *RAW_rx_data_as_byte_array) {
    // Ignore heartbeat messages
    if (RAW_rx_id == CAN_ID_HEARTBEAT) {
        return;
    }

#ifdef LOOPBACK_MODE
    RAW_rx_id += 8;
    RAW_rx_data_as_byte_array[5] = 0x60;
#endif

    // Get CAN device configuration
    CANDeviceConfig *selected_can_device = get_CANDeviceConfig_by_canid(RAW_rx_id);
    if (!selected_can_device) {
        User_Error_Handler(ERROR_CAN_MODULE_NOT_FOUND);
        return;
    }

    // Extract fields from the message
    uint64_t data = 0;
    bytes_to_big_endian(&data, RAW_rx_data_as_byte_array, 8);  // Convert to big-endian

    uint8_t rx_data_length = (data >> 56) & 0xFF;  // Extract data length (highest byte)
    uint16_t rx_pid = ((uint16_t)(data >> 32)) & 0x0000FFFF;      // Extract PID (next 2 bytes)
    uint32_t rx_payload = data & 0xFFFFFFFF;      // Extract payload (lowest 4 bytes)

    // Validate data length
    if (rx_data_length < 4 || rx_data_length > 8) {
        send_Console_Msg("Err: Data length out of range");
        Error_Handler();
    }

    if (rx_pid == 0x40C8)
    {
    	int i = 5;
    }
    // Get the PID configuration
    CANDevicePID *selected_pid = get_CANDevicePID_by_pid(selected_can_device, rx_pid);
    if (!selected_pid) {
        //User_Error_Handler(ERROR_MODULE_PID_NOT_FOUND);
        return;
    }

    // Process signals for the selected PID
    iterate_signals_for_changes(selected_pid, rx_payload);
}
