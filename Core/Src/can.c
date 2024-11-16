/*
 * can.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */

#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdint.h>
#include <stdbool.h> // for boolean support in c

#include "main.h"
#include "ssd1306.h"
#include "ui.h"
#include "utils.h"
#include "error.h"

uint64_t build_can_request(CANDeviceConfig *device, CANDevicePID *pid)
{
	uint64_t uint64_payload = 0;

	uint8_t request_size_in_bytes = 0x03; // 3 bytes because of payload below added up
	uint8_t command_byte = can_request_commands[CMD_READ].byte; // 0x22
	uint16_t pid_id = 0;
	pid_id = __builtin_bswap16(*(uint16_t *)pid->pid_id);

	// MAX_DLC_BYTE_LENGTH = 8
	uint8_t starting_msb = ((MAX_DLC_BYTE_LENGTH-1)*8);

	uint8_t uint8_shift = starting_msb - DATA_LENGTH_START;
	uint64_t uint64_size_in_bytes = (uint64_t)request_size_in_bytes;

	uint64_payload |= ((uint64_t)uint64_size_in_bytes) << uint8_shift;
	uint64_payload |= ((uint64_t)command_byte) << (starting_msb - (DATA_COMMAND_START * 8));
	uint64_payload |= ((uint64_t)pid_id) << (starting_msb - ((DATA_PID_START+1) * 8));

	return uint64_payload;
}

// Function to send a CAN request for a specific device
void send_can_request(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid)
{
    // Calculate the request ID by subtracting the offset
    uint64_t request_id = get_request_id(device->can_id);

    // Prepare the CAN transmission header
    TxHeader[can_instance].StdId = request_id;
    TxHeader[can_instance].ExtId = 0;
    TxHeader[can_instance].IDE = CAN_ID_STD;  // Use standard CAN ID
    TxHeader[can_instance].RTR = CAN_RTR_DATA;
    TxHeader[can_instance].DLC = MAX_DLC_BYTE_LENGTH;  // Default Data Length Code
    TxHeader[can_instance].TransmitGlobalTime = DISABLE;

    uint64_t request_data = build_can_request(device, pid);

	// reverse the byte order first (little endian to big endian)
    uint64_t reversed_bytes_request_data = __builtin_bswap64(request_data);
    // Copy request data into TxData buffer
    memcpy(TxData[can_instance], &reversed_bytes_request_data, TxHeader[can_instance].DLC);

	// Send the CAN message
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader[can_instance], TxData[can_instance], &TxMailbox[can_instance]) != HAL_OK) {
		char error_msg[255];
		sprintf(error_msg, "Err Tx on CAN%d", can_instance);
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



// Function to loop through all device configurations and send a request for each
void send_all_can_requests(void)
{
    for (uint8_t device_index = 0; device_index < CAN_DEVICE_COUNT; device_index++)
    {
        CANDeviceConfig *device = &can_devices[device_index];

        // Loop through each PID in the current device
        for (size_t pid_index = 0; pid_index < device->pid_count; pid_index++)
        {
            CANDevicePID *pid = &device->pids[pid_index];
            send_can_request(CAN_TRUCK, device, pid);
        }
    }
}


CANDeviceConfig *get_CANDeviceConfig_by_canid(uint32_t rx_id)
{
    for (uint8_t module_idx = 0; module_idx < CAN_DEVICE_COUNT; module_idx++)
    {
        CANDeviceConfig *module_config = &can_devices[module_idx];

        // Check if the rx_id matches the module's can_id
        if (module_config->can_id == rx_id)
        	return module_config;
    }
    return NULL;
}

CANDevicePID *get_CANDevicePID_by_pid(CANDeviceConfig *module, uint16_t pid)
{
    for (uint8_t pid_idx = 0; pid_idx < module->pid_count; pid_idx++)
    {
         CANDevicePID *device_pid = &module->pids[pid_idx];

         uint16_t module_pid = 0;
         memcpy(&module_pid, ((uint8_t *)&pid), PID_BYTE_LENGTH);

         if (module_pid == pid)
        	 return device_pid;
    }
    return NULL;
}

void iterate_signals_for_changes(CANDevicePID *device_pid, uint32_t payload)
{
	for (uint8_t signal_idx = 0; signal_idx < sizeof(device_pid->signals) / sizeof(device_pid->signals[0]); signal_idx++)
	{
		CANSignal *signal = &device_pid->signals[signal_idx];

		// Check the signal change based on its change_type
		switch (signal->change_type)
		{
			case STATE_BIT:
				// Compare the signal's "on" state using change_data (bitwise)
				uint32_t signal_data_mask = bytes_to_uint32((uint8_t *)signal->change_data);

				/* apply bitwise AND to payload and signal_data_mask
				 * if equals signal_data_mask (original mask),
				 * then signal is active because the bit is present and 1
				 * Example:
				 * signal_data_mask 0010 AND
				 * payload          0110
				 *                  ----
				 *                  0010 RESULT  (this means it was on)
				 */
				// Store the result as a uint32_t (0 for false, 1 for true)
				signal->data = (uint32_t)((payload & signal_data_mask) == signal_data_mask);

				break;
			case STATE_BYTE:
				// TODO: IMPLEMENT STATE_BYTE CONDITION
			case STATE_MULTIBYTE:
				// TODO: IMPLEMENT STATE_MULTIBYTE CONDITION
			default:
		}
	}
}

void parse_rx_CAN_message(uint32_t RAW_rx_id, uint8_t *RAW_rx_data_as_byte_array)
{
	// check to see if rx_id is the heartbeat id, if so, don't process it
	if (RAW_rx_id == CAN_ID_HEARTBEAT)
		return;

	// CANDevice placeholder pointer
	CANDeviceConfig *selected_can_device = NULL;
	CANDevicePID *selected_pid = NULL;

	// put 8 bytes of rx data into 64 bit variable
	const uint32_t data = bytes_to_uint32(RAW_rx_data_as_byte_array);
	// Use CANMasks to mask out each part of the data
    const uint8_t  rx_data_length = data & UINT8_LENGTH_MASK;
    const uint8_t  rx_command     = data & UINT8_COMMAND_MASK;
    const uint16_t rx_pid         = data & UINT16_PID_MASK;
    uint32_t rx_payload     = 0;

    // if not a RESP_READ 0x62 command in reply, then just exit this function
    if (rx_command != RESP_READ)
    	return;

    // ensure data length is valid range
    if (!IN_RANGE(rx_data_length, 0, MAX_PAYLOAD_LENGTH))
    {
    	send_Console_Msg("Er data len out of range");
    	Error_Handler();
    }

    // copy the length of payload section of data into rx_payload
    memcpy(&rx_payload, ((uint8_t *)&data) + DATA_PAYLOAD_START, rx_data_length);

    // BCM, SCCW, etc. These hold the list of PIDs
    if ((selected_can_device = get_CANDeviceConfig_by_canid(RAW_rx_id)) == NULL)
    {
    	User_Error_Handler(ERROR_CAN_MODULE_NOT_FOUND);
    	return; // just in case
    }

    // Get the PID from the device, example Hazard Button PID struct from BCM
    if ((selected_pid = get_CANDevicePID_by_pid(selected_can_device, rx_pid)) == NULL)
    {
    	User_Error_Handler(ERROR_MODULE_PID_NOT_FOUND);
    	return; // just in case
    }

    // Check the signals of the PID against the rx_payload
    iterate_signals_for_changes(selected_pid, rx_payload);
}
