/*
 * sim.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Ryan
 */

// need to at least include config.h to check for IS_SIMULATOR define
#include "config.h"
// only load this file if in the SIMULATOR mode
#ifdef IS_SIMULATOR

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "buffers.h"
#include "sim.h"
#include "hid.h"
#include "device_configs.h"
#include "can_core.h"
#include "ansi.h"
#include "log.h"
#include "can_helper.h"
#include "can_tx.h"

/*
bool isSimulator() {
    return isSimulatorMode; // Returns the pre-determined value
}
*/

/*
bool is_sim_mode_enabled(void)
{
	if (HAL_GPIO_ReadPin(SIM_MODE_GPIO_Port, SIM_MODE_Pin) == GPIO_PIN_RESET) {
	    // Pin is LOW (pulled down to GND)
	    printf("SIM_MODE_Pin is pulled down (LOW)\n");
	    return true;
	} else {
	    // Pin is HIGH (default pull-up state)
	    printf("SIM_MODE_Pin is in pull-up state (HIGH)\n");
	    return false;
	}
	return false;
}
*/

bool _g_sim_brake_on = false;
bool _g_sim_hazards_on = false;
bool _g_sim_reverse_on = false;
bool _g_sim_left_turn_on = false;
bool _g_sim_right_turn_on = false;
bool _g_sim_left_change_on = false;
bool _g_sim_right_change_on = false;



_SIM_Turn_Signal_States __sim__g_turn_signal_states = {
    .turn_left_state = 1,  // Example: Left turn active
    .right_left_state = 0,
    .turn_left_change_state = 1,  // Example: Lane change active
    .turn_right_change_state = 0
};

_SIM_Signal_States __sim__g_signal_states = {
    .hazard_button = false,
    .reverse_lamp = false,
    .brake_pedal = false,
    .turn_signal_multiplexed_byte = 0
};

uint8_t __sim__g_states_mapping_array[TURN_SIGNAL_COUNT]; // non-constant array

void __sim__initialize_states_mapping_array() {
    __sim__g_states_mapping_array[0] = __sim__g_turn_signal_states.turn_left_state;
    __sim__g_states_mapping_array[1] = __sim__g_turn_signal_states.right_left_state;
    __sim__g_states_mapping_array[2] = __sim__g_turn_signal_states.turn_left_change_state;
    __sim__g_states_mapping_array[3] = __sim__g_turn_signal_states.turn_right_change_state;
}
/*

uint8_t multiplex_turn_signal_states(_SIM_Turn_Signal_States *turn_signal_states, _SIM_Signal_States *signal_states) {
	// assign the mapping arrays
	const uint64_t *turn_signal_bitmask_mapping_array = _g_turn_signal_bitmask_mapping_array;
	const uint8_t *states_mapping_array = __sim__g_states_mapping_array;


    if (turn_signal_states == NULL || signal_states == NULL) {
        return 0; // Ensure valid pointers
    }

    // Start with a zeroed multiplexed byte
    uint8_t multiplexed_byte = 0;

    // Iterate through masks and states
    for (int i = 0; i < 4; i++) {
        if (states_mapping_array[i]) {
            multiplexed_byte |= ~turn_signal_bitmask_mapping_array[i]; // Apply the inverted mask
        }
    }

    return multiplexed_byte;
}
*/

/**
 * @brief Creates a simulated READ response packet for the given PID.
 *
 * @param device_pid Pointer to the CANDevicePID structure representing the PID.
 * @param packet Pointer to the CAN_Packet structure to store the generated packet.
 * @param tx_header Pointer to the CAN_TxHeaderTypeDef structure for the TX header.
 * @param stdid CAN standard ID.
 * @param signal_payload Precomputed payload to use directly.
 * @return True if the packet was successfully created; false otherwise.
 */
bool __sim__create_read_response_packet_for_pid(
    CANDevicePID *device_pid,
    CAN_Packet *packet,
    CAN_TxHeaderTypeDef *tx_header,
    CAN_StdIds stdid,
    const uint8_t *signal_payload)
{
    if (device_pid == NULL || packet == NULL || signal_payload == NULL) {
        log_message("Error: Null parameter in __sim__create_read_response_packet_for_pid");
        return false;
    }

    // Set the total data length
    packet->payload[DATA_LENGTH_START] = (DATA_PAYLOAD_START - 1) + MAX_PAYLOAD_BYTE_LENGTH;

    // Set the response command
    packet->payload[DATA_COMMAND_START] = RESP_READ_VALUE;

    // Copy the PID
    for (int i = 0; i < PID_BYTE_LENGTH; i++) {
        packet->payload[DATA_PID_START + i] = device_pid->pid_id[i];
    }

    // Copy the signal payload directly into the packet
    for (int i = 0; i < MAX_PAYLOAD_BYTE_LENGTH; i++) {
        packet->payload[DATA_PAYLOAD_START + i] = signal_payload[i];
    }

    // Create CAN TX header
    create_can_tx_header(tx_header, stdid);

    return true;
}

void __sim__send_tx_packet_response_to_can1(CANDevicePID *device_pid, const uint8_t *signal_payload)
{
    CANDeviceConfig *can_device_ptr = &(can_devices[device_pid->device_parent_id]);
    uint32_t stdid_from_device = can_device_ptr->id.response;

    CAN_Packet packet;
    CAN_TxHeaderTypeDef tx_header;

    if (!__sim__create_read_response_packet_for_pid(device_pid, &packet, &tx_header, stdid_from_device, signal_payload)) {
        log_message("* Error generating SIM Tx Packet for command: RESP_READ_VALUE!");
        return;
    }

    packet.header.dlc = tx_header.DLC;
    packet.flow = PACKET_TX;
    packet.meta.can_instance = CAN_TRUCK;
    packet.meta.timestamp = HAL_GetTick();
    packet.header.filter_match = 0;
    packet.header.id = tx_header.StdId;
    packet.header.is_extended_id = false;
    packet.header.is_remote_frame = false;
    packet.header.timestamp = packet.meta.timestamp;

    log_raw_can_packet(&packet);

    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, packet.payload, &tx_mailbox) != HAL_OK) {
        log_message("* Failed to transmit CAN packet!");
    }
}



void __sim__generate_multiplexed_payload(CANDevicePID *device_pid, uint8_t *multiplexed_payload) {
    // Initialize the payload to all zeros
    for (int i = 0; i < MAX_PAYLOAD_BYTE_LENGTH; i++) {
        multiplexed_payload[i] = 0x00;
    }

    // Iterate through each signal in the PID
    for (uint8_t signal_idx = 0; signal_idx < device_pid->num_of_signals; signal_idx++) {
        const CANSignal *signal = &device_pid->signals[signal_idx];

        // Check if the signal's data is non-zero
        if (signal->data != 0) {
            // Apply the relevant "state_on" bytes to the multiplexed payload
            for (uint8_t byte_idx = 0; byte_idx < signal->relevant_data_bytes; byte_idx++) {
                multiplexed_payload[byte_idx] |= signal->state_on[byte_idx];
            }
        }
    }
}

/**
 * @brief Generate a simulated CAN packet response from the truck based on PID and command.
 *
 * This function simulates the behavior of a truck's response to a CAN request by evaluating
 * the current state of signals associated with a given PID. For `MULTIPLEXED_BYTE` generation,
 * it computes a single-byte response by iterating through all signals in the PID and setting
 * bits based on their "on" state.
 *
 * @param device_pid Pointer to the `CANDevicePID` structure representing the PID to process.
 * @param command The command type (e.g., REQ_READ) to process.
 */
void __sim__generate_packet_response_from_truck(CANDevicePID *device_pid, CAN_Command command)
{
	CANCommands *request_command = get_can_request_command(command);

	if (request_command == NULL)
	{
        log_message("Unknown CAN command: 0x%02X\n", command);
        return;
	}

    switch (request_command->byte)
    {
    case REQ_READ_VALUE: {
        log_message("* CAN Command Received: " BWHT "%02X" CRESET ": " BWHT "%s" CRESET,
                    request_command->byte, request_command->name);

        switch (device_pid->state_generation_type) {
            case CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE: {
                uint8_t signal_payload[MAX_PAYLOAD_BYTE_LENGTH] = {0};

                // Non-multiplexed case: Use only the first signal's state
                const CANSignal *signal = &device_pid->signals[0];
                if (signal->data != 0) {
                    for (uint8_t i = 0; i < signal->relevant_data_bytes; i++) {
                        signal_payload[i] = signal->state_on[i];
                    }
                }

                __sim__send_tx_packet_response_to_can1(device_pid, signal_payload);
                break;
            }

            case CAN_STATE_GENERATION_MULTIPLEXED_BYTE: {
                uint8_t signal_payload[MAX_PAYLOAD_BYTE_LENGTH] = {0};

                for (uint8_t i = 0; i < device_pid->num_of_signals; i++) {
                    const CANSignal *signal = &device_pid->signals[i];
                    if (signal->data != 0) {
                        for (uint8_t j = 0; j < signal->relevant_data_bytes; j++) {
                            signal_payload[j] |= signal->state_on[j];
                        }
                    }
                }

                __sim__send_tx_packet_response_to_can1(device_pid, signal_payload);
                break;
            }

            default:
                log_message("* " RED "Unknown state generation type." CRESET);
                break;
        }
        break;
    }

        default:
            log_message("* " RED "CAN Command not implemented for response: " BWHT "0x%02X" CRESET ": " BWHT "%s" CRESET, command, request_command->name);
            break;
    }
}


#endif /* IS_SIMULATOR */
