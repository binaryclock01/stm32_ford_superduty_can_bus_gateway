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

/**
 * @brief Creates a simulated READ response packet for the given PID.
 *
 * @param device_pid Pointer to the CANDevicePID structure representing the PID.
 * @param packet Pointer to the CAN_Tx_Packet structure to store the generated packet.
 * @return True if the packet was successfully created; false otherwise.
 */
bool __sim__create_read_response_packet_for_pid(CANDevicePID *device_pid, CAN_Packet *packet, CAN_TxHeaderTypeDef *tx_header, CAN_StdIds stdid)
{
    if (device_pid == NULL || packet == NULL) {
        log_message("Error: Null device_pid or packet in __sim__create_read_response_packet_for_pid");
        return false;
    }

    // Validate relevant_data_bytes
    if (device_pid->signals[0].relevant_data_bytes > 4 || device_pid->signals[0].relevant_data_bytes < 1) {
        log_message("Error: Invalid relevant_data_bytes (%d)", device_pid->signals[0].relevant_data_bytes);
        return false;
    }

    // Set the total data length
    packet->payload[DATA_LENGTH_START] = (DATA_PAYLOAD_START - 1) + device_pid->signals[0].relevant_data_bytes;

    // Set the response command
    packet->payload[DATA_COMMAND_START] = RESP_READ_VALUE;

    // Copy the PID
    for (int i = 0; i < PID_BYTE_LENGTH; i++) {
        packet->payload[DATA_PID_START + i] = device_pid->pid_id[i];
    }

    // Determine which state array to use
    const uint8_t *pid_state_array = NULL;

    if (get_signal_data(&device_pid->signals[0])) {
        pid_state_array = device_pid->signals[0].state_on;
    } else {
        pid_state_array = device_pid->signals[0].state_off;
    }

    // Copy the state array into the payload
    // just do 4. if we don't do 4, then we'll get garbage on the last bytes. In device_configs if bytes are not used
    // Ford just sends 0xCC. Those 0xCC codes were copied into the state data arrays too, so just copy the entire thing.
    for (int i = 0; i < 4; i++) {
        packet->payload[DATA_PAYLOAD_START + i] = pid_state_array[i];
    }

    create_can_tx_header(tx_header, stdid);

    return true;
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
        case REQ_READ_VALUE: // Process Read Data by Identifier
        	log_message("* CAN Command Received: " BWHT "%02X" CRESET ": " BWHT "%s" CRESET, request_command->byte, request_command->name);
            switch (device_pid->state_generation_type)
            {
                case CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE:
                    // Single signal logic for non-multiplexed byte generation
                    if (device_pid->num_of_signals == 1) {
                        // Check the single signal's state
                        bool is_state_on = (get_signal_data(&device_pid->signals[0]) > 0 ? true : false);

                        log_message("* State of " BWHT "%s " CRESET "is: " BWHT "%s" CRESET, device_pid->name, is_state_on ? "ON" : "OFF");
                    }

                    CAN_Packet packet;
                    CAN_TxHeaderTypeDef tx_header;
                    uint32_t stdid_from_device = CAN_ID_BCM_RESPONSE;

                    if (!__sim__create_read_response_packet_for_pid(device_pid, &packet, &tx_header,stdid_from_device))
                    {
                    	log_message("* Error generating SIM Tx Packet for command: RESP_READ_VALUE!");
                    	return;
                    }

                    packet.header.dlc 	= tx_header.DLC;
                    packet.flow 		= PACKET_TX;
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
                    } else {
                        //log_message("* Successfully transmitted CAN packet.");
                    }

                    break;

                case CAN_STATE_GENERATION_MULTIPLEXED_BYTE: {
                    /**
                     * MULTIPLEXED_BYTE logic:
                     * - Iterate through all signals in the PID.
                     * - Use a turn signal bitmask to determine the relevant bit for each signal.
                     * - Mask the signal's `data` with both the bitmask and the relevant bytes mask.
                     * - Compare the result with the predefined `state_on` to check if the signal is "on."
                     * - If the signal is "on," set the corresponding bit in the multiplexed byte.
                     */

                    uint8_t multiplexed_byte = 0x00; // Initialize response byte to 0

                    for (uint8_t i = 0; i < device_pid->num_of_signals; i++) {
                        const CANSignal *signal = &device_pid->signals[i];

                        // 1. Retrieve the bitmask for this signal
                        uint64_t bitmask = _g_turn_signal_bitmask_mapping_array[i];

                        // 2. Apply the relevant byte mask to isolate only relevant data bytes
                        uint32_t relevant_mask = get_relevant_mask(signal->relevant_data_bytes);

                        // 3. Mask the signal's `data` with `bitmask` and `relevant_mask`
                        uint64_t masked_data = signal->data & relevant_mask & ~bitmask;

                        /**
                         * 4. Compute `signal_on_masked`:
                         * - Mask the predefined `state_on` value with the `relevant_mask` and `bitmask`.
                         * - This ensures `state_on` is precisely aligned with the relevant bits in `data`.
                         */
                        uint32_t signal_on_masked = (*((uint32_t *)signal->state_on) & relevant_mask & bitmask);

                        /**
                         * 5. Final Check:
                         * Compare the masked `data` to the masked `state_on` value.
                         * - Ensures that only the relevant bits are evaluated.
                         * - Prevents mismatches due to unrelated or reserved bits.
                         */
                        if (masked_data == signal_on_masked) {
                            // Signal is "on": Set the corresponding bit in the multiplexed byte
                            multiplexed_byte |= ~bitmask;
                        }
                    }

                    // Output the computed multiplexed byte
                    log_message("* State of " BWHT "%s " CRESET "is: " BWHT "%08X" CRESET, device_pid->name, multiplexed_byte);
                } break;

                default:
                    log_message("* " RED "Unknown state generation type." CRESET);
                    break;
            }
            break;

        default:
            log_message("* " RED "CAN Command not implemented for response: " BWHT "0x%02X" CRESET ": " BWHT "%s" CRESET, command, request_command->name);
            break;
    }
}


#endif /* IS_SIMULATOR */
