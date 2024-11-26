/*
 * sim.c
 *
 *  Created on: Nov 23, 2024
 *      Author: Ryan
 */

// only load this file if in the SIMULATOR mode

#include <stdbool.h>
#include <stdio.h>

#include "sim.h"
#include "config.h"
#include "device_configs.h"
#include "can_core.h"

#include "log.h"

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
#ifdef IS_SIMULATOR

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
void __sim__generate_packet_response_from_truck(CANDevicePID *device_pid, CAN_Command command) {
    switch (command) {
        case REQ_READ: // Process Read Data by Identifier
            switch (device_pid->state_generation_type) {

                case CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE:
                    // Single signal logic for non-multiplexed byte generation
                    if (device_pid->num_of_signals == 1) {
                        // Check the single signal's state
                        bool is_state_on = is_signal_on(&device_pid->signals[0]);
                        log_message("State is %s\n", is_state_on ? "ON" : "OFF");
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
                    log_message("Multiplexed Byte: 0x%02X\n", multiplexed_byte);
                } break;

                default:
                    log_message("Unknown state generation type.\n");
                    break;
            }
            break;

        default:
            log_message("Unknown CAN command.\n");
            break;
    }
}





#endif /* IS_SIMULATOR */
