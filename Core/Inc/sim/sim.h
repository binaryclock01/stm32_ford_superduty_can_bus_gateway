/*
 * sim.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Ryan
 */

// only load this file if in the SIMULATOR mode

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "device_configs.h"

#ifndef INC_SIM_H_
#define INC_SIM_H_

#ifdef IS_SIMULATOR

typedef struct {
	bool hazard_button;
	bool reverse_lamp;
	bool brake_pedal;
	bool turn_signal_multiplexed_byte;
} _SIM_Signal_States;

typedef struct {
	uint8_t turn_left_state;
	uint8_t right_left_state;
	uint8_t turn_left_change_state;
	uint8_t turn_right_change_state;
} _SIM_Turn_Signal_States;


extern _SIM_Signal_States __sim__g_signal_states;
extern _SIM_Turn_Signal_States __sim__g_turn_signal_states;

// maps each simulated state above to the global variable
// this is used in void multiplex_turn_signal_states()
#define TURN_SIGNAL_COUNT 4


void __sim__generate_packet_response_from_truck(CANDevicePID *device_pid, CAN_Command command);
void __sim__initialize_states_mapping_array();

#endif /* INC_SIM_H_ */

#endif /* IS_SIMULATOR */
