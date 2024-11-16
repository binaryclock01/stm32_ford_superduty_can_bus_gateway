/*
 * ui.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */


#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdbool.h> // for boolean support in c

#include "main.h"
#include "ui.h"
#include "ssd1306_fonts.h"
#include "utils.h"

void init_OLED_Data()
{
	screen_line = 0;
	memset(screen_data, 0, sizeof(screen_data));
	memset(screen_data_states, 0, sizeof(screen_data_states));
}

void scroll_messages() {
    // Shift all message lines up by one
    for (int i = 0; i < MESSAGE_LINES - 1; i++) {
        strcpy(screen_data[STATE_LINES + i], screen_data[STATE_LINES + i + 1]);
    }
    // Clear the last line
    memset(screen_data[SCREEN_MAX_CHAR_LINES - 1], 0, sizeof(screen_data[SCREEN_MAX_CHAR_LINES - 1]));
}


void send_Console_Msg(char *msg) {
    // Scroll if all message lines are filled
    if (screen_line >= MESSAGE_LINES) {
        scroll_messages();
        screen_line = MESSAGE_LINES - 1; // Keep screen_line within MESSAGE_LINES
    }

    // Copy the message to the appropriate line in screen_data
    strncpy(screen_data[STATE_LINES + screen_line], msg, sizeof(screen_data[0]) - 1);
    screen_data[STATE_LINES + screen_line][sizeof(screen_data[0]) - 1] = '\0'; // Ensure null-termination

    // Update the screen with both states and messages
    draw_screen_data_states(CAN_TRUCK);
    display_messages();

    // Move to the next line for future messages
    screen_line++;
}

void display_messages() {
    // Clear the entire bottom section where messages are displayed
    ssd1306_FillRectangle(0, STATE_LINES * SCREEN_FONT_HEIGHT, SSD1306_WIDTH, SSD1306_HEIGHT, Black);

    // Start from the line after the state info and print messages
    for (uint8_t i = 0; i < MESSAGE_LINES; i++) {
        uint8_t y_pos = (STATE_LINES + i) * SCREEN_FONT_HEIGHT;
        ssd1306_SetCursor(0, y_pos);
        ssd1306_WriteString(screen_data[STATE_LINES + i], Font_6x8, White);
    }

    ssd1306_UpdateScreen();  // Update the screen to reflect changes
}

void draw_screen_data_states(CANInstance can_instance) {
    // Clear the screen area for the state display
    ssd1306_FillRectangle(0, 0, SSD1306_WIDTH, STATE_LINES * SCREEN_FONT_HEIGHT, Black);

    // Display BCM states (Brake Pedal, Reverse Light, Hazard Button)
    snprintf(screen_data_states[0], sizeof(screen_data_states[0]),
             "%s:%d %s:%d %s:%d",
             bcm_pids[BCM_BRAKE_PEDAL].short_name, tobinary(bcm_pids[BCM_BRAKE_PEDAL].signals[0].data),
             bcm_pids[BCM_REVERSE_LIGHT].short_name, tobinary(bcm_pids[BCM_REVERSE_LIGHT].signals[0].data),
             bcm_pids[BCM_HAZARD_BUTTON].short_name, tobinary(bcm_pids[BCM_HAZARD_BUTTON].signals[0].data));

    // Display SCCM left-side signals (Left Turn Signal, Left Lane Change)
    snprintf(screen_data_states[1], sizeof(screen_data_states[1]),
             "%s:%d %s:%d",
             sccm_pids[0].signals[SCCM_LEFT_TURN].short_name, tobinary(sccm_pids[0].signals[SCCM_LEFT_TURN].data),
             sccm_pids[0].signals[SCCM_LEFT_CHANGE].short_name, tobinary(sccm_pids[0].signals[SCCM_LEFT_CHANGE].data));

    // Display SCCM right-side signals (Right Turn Signal, Right Lane Change)
    snprintf(screen_data_states[2], sizeof(screen_data_states[2]),
             "%s:%d %s:%d",
             sccm_pids[0].signals[SCCM_RIGHT_TURN].short_name, tobinary(sccm_pids[0].signals[SCCM_RIGHT_TURN].data),
             sccm_pids[0].signals[SCCM_RIGHT_CHANGE].short_name, tobinary(sccm_pids[0].signals[SCCM_RIGHT_CHANGE].data));

    // Display transmission and reception counts
    snprintf(screen_data_states[3], sizeof(screen_data_states[3]), "Tx:%lu Rx:%lu",
             (unsigned long)tx_count[can_instance], (unsigned long)rx_count[can_instance]);

    // Write each line to the screen
    for (uint8_t i = 0; i < STATE_LINES; i++) {
        ssd1306_SetCursor(0, i * SCREEN_FONT_HEIGHT);
        ssd1306_WriteString(screen_data_states[i], Font_6x8, White);
    }

    // Update the screen to display changes
    ssd1306_UpdateScreen();
}

