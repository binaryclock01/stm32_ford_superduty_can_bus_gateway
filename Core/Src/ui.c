/*
 * ui.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */

#include <device_configs.h>  // Include the configurations for devices
#include <string.h>          // For memset, strncpy
#include <stdio.h>           // For snprintf
#include <stdbool.h>         // For boolean support

#include "main.h"
#include "ui.h"
#include "ssd1306_fonts.h"
#include "utils.h"
#include "can.h"


/* ---| DISPLAY VARIABLES |-------------------------------------------------------------------- */
char screen_data[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
char screen_data_states[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
uint8_t screen_line = 0;


/**
 * @brief Initialize OLED screen data buffers.
 *
 * Resets the screen data and state buffers to empty, and sets the current
 * message line to the top of the message area.
 */
void init_oled_data() {
    screen_line = 0; // Reset the current message line index
    memset(screen_data, 0, sizeof(screen_data));           // Clear message data buffer
    memset(screen_data_states, 0, sizeof(screen_data_states)); // Clear state data buffer
}

/**
 * @brief Scroll existing messages up by one line.
 *
 * Shifts all message lines up by one, making room for a new message at the bottom.
 * The last message line is cleared.
 */
void scroll_messages() {
    // Shift all message lines upward
    for (int i = 0; i < MESSAGE_LINES - 1; i++) {
        strncpy(screen_data[STATE_LINES + i], screen_data[STATE_LINES + i + 1], sizeof(screen_data[0]) - 1);
    }

    // Clear the last message line
    memset(screen_data[STATE_LINES + MESSAGE_LINES - 1], 0, sizeof(screen_data[0]));
}

/**
 * @brief Send a message to the console and update the OLED display.
 *
 * Adds a new message to the OLED message area. If the message area is full,
 * older messages are scrolled up to make room.
 *
 * @param msg Pointer to the null-terminated message string to be displayed.
 */
void send_console_msg(const char *msg) {
    // Check if the message buffer is full and scroll if necessary
    if (screen_line >= MESSAGE_LINES) {
        scroll_messages();
        screen_line = MESSAGE_LINES - 1; // Clamp the line index to the maximum
    }

    // Add the new message to the message buffer
    strncpy(screen_data[STATE_LINES + screen_line], msg, sizeof(screen_data[0]) - 1);
    screen_data[STATE_LINES + screen_line][sizeof(screen_data[0]) - 1] = '\0'; // Ensure null-termination

    // Update the display with the current states and messages
    draw_screen_data_states(CAN_TRUCK); // Draw state information
    display_messages();                // Update the message area

    // Move to the next message line
    screen_line++;
}

/**
 * @brief Display messages stored in the message buffer.
 *
 * Clears the message area of the OLED screen and writes all messages from
 * the buffer to the screen.
 */
void display_messages() {
    // Clear the message display area (below the state area)
    ssd1306_FillRectangle(0, STATE_LINES * SCREEN_FONT_HEIGHT, SSD1306_WIDTH, SSD1306_HEIGHT, Black);

    // Write each message line to the display
    for (uint8_t i = 0; i < MESSAGE_LINES; i++) {
        uint8_t y_pos = (STATE_LINES + i) * SCREEN_FONT_HEIGHT; // Calculate vertical position
        ssd1306_SetCursor(0, y_pos);
        ssd1306_WriteString(screen_data[STATE_LINES + i], Font_6x8, White);
    }

    // Apply changes to the OLED screen
    ssd1306_UpdateScreen();
}

/**
 * @brief Draw state data for the CAN instance on the OLED screen.
 *
 * Updates the state information area of the OLED screen based on the current
 * states of the CAN signals and transmission/reception counts.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX) to display data for.
 */
void draw_screen_data_states(CANInstance can_instance) {
	CANData *selected_can_data = &can_data[can_instance];

    // Clear the state display area (top of the screen)
    ssd1306_FillRectangle(0, 0, SSD1306_WIDTH, STATE_LINES * SCREEN_FONT_HEIGHT, Black);

    // Line 0: Brake Pedal, Reverse Light, Hazard Button states
    snprintf(screen_data_states[0], sizeof(screen_data_states[0]),
             "%s:%d %s:%d %s:%d",
             bcm_pids[BCM_BRAKE_PEDAL].short_name, tobinary(bcm_pids[BCM_BRAKE_PEDAL].signals[0].data),
             bcm_pids[BCM_REVERSE_LIGHT].short_name, tobinary(bcm_pids[BCM_REVERSE_LIGHT].signals[0].data),
             bcm_pids[BCM_HAZARD_BUTTON].short_name, tobinary(bcm_pids[BCM_HAZARD_BUTTON].signals[0].data));

    // Line 1: Left Turn Signal and Left Lane Change states
    snprintf(screen_data_states[1], sizeof(screen_data_states[1]),
             "%s:%d %s:%d",
             sccm_pids[0].signals[SCCM_LEFT_TURN].short_name, tobinary(sccm_pids[0].signals[SCCM_LEFT_TURN].data),
             sccm_pids[0].signals[SCCM_LEFT_CHANGE].short_name, tobinary(sccm_pids[0].signals[SCCM_LEFT_CHANGE].data));

    // Line 2: Right Turn Signal and Right Lane Change states
    snprintf(screen_data_states[2], sizeof(screen_data_states[2]),
             "%s:%d %s:%d",
             sccm_pids[0].signals[SCCM_RIGHT_TURN].short_name, tobinary(sccm_pids[0].signals[SCCM_RIGHT_TURN].data),
             sccm_pids[0].signals[SCCM_RIGHT_CHANGE].short_name, tobinary(sccm_pids[0].signals[SCCM_RIGHT_CHANGE].data));

    // Line 3: Transmission (Tx) and Reception (Rx) counts
    snprintf(screen_data_states[3], sizeof(screen_data_states[3]), "Tx:%lu Rx:%lu",
             (unsigned long)selected_can_data->tx_count, (unsigned long)selected_can_data->rx_count);

    // Write each state line to the OLED display
    for (uint8_t i = 0; i < STATE_LINES; i++) {
        ssd1306_SetCursor(0, i * SCREEN_FONT_HEIGHT);
        ssd1306_WriteString(screen_data_states[i], Font_6x8, White);
    }

    // Apply changes to the OLED screen
    ssd1306_UpdateScreen();
}

