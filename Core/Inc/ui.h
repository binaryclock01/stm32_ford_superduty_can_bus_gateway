/*
 * ui.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 */

#ifndef UI_H
#define UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "main.h"                // For global variables and constants
#include <device_configs.h>      // For CANInstance and related configurations

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Initialize the OLED display data buffer and reset the screen line counter.
 */
void init_OLED_Data(void);

/**
 * @brief Scrolls the message display, shifting messages up by one line.
 */
void scroll_messages(void);

/**
 * @brief Sends a console message to the display and updates the message section.
 * @param msg Pointer to the message string to be displayed.
 */
void send_Console_Msg(char *msg);

/**
 * @brief Updates the OLED display with the current messages.
 */
void display_messages(void);

/**
 * @brief Draws state information for a specific CAN instance on the OLED display.
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) to display state information for.
 */
void draw_screen_data_states(CANInstance can_instance);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */
