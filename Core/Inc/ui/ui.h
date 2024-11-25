/*
 * ui.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * Header file for the UI module that handles OLED display updates,
 * including managing message scrolling, console output, and state information.
 */

#ifndef UI_H
#define UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For fixed-width integer types
#include <stdbool.h>             // For boolean support
#include "main.h"                // For global variables and constants
#include "device_configs.h"      // For CANInstance and related configurations
//#include "can_common.h"


/* ---| FONT CONSTANTS |----------------------------------------------------- */
#define SCREEN_FONT_HEIGHT 8   /**< Font height in pixels (Font_6x8). */
#define SCREEN_FONT_WIDTH  6   /**< Font width in pixels (Font_6x8). */

/* ---| DISPLAY CONSTANTS |-------------------------------------------------- */
//#define SCREEN_MAX_CHAR_LINES (SSD1306_HEIGHT / SCREEN_FONT_HEIGHT) /**< Max lines on screen height. */
//#define SCREEN_MAX_CHAR_WIDTH (SSD1306_WIDTH / SCREEN_FONT_WIDTH)   /**< Max chars on screen width. */
#define STATE_LINES 4           /**< Number of lines used for state info. */
//#define MESSAGE_LINES (SCREEN_MAX_CHAR_LINES - STATE_LINES) /**< Remaining lines for messages. */
#define DEFAULT_OLED_CAN_INSTANCE CAN_TRUCK // Default CAN instance for OLED updates

//#define USE_SSD1306

//extern char screen_data[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
//extern char screen_data_states[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
extern uint8_t screen_line;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Update the OLED screen with the current state data and messages.
 *
 * This function updates both the state information (top area) and the
 * message buffer (bottom area) of the OLED screen, ensuring a consistent
 * display of system status.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX) to display state data for.
 */
void update_oled_status(CANInstance can_instance);

/**
 * @brief Initialize the OLED display buffers and reset screen counters.
 *
 * This function resets the internal buffers used for storing messages and
 * state data displayed on the OLED. It also resets the screen line index,
 * ensuring new messages start at the top of the message section.
 */
void init_oled_data(void);

/**
 * @brief Scroll messages on the OLED display.
 *
 * Shifts all messages in the message section of the display up by one line.
 * The oldest message is discarded, and the last line is cleared to make room
 * for a new message.
 */
void scroll_messages(void);

/**
 * @brief Send a message to the console and update the OLED display.
 *
 * Adds a new message to the message buffer and displays it on the OLED screen.
 * If the buffer is full, older messages are scrolled up to make room.
 *
 * @param msg Pointer to the null-terminated message string to display.
 */
void send_console_msg(const char *msg);

/**
 * @brief Render all stored messages on the OLED display.
 *
 * Clears the message area of the OLED screen and writes all messages currently
 * stored in the buffer. This function is typically called after updating the
 * message buffer.
 */
void display_messages(void);

/**
 * @brief Render state data for a specific CAN instance on the OLED display.
 *
 * Displays state information for a given CAN instance, including signal states
 * and transmission/reception statistics. The state data is shown in the top
 * section of the OLED screen.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX) to display data for.
 */
void draw_screen_data_states(CANInstance can_instance);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */
