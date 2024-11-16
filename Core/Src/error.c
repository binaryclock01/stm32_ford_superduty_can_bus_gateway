/*
 * error.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Ryan
 */
#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdbool.h> // for boolean support in c
#include "error.h"
#include "ui.h"

void User_Error_Handler(uint8_t error_code)
{
	const char *error_prefix = "ER";
	char error_msg[MAX_ERROR_STRING_LENGTH];
	char error_msg_final[MAX_ERROR_STRING_LENGTH];
	bool call_main_error_handler = true;

	// clear error_msg buffer
	memset(error_msg, '\0', sizeof(error_msg));
	memset(error_msg_final, '\0', sizeof(error_msg_final));

	switch (error_code)
	{
		case ERROR_CAN_MODULE_NOT_FOUND:
	    	snprintf(error_msg, MAX_ERROR_STRING_LENGTH, "CAN mod not found");
			break;

		case ERROR_MODULE_PID_NOT_FOUND:
	    	snprintf(error_msg, MAX_ERROR_STRING_LENGTH, "PID not found");
			break;

		case ERROR_NO_ERROR:
			call_main_error_handler = false;
			break;

		default:
			snprintf(error_msg, MAX_ERROR_STRING_LENGTH, "Unknown Err#");
	}

	// print the error string if not empty
	if (strlen(error_msg) != 0)
	{
		snprintf(error_msg_final, MAX_ERROR_STRING_LENGTH, "%s#%u %s", error_prefix, (uint8_t)error_code, error_msg);
		send_Console_Msg(error_msg_final);
	}

	// This will halt operation
	if (call_main_error_handler)
		Error_Handler();
}
