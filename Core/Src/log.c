/*
 * log.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Ryan
 */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "log.h"
#include "ansi.h"

LogBuffer log_buffer = { .head = 0, .tail = 0, .count = 0 };

void init_log_system(void) {
    log_buffer.mutex_id = osMutexNew(NULL); // Create the mutex
    if (log_buffer.mutex_id == NULL) {
        // Handle mutex creation failure
    }

}

void log_message(const char *format, ...) {
    // Acquire the mutex
    if (osMutexAcquire(log_buffer.mutex_id, osWaitForever) != osOK) {
        return; // Mutex acquisition failed, skip logging
    }

    // Check if buffer is full
    if (log_buffer.count >= LOG_BUFFER_SIZE) {
        osMutexRelease(log_buffer.mutex_id); // Release the mutex
        return; // Buffer is full, drop the log message
    }

    // Prepare the log message
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer.messages[log_buffer.head], LOG_MESSAGE_MAX_LENGTH, format, args);
    va_end(args);

    // Update the buffer
    log_buffer.head = (log_buffer.head + 1) % LOG_BUFFER_SIZE;
    log_buffer.count++;

    // Release the mutex
    osMutexRelease(log_buffer.mutex_id);
}

void __rtos__log_task(void) {
	char batch_buffer[LOG_BUFFER_SIZE * LOG_MESSAGE_MAX_LENGTH];
	size_t batch_length = 0;

	for (;;) {
		batch_length = 0;

		// Acquire the mutex
		if (osMutexAcquire(log_buffer.mutex_id, osWaitForever) == osOK) {
			while (log_buffer.count > 0 && batch_length < sizeof(batch_buffer)) {
				// Append the oldest message to the batch
				size_t len = strlen(log_buffer.messages[log_buffer.tail]);
				memcpy(&batch_buffer[batch_length], log_buffer.messages[log_buffer.tail], len);
				batch_length += len;

				// Add a newline for readability
				batch_buffer[batch_length++] = '\n';
				batch_buffer[batch_length++] = '\r';


				// Update the buffer
				log_buffer.tail = (log_buffer.tail + 1) % LOG_BUFFER_SIZE;
				log_buffer.count--;
			}

			osMutexRelease(log_buffer.mutex_id);
		}

		if (batch_length > 0) {
			HAL_UART_Transmit(&huart2, (uint8_t *)batch_buffer, batch_length, HAL_MAX_DELAY);
		}

		osDelay(10); // Delay to avoid hogging the CPU
	}
}

void display_welcome_message(void)
{
	printf(CLR); // Clear screen and reset cursor position

	// Header
	printf(BBLU "====================================================================\r\n");
	printf(BBLU "|" BYEL "                 FORD SUPERDUTY TRUCK CAN BUS GATEWAY               " BBLU "|\r\n");
	printf(BBLU "====================================================================" CRESET "\r\n");

	// Spacer
	printf("\r\n");

	// Status Messages
	printf(BGRN "* " CRESET GRN "STM32 initialized" CRESET "\r\n");
	printf(BGRN "* " CRESET GRN "Logging system initialized" CRESET "\r\n");

	// Footer
	printf("\r\n" BBLU "====================================================================" CRESET "\r\n");

}
