/*
 * log.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Ryan
 */

#include <stdint.h>
#include "main.h"


#ifndef INC_LOG_H_
#define INC_LOG_H_

#define LOG_BUFFER_SIZE 256
#define LOG_MESSAGE_MAX_LENGTH 128

typedef struct {
    char messages[LOG_BUFFER_SIZE][LOG_MESSAGE_MAX_LENGTH];
    uint16_t head; // Write position
    uint16_t tail; // Read position
    uint16_t count; // Number of messages in the buffer
    osMutexId_t mutex_id; // Mutex for thread-safe access
} LogBuffer;

extern LogBuffer log_buffer;

void display_welcome_message(void);
void log_message(const char *format, ...);
void init_log_system(void);
void __rtos__log_task(void);

#endif /* INC_LOG_H_ */
