/*
 * system.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Ryan
 */
#include "log.h"
#include "core_cm4.h"
#include "ansi.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

void reset_system(void) {
    // Clean up dynamically allocated resources
    destroy_log_buffer__heap__();

    // Trigger system reset
    NVIC_SystemReset();
}

// Prints the total available and free heap memory
void check_heap_usage(void) {
    size_t free_heap = xPortGetFreeHeapSize();
    printf("* Free Heap Size: " BWHT "%lu" CRESET " bytes\r\n", (unsigned long)free_heap);
}

void check_min_heap_usage(void) {
    size_t min_heap = xPortGetMinimumEverFreeHeapSize();
    printf("Minimum Free Heap Size Ever: " BWHT "%lu" CRESET " bytes\r\n", (unsigned long)min_heap);
}

void checkTaskStackUsage(void) {
    UBaseType_t unusedStackWords = uxTaskGetStackHighWaterMark(NULL); // NULL for current task
    log_message("* Unused stack space: " BWHT "%lu" CRESET " words (" BWHT "%lu " CRESET "bytes)\r\n", unusedStackWords, unusedStackWords * 4);

}
