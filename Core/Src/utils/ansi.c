/*
 * ansi.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Ryan
 */

#include <stddef.h>

void strip_ansi_codes(char *output, const char *input, size_t max_length) {
    size_t j = 0;
    for (size_t i = 0; input[i] != '\0' && j < max_length - 1; i++) {
        if (input[i] == '\e') {
            // Skip ANSI escape sequences
            while (input[i] != '\0' && input[i] != 'm') {
                i++;
            }
        } else {
            output[j++] = input[i];
        }
    }
    output[j] = '\0'; // Null-terminate the stripped string
}
