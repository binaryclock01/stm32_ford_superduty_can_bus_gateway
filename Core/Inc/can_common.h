/*
 * can_common.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This file contains common definitions and data structures for CAN communication,
 * shared across various modules such as CAN core, UI, and error handling.
 */

#ifndef CAN_COMMON_H_
#define CAN_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // Fixed-width integer type
#include <stdbool.h>

#include "main.h"
//#include "device_configs.h"
#include "config.h"
#include "can_packet.h"

/* -----------------------------------------------------------------------------
   Constants
   -------------------------------------------------------------------------- */



/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

/**
 * @brief Enum to represent CAN instances for modularity and clarity.
 */





/* -----------------------------------------------------------------------------
   Typedefs
   -------------------------------------------------------------------------- */







#ifdef __cplusplus
}
#endif

#endif /* CAN_COMMON_H_ */
