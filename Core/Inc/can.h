/*
 * can.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 */

#ifndef CAN_H_
#define CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              // For uint8_t, uint16_t, uint32_t, uint64_t
#include <stdbool.h>             // For boolean support
#include "device_configs.h"      // For CANDeviceConfig, CANDevicePID, etc.
#include "main.h"                // For HAL_CAN definitions
#include "ui.h"                  // For send_Console_Msg()
#include "utils.h"               // For utility functions
#include "error.h"               // For error handling

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/**
 * @brief Build a CAN request payload for a specific device and PID.
 * @param device Pointer to the CANDeviceConfig representing the device.
 * @param pid Pointer to the CANDevicePID representing the PID to request.
 * @return A 64-bit value representing the built CAN request payload.
 */
uint64_t build_can_request(CANDeviceConfig *device, CANDevicePID *pid);

/**
 * @brief Send a CAN request for a specific device and PID.
 * @param can_instance CAN instance (e.g., CAN1, CAN2).
 * @param device Pointer to the CANDeviceConfig representing the device.
 * @param pid Pointer to the CANDevicePID representing the PID to request.
 */
void send_can_request(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid);

/**
 * @brief Send a CAN request for all configured devices and their PIDs.
 */
void send_all_can_requests(void);

/**
 * @brief Retrieve a CANDeviceConfig by its CAN ID.
 * @param rx_id The received CAN ID.
 * @return Pointer to the matching CANDeviceConfig, or NULL if not found.
 */
CANDeviceConfig *get_CANDeviceConfig_by_canid(uint32_t rx_id);

/**
 * @brief Retrieve a CANDevicePID by its PID from a specific device.
 * @param module Pointer to the CANDeviceConfig representing the device.
 * @param pid The PID to look for.
 * @return Pointer to the matching CANDevicePID, or NULL if not found.
 */
CANDevicePID *get_CANDevicePID_by_pid(CANDeviceConfig *module, uint16_t pid);

/**
 * @brief Iterate over the signals of a PID and check for changes based on the received payload.
 * @param device_pid Pointer to the CANDevicePID containing the signals.
 * @param payload The received payload data to compare against.
 */
void iterate_signals_for_changes(CANDevicePID *device_pid, uint32_t payload);

/**
 * @brief Parse a received CAN message and handle it.
 * @param RAW_rx_id The received CAN ID.
 * @param RAW_rx_data_as_byte_array The received CAN data as an array of bytes.
 */
void parse_rx_CAN_message(uint32_t RAW_rx_id, uint8_t *RAW_rx_data_as_byte_array);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */
