/*
 * can_core.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This header declares functions and data structures for managing CAN communication,
 * including initialization, message handling, and device-specific configurations.
 */

#ifndef CAN_CORE_H_
#define CAN_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>             // Fixed-width integer types
#include <stdbool.h>            // Boolean support

#include "config.h"
#include "device_configs.h"     // CANDeviceConfig, CANDevicePID, etc.
#include "can_common.h"         // Common CAN utilities
#include "rtos.h"               // RTOS utilities

/* --------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */
#define DLC_MAX 8                                   /**< Maximum data length for standard CAN frames */
#define MAX_RETRIES 3                               /**< Maximum retries for CAN message transmission */

/* --------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

/* --- Initialization and Setup Functions --- */

/**
 * @brief Initialize and start all CAN buses.
 */
void start_hal_can_buses(void);

/**
 * @brief Start a specific CAN bus and log the result.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @param hcan_human_string Human-readable description of the CAN bus.
 */
void start_hal_can_bus(CAN_HandleTypeDef *hcan, const char *hcan_human_string);

/* --- CAN Packet Queue Management --- */

/**
 * @brief Get the queue handle associated with a given queue number.
 *
 * @param queue_num The circular queue type.
 * @return The queue handle.
 */
osMessageQueueId_t get_queue_handle_by_queue_num(Circular_Queue_Types queue_num);

/**
 * @brief Get the queue handle for a specific CAN instance and flow direction.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param flow_dir The flow direction (e.g., QUEUE_TYPE_FLOW_TX or QUEUE_TYPE_FLOW_RX).
 * @return The queue handle, or NULL if invalid.
 */
osMessageQueueId_t get_queue_handle_by_can_instance(CANInstance can_instance, Queue_Type_Flow flow_dir);

/**
 * @brief Get the queue number associated with a specific CAN instance and flow direction.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param queue_type The queue type for the flow (e.g., QUEUE_TYPE_FLOW_TX).
 * @return The circular queue type.
 */
Circular_Queue_Types get_queue_num_by_can_instance(CANInstance can_instance, Queue_Type_Flow queue_type);

/* --- CAN Packet Transmission Functions --- */

/**
 * @brief Send a CAN packet to the transmission queue.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid Pointer to the CANDevicePID structure.
 * @param verb The CAN verb type (e.g., request or reply).
 */
void send_can_packet_to_tx_queue(CANInstance can_instance, CANDeviceConfig *device, CANDevicePID *pid, CAN_Verb_Type verb);

void activate_can_bus_fifo_callbacks(void);

/**
 * @brief Sends a CAN packet using the HAL CAN API.
 *
 * Handles CAN-specific operations like header conversion, message transmission,
 * and logging.
 *
 * @param packet Pointer to the `CAN_Packet` containing metadata, header, and payload.
 * @return true if the packet was successfully transmitted, false otherwise.
 */
bool _send_tx_packet_to_can_interface(CAN_Packet *packet);

/**
 * @brief Process and send all CAN requests.
 */
void send_all_requests(void);

/* --- CAN Packet Reception and Processing Functions --- */

/**
 * @brief Process signal changes for a given PID and payload.
 *
 * Iterates through the signals within the specified PID and checks for changes
 * based on the provided payload. Updates the signal state if a change is detected.
 *
 * @param device_pid Pointer to the `CANDevicePID` containing the signals.
 * @param payload The received payload data to compare against.
 */
void process_signal_changes(CANDevicePID *device_pid, uint32_t payload);

/**
 * @brief Retrieve a message from the CAN FIFO and populate a CAN_Packet.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @param can_instance The CAN instance.
 * @param packet Pointer to the CAN_Packet structure to populate.
 * @return True if successful, false otherwise.
 */
bool get_rx_message_from_CAN_RX_FIFO0(CAN_HandleTypeDef *hcan, CANInstance can_instance, CAN_Packet *packet);

/**
 * @brief Process a received CAN packet.
 *
 * @param queue_enum The circular queue type.
 * @param can_instance_enum The CAN instance enum.
 * @param packet Pointer to the CAN_Packet.
 */
void process_can_rx_packet(Circular_Queue_Types queue_enum, CANInstance can_instance_enum, CAN_Packet *packet);

/* --- CAN Header and Payload Management --- */

/**
 * @brief Configure a CAN Tx header.
 *
 * @param tx_header Pointer to the CAN_TxHeaderTypeDef structure.
 * @param std_id The standard identifier for the CAN message.
 */
void create_can_tx_header(CAN_TxHeaderTypeDef *tx_header, uint32_t std_id);

/**
 * @brief Generate a CAN payload for a specific device and PID.
 *
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid Pointer to the CANDevicePID structure.
 * @param TxData Pointer to the buffer where the payload will be stored.
 */
void generate_can_tx_read_data_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData);

/* --- Device and PID Configuration Functions --- */

/**
 * @brief Retrieve a device configuration by its CAN ID.
 *
 * @param stdid The CAN ID.
 * @param flow The packet flow direction.
 * @return Pointer to the CANDeviceConfig structure, or NULL if not found.
 */
CANDeviceConfig *get_device_config_by_id(uint32_t stdid);

/**
 * @brief Retrieve a PID configuration from a device by PID.
 *
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid The PID to search for.
 * @return Pointer to the CANDevicePID structure, or NULL if not found.
 */
CANDevicePID *get_pid_by_id(CANDeviceConfig *device, uint16_t pid);

/**
 * @brief Determine if a CAN message should be ignored.
 *
 * Filters out non-critical messages like heartbeats or other irrelevant data.
 *
 * @param can_id The CAN ID of the message.
 * @return True if the message should be ignored, false otherwise.
 */
bool should_ignore_message(uint32_t can_id);

bool is_signal_on(const CANSignal *signal, uint32_t payload);

/* --- CAN Instance Mapping Functions --- */

/**
 * @brief Retrieve the CAN handle for the given CAN instance.
 *
 * @param instance The CANInstance (e.g., CAN_TRUCK or CAN_AUX).
 * @return Pointer to the corresponding HAL CAN handle.
 */
CAN_HandleTypeDef *get_hcan_from_instance(CANInstance instance);

/**
 * @brief Retrieve the CAN instance for the given hardware instance.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @return The CANInstance (e.g., CAN_TRUCK or CAN_AUX).
 */
CANInstance get_can_instance_enum(CAN_HandleTypeDef *hcan);

// Define a mapping array to map hardware instances to CANInstance indexes
CANInstance get_can_instance_from_hcan(CAN_HandleTypeDef *hcan);
/**
 * @brief Parses raw CAN data into a structured format.
 *
 * @param raw_data Pointer to the raw CAN data (8 bytes).
 * @param parsed_data Pointer to the parsed data structure.
 * @return true if parsing succeeds, false otherwise.
 */
bool parse_raw_can_data(const uint8_t *raw_data, Parsed_CAN_Data *parsed_data);

/**
 * @brief Validates parsed CAN data for integrity.
 *
 * @param parsed_data Pointer to the parsed data structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const Parsed_CAN_Data *parsed_data);


#ifdef __cplusplus
}
#endif

#endif /* CAN_CORE_H_ */
