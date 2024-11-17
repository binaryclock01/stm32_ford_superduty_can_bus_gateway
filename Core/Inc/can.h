/*
 * can.h
 *
 * Created on: Nov 15, 2024
 * Author: Ryan
 *
 * Description:
 * This file declares functions and data structures for managing CAN communication,
 * including request building, message handling, and signal processing.
 */

#ifndef CAN_H_
#define CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>             // Fixed-width integer types
#include <stdbool.h>            // Boolean support
#include "device_configs.h"     // CANDeviceConfig, CANDevicePID, etc.
#include "main.h"               // HAL_CAN definitions
#include "ui.h"                 // Console message utilities
#include "error.h"              // Error handling utilities
#include "utils.h"
#include "cmsis_os.h"        // RTOS CMSIS types, such as osMutedId_t

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */

#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */
#define DLC_MAX 8 /**< Maximum data length for standard CAN frames */

#define CAN_PACKET_POOL_SIZE 10  // Adjust size based on expected throughput
#define CAN_BUFFER_SIZE 64  // Adjust size as needed
#define MAX_RETRIES 3        // Maximum retries for CAN message transmission
#define RETRY_DELAY_MS 100   // Delay between retries in milliseconds
/* -----------------------------------------------------------------------------
   Enumerations
   -------------------------------------------------------------------------- */

/**
 * @brief Enum to represent CAN instances for modularity and clarity.
 */
typedef enum {
    CAN_TRUCK, /**< Truck CAN bus (CAN1) */
    CAN_AUX,   /**< Auxiliary CAN bus (CAN2) */
    CAN_TOTAL  /**< Total number of CAN instances */
} CANInstance;

/* -----------------------------------------------------------------------------
   Structures
   -------------------------------------------------------------------------- */

typedef struct {
    CAN_RxHeaderTypeDef header;  /**< CAN message header (ID, length, etc.) */
    uint8_t data[8];             /**< CAN message data (payload) */
    CANInstance can_instance;    /**< The CAN instance that received the message */
    uint32_t timestamp;          /**< Timestamp of when the packet was received */
} CANPacket;


typedef struct {
    CANPacket packets[CAN_BUFFER_SIZE]; /**< Circular buffer of CAN packets. */
    uint8_t head;                       /**< Index of the next write position. */
    uint8_t tail;                       /**< Index of the next read position. */
    uint8_t count;                      /**< Number of packets currently in the buffer. */
} CANBuffer;

/**
 * @brief Structure to manage CAN data and state.
 */
typedef struct {
    char name[100];                      /**< Name of the CAN instance (e.g., "TruckNet"). */
    CAN_TxHeaderTypeDef TxHeader;        /**< CAN transmission header. */
    CAN_RxHeaderTypeDef RxHeader;        /**< CAN reception header. */
    uint32_t TxMailbox;                  /**< CAN transmission mailbox identifier. */
    uint8_t RxData[DLC_MAX];             /**< Buffer for received CAN data. */
    uint8_t TxData[DLC_MAX];             /**< Buffer for transmitted CAN data. */
    uint32_t tx_count;                   /**< Counter for transmitted messages. */
    uint32_t rx_count;                   /**< Counter for received messages. */
} CANData;

typedef struct {
  CANInstance can_instance; // CAN1 or CAN2
  CANData data;             // CAN data structure
} CANMessage;

/**
 * @brief Structure to hold parsed CAN data.
 */
typedef struct {
    uint8_t data_length; /**< Length of the received CAN data. */
    uint16_t pid;        /**< Parsed PID from the CAN message. */
    uint32_t payload;    /**< Extracted payload from the CAN message. */
} ParsedCANData;

typedef struct {
    uint8_t used;              // 0: free, 1: allocated
    CANPacket packet;          // The actual CAN packet
} CANPacketPoolEntry;

/* -----------------------------------------------------------------------------
   Function Declarations
   -------------------------------------------------------------------------- */

void init_can_packet_pool(void);
void free_can_packet(CANPacket *packet);
CANPacket *allocate_can_packet(void);

/* --- Initialization and Setup Functions --- */


/**
 * @brief Get the message queue handle for a given CAN hardware instance.
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @return osMessageQueueId_t* Pointer to the corresponding message queue handle, or NULL if not found.
 */
osMessageQueueId_t* get_queue_handle_from_hcan(CAN_HandleTypeDef *hcan);
/**
 * @brief Setup the CAN Tx header with the given request ID.
 *
 * Initializes the CAN Tx header with standard values like ID, identifier type,
 * frame type, and data length.
 *
 * @param TxHeader Pointer to the CAN_TxHeaderTypeDef structure.
 * @param request_id The request ID to set in the header.
 */
void setup_can_tx_header(CAN_TxHeaderTypeDef *TxHeader, uint64_t request_id);

/**
 * @brief Generate a CAN payload for the specified device and PID.
 *
 * Constructs the CAN payload based on the target device and PID configuration.
 *
 * @param device Pointer to the CANDeviceConfig structure.
 * @param pid Pointer to the CANDevicePID structure.
 * @param TxData Pointer to the buffer where the payload will be stored.
 */
void generate_can_tx_payload(CANDeviceConfig *device, CANDevicePID *pid, uint8_t *TxData);

/* --- CAN Message Handling Functions --- */

/**
 * @brief Send a CAN data packet. Typically called by a higher level function.
 *
 * THIS IS A LOWER LEVEL FUNCTION DENOTED BY _function
 *
 * Typically you want to use the function "send_can_request" if you are trying to send a CAN
 * request as that uses the internal can device and PIDs to build the packet correctly.
 *
 * This function sends a CAN message using the specified CAN data structure, which includes
 * the Tx header, data buffer, and mailbox. Handles errors by invoking the
 * error handler and logging an error message.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 * @param can_data Pointer to the CANData structure containing transmission details.
 * @return true if the message was transmitted successfully, false otherwise.
 */
bool _send_can_packet(CAN_HandleTypeDef *hcan, CANData *can_data);

/**
 * @brief Processes a received CAN packet.
 *
 * Handles the processing of a single CAN packet, including logging raw data,
 * checking for ignored messages, parsing, validation, and updating states.
 *
 * @param packet Pointer to the CANPacket to be processed.
 */
void process_can_packet(CANPacket *packet);

/**
 * @brief Retrieve a CAN message and populate a CANPacket for the specified CAN instance.
 *
 * This function attempts to retrieve a message from the CAN FIFO0 buffer and stores
 * the data into a CANPacket, which can then be queued or processed further.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 * @param packet Pointer to a CANPacket where the retrieved data will be stored.
 * @return true if the message was successfully retrieved, false otherwise.
 */
bool retrieve_can_message(CAN_HandleTypeDef *hcan, CANInstance can_instance, CANPacket *packet);

/**
 * @brief Parse and process an incoming CAN message.
 *
 * Handles the complete lifecycle of a received CAN message, including logging,
 * validation, and signal updates.
 *
 * @param can_instance The CAN instance (e.g., CAN1 or CAN2) receiving the message.
 */
void handle_incoming_can_packet(CANInstance can_instance);

/**
 * @brief Parse raw CAN data into a structured format.
 *
 * This function extracts fields such as data length, PID, and payload
 * from a raw 8-byte CAN data array and populates a ParsedCANData structure.
 * It performs basic validation on the input pointers and reports errors
 * using the user error handler if necessary.
 *
 * @param raw_data Pointer to the raw CAN data array (8 bytes).
 *                 This data represents a single CAN message payload.
 * @param parsed_data Pointer to the ParsedCANData structure where the
 *                    parsed fields will be stored.
 * @return true if parsing was successful, false if input pointers were invalid.
 */
bool parse_raw_can_data(const uint8_t *raw_data, ParsedCANData *parsed_data);

/**
 * @brief Validate the parsed CAN data fields.
 *
 * Ensures the data length and other parsed fields meet expected constraints.
 *
 * @param parsed_data Pointer to the ParsedCANData structure.
 * @return true if the data is valid, false otherwise.
 */
bool validate_parsed_can_data(const ParsedCANData *parsed_data);

/* --- Device and PID Configuration Functions --- */

/**
 * @brief Retrieve a device configuration by CAN ID.
 *
 * Searches the list of configured devices for a matching CAN ID.
 *
 * @param rx_id The CAN ID to search for.
 * @return Pointer to the matching CANDeviceConfig, or NULL if not found.
 */
CANDeviceConfig *get_device_config_by_id(uint32_t rx_id);

/**
 * @brief Retrieve a PID configuration by PID.
 *
 * Searches the list of PIDs in a device's configuration for a matching PID.
 *
 * @param device Pointer to the CANDeviceConfig containing the PIDs.
 * @param pid The PID to search for.
 * @return Pointer to the matching CANDevicePID, or NULL if not found.
 */
CANDevicePID *get_pid_by_id(CANDeviceConfig *device, uint16_t pid);

/* --- Logging Functions --- */

/**
 * @brief Log the raw incoming CAN message for debugging.
 *
 * Formats and logs the raw CAN ID and data bytes.
 *
 * @param can_instance The CAN instance (e.g., CAN_TRUCK or CAN_AUX).
 */
void log_raw_can_data(CANInstance can_instance);

/**
 * @brief Log validated CAN data for debugging.
 *
 * Provides a formatted log of the PID and payload.
 *
 * @param parsed_data Pointer to the ParsedCANData structure.
 */
void log_valid_can_data(const ParsedCANData *parsed_data);

/* --- Utility Functions --- */

/**
 * @brief Get the CAN instance corresponding to a hardware handle.
 *
 * Maps a HAL CAN handle (e.g., hcan1) to the appropriate CANInstance.
 *
 * @param hcan Pointer to the HAL CAN handle.
 * @return The CANInstance (e.g., CAN_TRUCK or CAN_AUX).
 */
CANInstance get_can_instance_from_hcan(CAN_HandleTypeDef *hcan);

/**
 * @brief Determine if a CAN message should be ignored.
 *
 * Filters out non-critical messages like heartbeats.
 *
 * @param can_id The CAN ID of the message.
 * @return true if the message should be ignored, false otherwise.
 */

/**
 * @brief Send CAN requests for all devices and their PIDs.
 *
 * Iterates through all configured devices and PIDs and sends a CAN request for each.
 * This function is typically used to initialize communication or request status
 * from all known devices on the CAN network.
 */
void send_all_requests(void);

bool should_ignore_message(uint32_t can_id);

extern CANData can_data[CAN_TOTAL];
extern CAN_HandleTypeDef *can_handles[];

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */
