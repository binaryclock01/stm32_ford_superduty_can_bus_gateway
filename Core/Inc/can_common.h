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
#include "device_configs.h"
#include "main.h"
#include "config.h"

/* -----------------------------------------------------------------------------
   Constants
   -------------------------------------------------------------------------- */

#define DLC_MAX 8 /**< Maximum data length for standard CAN frames */

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

typedef enum {
	CAN_VERB_UNKNOWN,
	CAN_VERB_REQUEST,
	CAN_VERB_REPLY,
} CAN_Verb_Type;

typedef enum {
    PACKET_RX,
    PACKET_TX
} CAN_Packet_Flow;


/* -----------------------------------------------------------------------------
   Typedefs
   -------------------------------------------------------------------------- */

typedef uint16_t CAN_PID;                    /**< CAN PID type */
typedef uint8_t CAN_Payload_Array[DLC_MAX];  /**< CAN message payload data */
typedef uint32_t CAN_Payload_uint32;         /**< CAN message payload data */
typedef uint8_t CAN_Command;

/* -----------------------------------------------------------------------------
   Structures
   -------------------------------------------------------------------------- */

/**
 * @brief Metadata structure for CAN packets.
 *
 * Contains information about the CAN instance and timestamp.
 */
typedef struct {
    CANInstance can_instance; /**< The CAN instance (e.g., CAN1, CAN2). */
    uint32_t timestamp;       /**< Timestamp in milliseconds. */
} CAN_Metadata;

/**
 * @brief Unified CAN Header Structure
 *
 * This structure consolidates the fields from CAN_RxHeaderTypeDef and
 * CAN_TxHeaderTypeDef to simplify handling of both Rx and Tx headers.
 */
typedef struct {
    uint32_t id;            /**< CAN Identifier (StdId or ExtId). */
    uint8_t dlc;            /**< Data Length Code (0-8). */
    bool is_extended_id;    /**< True if the ID is extended, false if standard. */
    bool is_remote_frame;   /**< True if the frame is a remote request, false otherwise. */
    uint8_t filter_match;   /**< Filter match index (Rx only, optional). */
    uint16_t timestamp;     /**< Timestamp (Rx only, optional). */
} CAN_Header;

/**
 * @brief Unified CAN Packet Structure
 *
 * Contains metadata, a unified header, and the payload for CAN communication.
 */
typedef struct {
    CAN_Packet_Flow flow;      /**< Direction of the packet (Rx or Tx). */
    CAN_Metadata meta;         /**< Metadata (CAN instance, timestamp). */
    CAN_Header header;         /**< Unified CAN header. */
    CAN_Payload_Array payload; /**< CAN message payload. */
} CAN_Packet;

typedef struct {
    CAN_Packet_Flow flow;      /**< Direction of the packet (Rx or Tx). */
    CAN_Metadata meta;         /**< Metadata (CAN instance, timestamp). */
    CAN_TxHeaderTypeDef header;  // Transmission header
    CAN_Payload_Array payload;          // Pointer to the CAN packet
} CAN_Tx_Packet;

/**
 * @brief Structure to hold parsed CAN data.
 */
typedef struct {
    uint8_t data_length; /**< Length of the received CAN data. */
    CAN_Command command;
    CAN_PID pid;        /**< Parsed PID from the CAN message. */
    CAN_Payload_uint32 payload;    /**< Extracted payload from the CAN message. */
} Parsed_CAN_Data;




#ifdef __cplusplus
}
#endif

#endif /* CAN_COMMON_H_ */
