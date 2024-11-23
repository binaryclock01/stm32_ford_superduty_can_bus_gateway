/*
 * device_configs.h
 *
 * Created on: Nov 12, 2024
 * Author: Ryan
 */

#ifndef SRC_DEVICE_CONFIGS_H_
#define SRC_DEVICE_CONFIGS_H_

#include <stdint.h>   // For uint8_t, uint32_t types
#include <stdbool.h>  // For boolean support
#include "can_common.h"

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */
#define CAN_ID_REQUEST_REPLY_OFFSET 8
#define PID_BYTE_LENGTH 2
#define MAX_PAYLOAD_BYTE_LENGTH 4
#define MAX_DLC_BYTE_LENGTH 8
#define MAX_SIGNALS_PER_PID 4
#define MAX_PAYLOAD_LENGTH 4
#define DLC_MAX   8
/* -----------------------------------------------------------------------------
   CAN IDs and Enums
   -------------------------------------------------------------------------- */
#define CAN_ID_HEARTBEAT 0x59E  // Heartbeat message

typedef enum {
	CAN_ID_BCM_REQUEST = 0x726,
	CAN_ID_BCM_REPLY = 0x72E,
	CAN_ID_SCCM_REQUEST = 0x724,
	CAN_ID_SCCM_REPLY = 0x72C,
} CAN_StdIds;

typedef struct {
	uint32_t request;
	uint32_t reply;
} CAN_StdId_Type;

typedef enum {
    CAN_ID_BCM = 0,   // Body Control Module CAN ID
    CAN_ID_SCCM,  // Steering Column Control Module CAN ID
	CAN_ID_TOTAL,
} CAN_Ids;

// Function to calculate request ID based on reply CAN ID
static inline uint32_t get_request_id(CAN_Ids module_id) {
    return module_id;
}

typedef enum {
    UINT64_LENGTH_MASK  = 0xFF00000000000000,
    UINT64_COMMAND_MASK = 0x00FF000000000000,
    UINT64_PID_MASK     = 0x0000FFFF00000000,
} CANMasks;

typedef enum {
    RELEVANT_1_BYTE = 0xFF000000,  // Mask for 1 relevant byte (MSB)
    RELEVANT_2_BYTE = 0xFFFF0000, // Mask for 2 relevant bytes (MSB + next byte)
    RELEVANT_3_BYTE = 0xFFFFFF00, // Mask for 3 relevant bytes
    RELEVANT_4_BYTE = 0xFFFFFFFF  // Mask for 4 relevant bytes (all bytes relevant)
} RelevantByteMask;

/* -----------------------------------------------------------------------------
   CAN Message Bit Indexes
   -------------------------------------------------------------------------- */
typedef enum {
    DATA_LENGTH_START = 0,
    DATA_COMMAND_START = 1,
    DATA_PID_START = 2,
    DATA_PAYLOAD_START = 4
} CANMessageBitIndexes;

/* -----------------------------------------------------------------------------
   CAN State Change Types
   -------------------------------------------------------------------------- */
typedef enum {
    STATE_BIT,        // Use MASK in CANStateChange.change_data to AND with Rx'd data
    STATE_BYTE,
    STATE_MULTIBYTE
} CANStateChangeType;

/* -----------------------------------------------------------------------------
   CAN Signal and Device Structures
   -------------------------------------------------------------------------- */
typedef struct {
    const char *name;                     // Full name of the signal
    const char *short_name;               // Short name for display
    CANStateChangeType change_type;       // Type of state change
    CAN_Payload_Array state_off; // Mask for "off" state
    CAN_Payload_Array state_on; // Mask for "on" state
    uint8_t relevant_data_bytes;
    uint32_t data;                        // Current signal state or data
} CANSignal;

typedef struct {
    const char *name;                     // Name of the PID
    const char *short_name;               // Short name for display
    uint8_t pid_id[PID_BYTE_LENGTH];       // PID byte data
    uint8_t num_of_signals;					// number of signals used in PID
    CANSignal signals[MAX_SIGNALS_PER_PID]; // Signals associated with this PID
} CANDevicePID;

typedef struct {
    const char *device_name;              // Device name (e.g., "BCM", "SCCM")
    CAN_StdId_Type id;                       // CAN ID for the device
    CANDevicePID *pids;                   // Pointer to PIDs
    uint8_t pid_count;                     // Number of PIDs
} CANDeviceConfig;

/* -----------------------------------------------------------------------------
   BCM and SCCM Configurations
   -------------------------------------------------------------------------- */
typedef enum {
    BCM_PID_BRAKE_PEDAL = 0,
    BCM_PID_REVERSE_LIGHT,
    BCM_PID_HAZARD_BUTTON,
    BCM_PID_COUNT  // Reflects the number of entries
} BCMPIDIndex;

typedef enum {
    BCM_BRAKE_PEDAL = 0,
    BCM_REVERSE_LIGHT,
    BCM_HAZARD_BUTTON,
    BCM_CONFIG_COUNT
} BCMConfigIndex;

extern CANDevicePID bcm_pids[];  // BCM PID configurations

typedef enum {
    SCCM_PID_TURN_SIGNALS = 0,
    SCCM_PID_COUNT
} SCCMPIDIndex;

typedef enum {
    SCCM_LEFT_TURN = 0,
    SCCM_RIGHT_TURN,
    SCCM_LEFT_CHANGE,
    SCCM_RIGHT_CHANGE,
    SCCM_CONFIG_COUNT
} SCCMConfigIndex;

extern CANDevicePID sccm_pids[];  // SCCM PID configurations

/* -----------------------------------------------------------------------------
   Commands and Replies
   -------------------------------------------------------------------------- */
typedef struct {
    const char *name;          // Command name
    const char *short_name;    // Short name for display
    uint8_t byte;              // Command byte
} CANCommands;

typedef enum {
    CMD_DIAG,       // Index 0: Diagnostic Session Control
    CMD_RESET,      // Index 1: ECU Reset
    CMD_CLEAR,      // Index 2: Clear Diagnostic Information
    CMD_DTC,        // Index 3: Read DTC Information
    CMD_READ,       // Index 4: Read Data by Identifier
    CMD_SEC_ACC,    // Index 5: Security Access
    CMD_COMM_CTRL,  // Index 6: Communication Control
    CMD_WRITE,      // Index 7: Write Data by Identifier
    CMD_ROUTINE,    // Index 8: Routine Control
    CMD_DOWNLOAD,   // Index 9: Request Download
    CMD_UPLOAD,     // Index 10: Request Upload
    CMD_TRANSFER,   // Index 11: Transfer Data
    CMD_EXIT_TRANS, // Index 12: Request Transfer Exit
    CMD_TESTER,     // Index 13: Tester Present
    CMD_NEG_RESP    // Index 14: Negative Response
} CANCommandIndex;

typedef enum {
    RESP_DIAG_SESS,       // 0x50
    RESP_ECU_RESET,       // 0x51
    RESP_CLR_DIAG,        // 0x54
    RESP_DTC,             // 0x59
    RESP_READ,            // 0x62
    RESP_SEC_ACC,         // 0x67
    RESP_COMM_CTRL,       // 0x68
    RESP_WRITE,           // 0x6E
    RESP_ROUTINE,         // 0x71
    RESP_DOWNLOAD,        // 0x74
    RESP_XFER_DATA,       // 0x76
    RESP_XFER_EXIT,       // 0x77
    RESP_TESTER,          // 0x7E
    RESP_NEG              // 0x7F
} CANReplyCommandIndex;

// device_configs.h
#define CAN_DEVICE_COUNT 2  // Number of devices in the array

extern CANCommands can_request_commands[]; // Array of CAN request commands
extern CANCommands can_reply_commands[];   // Array of CAN reply commands

/* -----------------------------------------------------------------------------
   Device Configuration Array
   -------------------------------------------------------------------------- */
extern CANDeviceConfig can_devices[];  // Array of all CAN device configurations

static inline uint32_t get_relevant_mask(uint8_t relevant_data_bytes) {
    static const RelevantByteMask masks[] = {
        RELEVANT_1_BYTE,  // relevant_data_bytes = 1
        RELEVANT_2_BYTE, // relevant_data_bytes = 2
        RELEVANT_3_BYTE, // relevant_data_bytes = 3
        RELEVANT_4_BYTE  // relevant_data_bytes = 4
    };

    if (relevant_data_bytes >= 1 && relevant_data_bytes <= 4) {
        return masks[relevant_data_bytes - 1];
    }

    return RELEVANT_4_BYTE; // Default to all bytes relevant if input is invalid
}


#endif /* SRC_DEVICE_CONFIGS_H_ */
