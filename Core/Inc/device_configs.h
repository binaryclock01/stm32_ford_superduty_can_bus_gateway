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

/* -----------------------------------------------------------------------------
   Constants and Macros
   -------------------------------------------------------------------------- */
#define CAN_ID_REQUEST_REPLY_OFFSET 8
#define PID_BYTE_LENGTH 2
#define MAX_PAYLOAD_BYTE_LENGTH 4
#define MAX_DLC_BYTE_LENGTH 8
#define MAX_SIGNALS_PER_PID 4
#define MAX_PAYLOAD_LENGTH 4

/* -----------------------------------------------------------------------------
   CAN IDs and Enums
   -------------------------------------------------------------------------- */
#define CAN_ID_HEARTBEAT 0x59E  // Heartbeat message

typedef enum {
    CAN_ID_BCM = 0x72E,   // Body Control Module CAN ID
    CAN_ID_SCCM = 0x72C,  // Steering Column Control Module CAN ID
} CAN_IDs;

// Function to calculate request ID based on reply CAN ID
static inline uint32_t get_request_id(CAN_IDs module_id) {
    return module_id - CAN_ID_REQUEST_REPLY_OFFSET;
}

typedef enum {
    UINT8_LENGTH_MASK  = 0xFF00000000000000,
    UINT8_COMMAND_MASK = 0x00FF000000000000,
    UINT16_PID_MASK    = 0x0000FFFF00000000,
} CANMasks;

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
    uint8_t change_data[MAX_PAYLOAD_BYTE_LENGTH]; // Mask for "on" state
    uint32_t data;                        // Current signal state or data
} CANSignal;

typedef struct {
    const char *name;                     // Name of the PID
    const char *short_name;               // Short name for display
    int8_t pid_id[PID_BYTE_LENGTH];       // PID byte data
    CANSignal signals[MAX_SIGNALS_PER_PID]; // Signals associated with this PID
} CANDevicePID;

typedef struct {
    const char *device_name;              // Device name (e.g., "BCM", "SCCM")
    CAN_IDs can_id;                       // CAN ID for the device
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

#endif /* SRC_DEVICE_CONFIGS_H_ */
