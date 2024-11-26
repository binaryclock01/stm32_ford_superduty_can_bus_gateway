/*
 * device_configs.c
 *
 * Created on: Nov 12, 2024
 * Author: Ryan
 */

#include <stddef.h>
#include "device_configs.h"
#include "config.h"

// Define an array of masks
const uint64_t _g_turn_signal_bitmask_mapping_array[] = {
    TURN_SIGNAL_LEFT_BIT_MASK,
    TURN_SIGNAL_RIGHT_BIT_MASK,
    TURN_SIGNAL_LEFT_CHANGE_BIT_MASK,
    TURN_SIGNAL_RIGHT_CHANGE_BIT_MASK
};

// BCM Configurations Array
CANDevicePID bcm_pids[] = {
    {
        .name = "Hazard Button", .short_name = "Hzd",
        .pid_id = {0x71, 0x50},
		.num_of_signals = 1,
		.state_generation_type = CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE,
        .signals = {
            { .name = "Hazard Button", .short_name = "Hzd",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
			  .state_off   = {0x00, 0x00, 0x00, 0x00},
              .state_on    = {0x04, 0x00, 0x00, 0x00} }
        }
    },
    {
        .name = "Brake Pedal", .short_name = "Brk",
        .pid_id = {0x2B, 0x00},
		.num_of_signals = 1,
		.state_generation_type = CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE,
		.signals = {
            { .name = "Brake Pedal", .short_name = "Brk",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
			  .state_off   = {0x20, 0x00, 0x00, 0x00},
              .state_on    = {0x40, 0x00, 0x00, 0x00} }
        }
    },
    {
        .name = "Reverse Light", .short_name = "Rev",
        .pid_id = {0x40, 0xC8},
		.num_of_signals = 1,
		.state_generation_type = CAN_STATE_GENERATION_NON_MULTIPLEXED_BYTE,
        .signals = {
            { .name = "Reverse Light", .short_name = "Rev",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 1,
			  .state_off   = {0x02, 0xCC, 0xCC, 0xCC},
              .state_on    = {0x01, 0xCC, 0xCC, 0xCC} }
        }
    }
};

// SCCM Configurations Array
CANDevicePID sccm_pids[] = {
    {
        .name = "Turn Signals", .short_name = "TS",
        .pid_id = {0x71, 0x50},
		.num_of_signals = 4,
		.state_generation_type = CAN_STATE_GENERATION_MULTIPLEXED_BYTE,
        .signals = {
            { .name = "Left Turn Signal", .short_name = "LT",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
              .state_off  = {0x00, 0x00, 0x00, 0x00},
              .state_on   = {0x01, 0x00, 0x00, 0x00} },

            { .name = "Right Turn Signal", .short_name = "RT",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
              .state_off  = {0x00, 0x00, 0x00, 0x00},
              .state_on   = {0x02, 0x00, 0x00, 0x00} },

            { .name = "Left Lane Change", .short_name = "LC",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
              .state_off  = {0x00, 0x00, 0x00, 0x00},
              .state_on   = {0x00, 0x02, 0x00, 0x00} },

            { .name = "Right Lane Change", .short_name = "RC",
              .change_type = STATE_BIT, .data = 0,
			  .relevant_data_bytes = 4,
              .state_off  = {0x00, 0x00, 0x00, 0x00},
              .state_on   = {0x00, 0x04, 0x00, 0x00} },
        }
    }
};


// Order aligns to
//
//    CAN_ID_BCM = 0,   // Body Control Module CAN ID
//    CAN_ID_SCCM,  // Steering Column Control Module CAN ID
//
// Which is a typedef enum CAN_Ids in device_config.h.  Ensure they stay the same!!
CANDeviceConfig can_devices[CAN_DEVICE_COUNT] = {
    { .device_name = "BCM", .id.request = CAN_ID_BCM_REQUEST, .id.response = CAN_ID_BCM_REPLY, .pids = bcm_pids, .pid_count = BCM_PID_COUNT },
    { .device_name = "SCCM", .id.request = CAN_ID_SCCM_REQUEST, .id.response = CAN_ID_SCCM_REPLY, .pids = sccm_pids, .pid_count = SCCM_PID_COUNT }
};

CANCommands can_request_commands[] = {
    { .byte = REQ_DIAG_SESS_VALUE,    .name = "Diagnostic Session Control",    .short_name = "Diag"       },
    { .byte = REQ_ECU_RESET_VALUE,    .name = "ECU Reset",                     .short_name = "Reset"      },
    { .byte = REQ_CLR_DIAG_VALUE,     .name = "Clear Diagnostic Information",  .short_name = "Clear"      },
    { .byte = REQ_DTC_VALUE,          .name = "Read DTC Information",          .short_name = "DTC"        },
    { .byte = REQ_READ_VALUE,         .name = "Read Data by Identifier",       .short_name = "Read"       },
    { .byte = REQ_SEC_ACC_VALUE,      .name = "Security Access",               .short_name = "SecAcc"     },
    { .byte = REQ_COMM_CTRL_VALUE,    .name = "Communication Control",         .short_name = "CommCtrl"   },
    { .byte = REQ_WRITE_VALUE,        .name = "Write Data by Identifier",      .short_name = "Write"      },
    { .byte = REQ_ROUTINE_VALUE,      .name = "Routine Control",               .short_name = "Routine"    },
    { .byte = REQ_DOWNLOAD_VALUE,     .name = "Request Download",              .short_name = "Download"   },
    { .byte = REQ_UPLOAD_VALUE,       .name = "Request Upload",                .short_name = "Upload"     },
    { .byte = REQ_XFER_DATA_VALUE,    .name = "Transfer Data",                 .short_name = "Transfer"   },
    { .byte = REQ_XFER_EXIT_VALUE,    .name = "Request Transfer Exit",         .short_name = "ExitTrans"  },
    { .byte = REQ_TESTER_VALUE,       .name = "Tester Present",                .short_name = "Tester"     }
};


CANCommands can_response_commands[] = {
    { .byte = RESP_DIAG_SESS_VALUE,    .name = "Response: Diagnostic Session Control",    .short_name = "Diag Sess"      },
    { .byte = RESP_ECU_RESET_VALUE,    .name = "Response: ECU Reset",                     .short_name = "Resp ECU Reset" },
    { .byte = RESP_CLR_DIAG_VALUE,     .name = "Response: Clear Diagnostic Information",  .short_name = "Resp Clr Diag"  },
    { .byte = RESP_DTC_VALUE,          .name = "Response: Read DTC Information",          .short_name = "Resp DTC"       },
    { .byte = RESP_READ_VALUE,         .name = "Response: Read Data By Identifier",       .short_name = "Read Resp"      },
    { .byte = RESP_SEC_ACC_VALUE,      .name = "Response: Security Access",               .short_name = "Resp Sec Acc"   },
    { .byte = RESP_COMM_CTRL_VALUE,    .name = "Response: Communication Control",         .short_name = "Resp Comm Ctl"  },
    { .byte = RESP_WRITE_VALUE,        .name = "Response: Write Data By Identifier",      .short_name = "Write Resp"     },
    { .byte = RESP_ROUTINE_VALUE,      .name = "Response: Routine Control",               .short_name = "Resp Routine"   },
    { .byte = RESP_DOWNLOAD_VALUE,     .name = "Response: Request Download",              .short_name = "Resp Download"  },
    { .byte = RESP_XFER_DATA_VALUE,    .name = "Response: Transfer Data",                 .short_name = "Resp Xfer Data" },
    { .byte = RESP_XFER_EXIT_VALUE,    .name = "Response: Request Transfer Exit",         .short_name = "Resp Xfer Exit" },
    { .byte = RESP_TESTER_VALUE,       .name = "Response: Tester Present",                .short_name = "Resp Tester"    },
    { .byte = RESP_NEG_VALUE,          .name = "Negative Response",                       .short_name = "Resp Neg"       }
};


CANCommands *get_can_request_command(uint8_t can_cmd_byte) {
    // Determine the size of the can_request_commands array
    size_t num_commands = sizeof(can_request_commands) / sizeof(can_request_commands[0]);

    // Iterate over the array to find the matching command
    for (size_t i = 0; i < num_commands; i++) {
        if (can_request_commands[i].byte == can_cmd_byte) {
            return &can_request_commands[i]; // Return pointer to the matching struct
        }
    }

    // If no match is found, return NULL
    return NULL;
}

CANCommands *get_can_response_command(uint8_t can_cmd_byte) {
    // Determine the size of the can_response_commands array
    size_t num_commands = sizeof(can_response_commands) / sizeof(can_response_commands[0]);

    // Iterate over the array to find the matching command
    for (size_t i = 0; i < num_commands; i++) {
        if (can_response_commands[i].byte == can_cmd_byte) {
            return &can_response_commands[i]; // Return pointer to the matching struct
        }
    }

    // If no match is found, return NULL
    return NULL;
}
