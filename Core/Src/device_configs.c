/*
 * device_configs.c
 *
 * Created on: Nov 12, 2024
 * Author: Ryan
 */

#include "device_configs.h"

// BCM Configurations Array
CANDevicePID bcm_pids[] = {
    {
        .name = "Hazard Button", .short_name = "Hzd",
        .pid_id = {0x71, 0x50},
		.num_of_signals = 1,
        .signals = {
            { .name = "Hazard Button", .short_name = "Hzd",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x04, 0x00, 0x00, 0x00} }
        }
    },
    {
        .name = "Brake Pedal", .short_name = "Brk",
        .pid_id = {0x2B, 0x00},
		.num_of_signals = 1,
		.signals = {
            { .name = "Brake Pedal", .short_name = "Brk",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x40, 0x00, 0x00, 0x00} }
        }
    },
    {
        .name = "Reverse Light", .short_name = "Rev",
        .pid_id = {0x40, 0xC8},
		.num_of_signals = 1,
        .signals = {
            { .name = "Reverse Light", .short_name = "Rev",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x01, 0x00, 0x00, 0x00} }
        }
    }
};

// SCCM Configurations Array
CANDevicePID sccm_pids[] = {
    {
        .name = "Turn Signals", .short_name = "TS",
        .pid_id = {0x71, 0x50},
		.num_of_signals = 4,
        .signals = {
            { .name = "Left Turn Signal", .short_name = "LT",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x01, 0x00, 0x00, 0x00} },
            { .name = "Right Turn Signal", .short_name = "RT",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x02, 0x00, 0x00, 0x00} },
            { .name = "Left Lane Change", .short_name = "LC",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x00, 0x02, 0x00, 0x00} },
            { .name = "Right Lane Change", .short_name = "RC",
              .change_type = STATE_BIT, .data = 0,
              .change_data = {0x00, 0x04, 0x00, 0x00} }
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
    { .device_name = "BCM", .id.request = CAN_ID_BCM_REQUEST, .id.reply = CAN_ID_BCM_REPLY, .pids = bcm_pids, .pid_count = BCM_PID_COUNT },
    { .device_name = "SCCM", .id.request = CAN_ID_SCCM_REQUEST, .id.reply = CAN_ID_SCCM_REPLY, .pids = sccm_pids, .pid_count = SCCM_PID_COUNT }
};

// Command Definitions
CANCommands can_request_commands[] = {
    { .byte = 0x10, .name = "Diagnostic Session Control", .short_name = "Diag" },
    { .byte = 0x11, .name = "ECU Reset", .short_name = "Reset" },
    { .byte = 0x14, .name = "Clear Diagnostic Information", .short_name = "Clear" },
    { .byte = 0x19, .name = "Read DTC Information", .short_name = "DTC" },
    { .byte = 0x22, .name = "Read Data by Identifier", .short_name = "Read" },
    { .byte = 0x27, .name = "Security Access", .short_name = "SecAcc" },
    { .byte = 0x28, .name = "Communication Control", .short_name = "CommCtrl" },
    { .byte = 0x2E, .name = "Write Data by Identifier", .short_name = "Write" },
    { .byte = 0x31, .name = "Routine Control", .short_name = "Routine" },
    { .byte = 0x34, .name = "Request Download", .short_name = "Download" },
    { .byte = 0x35, .name = "Request Upload", .short_name = "Upload" },
    { .byte = 0x36, .name = "Transfer Data", .short_name = "Transfer" },
    { .byte = 0x37, .name = "Request Transfer Exit", .short_name = "ExitTrans" },
    { .byte = 0x3E, .name = "Tester Present", .short_name = "Tester" },
    { .byte = 0x7F, .name = "Negative Response", .short_name = "NegResp" }
};

CANCommands can_reply_commands[] = {
    { .byte = 0x50, .name = "Response: Diagnostic Session Control", .short_name = "Diag Sess" },
    { .byte = 0x51, .name = "Response: ECU Reset", .short_name = "Resp ECU Reset" },
    { .byte = 0x54, .name = "Response: Clear Diagnostic Information", .short_name = "Resp Clr Diag" },
    { .byte = 0x59, .name = "Response: Read DTC Information", .short_name = "Resp DTC" },
    { .byte = 0x62, .name = "Response: Read Data By Identifier", .short_name = "Read Resp" },
    { .byte = 0x67, .name = "Response: Security Access", .short_name = "Resp Sec Acc" },
    { .byte = 0x68, .name = "Response: Communication Control", .short_name = "Resp Comm Ctl" },
    { .byte = 0x6E, .name = "Response: Write Data By Identifier", .short_name = "Write Resp" },
    { .byte = 0x71, .name = "Response: Routine Control", .short_name = "Resp Routine" },
    { .byte = 0x74, .name = "Response: Request Download", .short_name = "Resp Download" },
    { .byte = 0x76, .name = "Response: Transfer Data", .short_name = "Resp Xfer Data" },
    { .byte = 0x77, .name = "Response: Request Transfer Exit", .short_name = "Resp Xfer Exit" },
    { .byte = 0x7E, .name = "Response: Tester Present", .short_name = "Resp Tester" },
    { .byte = 0x7F, .name = "Negative Response", .short_name = "Resp Neg" }
};
