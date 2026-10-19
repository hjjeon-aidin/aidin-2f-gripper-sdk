// Aidin Gripper SDK - Modbus register map (DH-Robotics RTU compatible)
// Mirrored from firmware: Aidin_gripper_functions/gripper/Inc/aidin_modbus.h
//                        Aidin_gripper_functions/modbus/include/mbtask.h
#pragma once

#include <cstdint>

namespace aidin::reg {

// ---------- Action Request (master -> slave, holding registers) ----------
inline constexpr uint16_t ACTION    = 0x0100;  // rACT|rHOM|rGTO|rATR|rADR
inline constexpr uint16_t SPEED     = 0x0101;  // rSP  0..255
inline constexpr uint16_t FORCE     = 0x0102;  // rFR  0..255
inline constexpr uint16_t POSITION  = 0x0103;  // rPR  0=open .. 255=close

// Action register bit masks
inline constexpr uint16_t RACT = 1u << 0;  // activation request
inline constexpr uint16_t RHOM = 1u << 1;  // homing (rising edge)
inline constexpr uint16_t RGTO = 1u << 3;  // go-to position
inline constexpr uint16_t RATR = 1u << 4;  // auto-release (emergency)
inline constexpr uint16_t RADR = 1u << 5;  // auto-release dir: 0=open, 1=close

// Full-initialization opcode (protocol v1.12.0, DH-Robotics compatible):
// writing exactly this VALUE to ACTION triggers stroke calibration (open
// homing + close-contact stroke measurement, flash-persisted). Includes the
// RACT bit, so a single write from the inactive state activates + calibrates.
// Edge-triggered on the written value (previous write != 0xA5 -> 0xA5).
inline constexpr uint16_t ACTION_FULL_INIT = 0x00A5;

// ---------- Gripper Status (slave -> master, holding registers) ----------
inline constexpr uint16_t STATUS      = 0x0200;  // gACT|gHOM|gGTO|gSTA|gOBJ
inline constexpr uint16_t FAULT       = 0x0201;  // gFLT
inline constexpr uint16_t POS_ECHO    = 0x0202;  // gPR (echo of rPR)
inline constexpr uint16_t POS_ACTUAL  = 0x0203;  // gPO 0=open .. 255=close
inline constexpr uint16_t CUR_ACTUAL  = 0x0204;  // gCU 0..255
inline constexpr uint16_t SPD_ACTUAL  = 0x0205;  // gSP 0..255
inline constexpr uint16_t VOLTAGE     = 0x0206;  // gV  [0.1 V units]
inline constexpr uint16_t FAULT_LATCH = 0x0207;  // gFLTO

// Status register bit masks
inline constexpr uint16_t GACT      = 1u << 0;
inline constexpr uint16_t GHOM      = 1u << 1;
inline constexpr uint16_t GGTO      = 1u << 3;
inline constexpr uint16_t GSTA_MASK = 0x3u << 4;
inline constexpr uint16_t GSTA_RESET  = 0x0u << 4;
inline constexpr uint16_t GSTA_INIT   = 0x1u << 4;
inline constexpr uint16_t GSTA_ACTIVE = 0x3u << 4;
inline constexpr uint16_t GOBJ_MASK   = 0x3u << 6;
inline constexpr uint16_t GOBJ_MOVING = 0x0u << 6;
inline constexpr uint16_t GOBJ_OPEN   = 0x1u << 6;
inline constexpr uint16_t GOBJ_CLOSE  = 0x2u << 6;
inline constexpr uint16_t GOBJ_TARGET = 0x3u << 6;

// ---------- Device configuration (factory / installer) ----------
inline constexpr uint16_t CFG_MB_ADDR    = 0x0182;  // slave address 1..247
inline constexpr uint16_t CFG_MB_BAUD_LO = 0x0183;  // baud [15:0]
inline constexpr uint16_t CFG_MB_BAUD_HI = 0x0184;  // baud [31:16]
inline constexpr uint16_t CFG_CAN_ID     = 0x0185;
inline constexpr uint16_t CFG_CAN_RATE   = 0x0186;
inline constexpr uint16_t CFG_DEV_TYPE   = 0x0187;  // 101=Aidin gripper, 102=Apicoo, 103=AFT200, 104=AFT150
inline constexpr uint16_t CFG_SERIAL_LO  = 0x0188;
inline constexpr uint16_t CFG_SERIAL_HI  = 0x0189;
inline constexpr uint16_t CFG_SENSOR_TYPE = 0x018A; // external sensor: 0=none, 1=AFT, 2=tactile
inline constexpr uint16_t CFG_STROKE     = 0x018B;  // measured stroke [rad x 1000] (read-only; STROKE_CAL/full-init updates)

// CFG_SENSOR_TYPE values. Applied at boot: write -> SAVE_CONFIG -> power cycle.
inline constexpr uint16_t SENSOR_NONE    = 0;
inline constexpr uint16_t SENSOR_AFT     = 1;
inline constexpr uint16_t SENSOR_TACTILE = 2;

// Trajectory limit registers (protocol v1.11.0). rSP scales against these:
// vel = TRAJ_MAX_VEL * rSP/255, acc = TRAJ_MAX_ACC * rSP/255.
// Writes take effect only via DEV_CMD_TRAJ_APPLY; RAM only (reboot restores
// firmware defaults 100 / 400).
inline constexpr uint16_t TRAJ_MAX_VEL = 0x019B;  // uint16 [rad/s],   valid 1..150, boot default 100
inline constexpr uint16_t TRAJ_MAX_ACC = 0x019C;  // uint16 [rad/s^2], valid 1..800, boot default 400

// Baud rate (CFG_MB_BAUD_LO/HI, protocol v1.16.0): raw bps split into LO/HI
// words — NOT a code. Supported: 9600, 19200, 38400, 57600, 115200 (default),
// 230400, 460800, 921600, 1000000 (>= 460800 needs an adapter + RS485 line that
// can carry it).
// Write LO/HI -> DEV_CMD_SAVE_CONFIG -> power cycle; the UART switches
// at the next boot. SAVE_CONFIG rejects an unsupported value (DEV_STATUS_ERROR,
// nothing saved). Firmware <= v1.15.x stored the value but always ran at 115200.

// Developer command registers (factory use only)
inline constexpr uint16_t DEV_CMD    = 0x0180;
inline constexpr uint16_t DEV_STATUS = 0x0181;

// DEV_CMD codes (docs/protocol/protocol.md §3 — full list lives there)
inline constexpr uint16_t DEV_CMD_SAVE_CONFIG  = 0x02;  // persist config regs (0x0182~) to flash
inline constexpr uint16_t DEV_CMD_FAULT_CLEAR  = 0x07;
inline constexpr uint16_t DEV_CMD_SENSOR_ZERO  = 0x08;  // software tare of sensor Fx/Fy/Fz (RAM only)
inline constexpr uint16_t DEV_CMD_TACTILE_SEND = 0x09;  // tactile CAN passthrough (TACT_CMD_*)
inline constexpr uint16_t DEV_CMD_STROKE_CAL   = 0x0A;  // stroke calibration (~20 s, needs rACT=1)
inline constexpr uint16_t DEV_CMD_TRAJ_APPLY   = 0x0E;  // apply TRAJ_MAX_VEL/ACC (RAM only)
inline constexpr uint16_t DEV_CMD_TACTILE_CMD  = 0x0F;  // tactile structured command (TACT_SCMD_*)

// DEV_STATUS codes
inline constexpr uint16_t DEV_STATUS_IDLE    = 0x00;
inline constexpr uint16_t DEV_STATUS_RUNNING = 0x01;
inline constexpr uint16_t DEV_STATUS_OK      = 0x02;
inline constexpr uint16_t DEV_STATUS_ERROR   = 0x03;

// ---------- 3-axis tactile sensor (CFG_SENSOR_TYPE = 2) ----------
// Contact resultant per fingertip (protocol v1.13.0, gripper frame v1.18.0).
// Tip 1 = RIGHT finger, tip 2 = LEFT finger. Read both tips as one 16-word block
// from TACT1_STATUS. Per tip: [status][posX][posY][fx][fy][fz][rate][rxcnt]
//   pos [mm x 100]: X = 0..3200 from the wrist-side edge of the cell area toward
//                   the fingertip, 0 = no contact point; Y = +-1250 across the pad
//   f   [N x 100] after tare, fz > 0 = pressing; rate [Hz] (~100); rxcnt wraps
inline constexpr uint16_t TACT1_STATUS   = 0x0220;  // right finger (tip 1), 0x0220..0x0227
inline constexpr uint16_t TACT2_STATUS   = 0x0228;  // left finger (tip 2),  0x0228..0x022F
inline constexpr uint16_t TACT_TIP_WORDS = 8;
inline constexpr float    TACT_POS_SCALE   = 100.0f;  // register / 100 = mm
inline constexpr float    TACT_FORCE_SCALE = 100.0f;  // register / 100 = N

// TACT_STATUS bitfield (0x0220 / 0x0228). Low byte = active sensor type (2).
inline constexpr uint16_t TACT_ST_TYPE_MASK  = 0x00FF;
inline constexpr uint16_t TACT_ST_VALID      = 0x0100;  // result frame within 50 ms and valid
inline constexpr uint16_t TACT_ST_SATURATED  = 0x0200;  // a field hit int16 limits / sensor over-range
inline constexpr uint16_t TACT_ST_TIMEOUT    = 0x0400;  // no result frame for 50 ms
inline constexpr uint16_t TACT_ST_CONNECTED  = 0x0800;  // any frame from this tip within 200 ms
inline constexpr uint16_t TACT_ST_RAW_STREAM = 0x1000;  // connected, but streaming raw (no result frame)
inline constexpr uint16_t TACT_ST_CONTACT    = 0x2000;  // valid and Fz >= 0.5 N
inline constexpr uint16_t TACT_ST_SEEN       = 0x4000;  // detected at least once since boot
inline constexpr uint16_t TACT_ST_RAW_FRESH  = 0x8000;  // raw block (TACTx_RAW) updated within 200 ms

// Raw 69-channel diagnostic stream (protocol v1.14.0) — bring-up only. Per tip,
// 71 words: [sets][diag][C0..C68] (int16, unitless). Read only when RAW_FRESH.
inline constexpr uint16_t TACT1_RAW      = 0x0230;  // right finger, 0x0230..0x0276
inline constexpr uint16_t TACT2_RAW      = 0x0277;  // left finger,  0x0277..0x02BD
inline constexpr uint16_t TACT_RAW_CH    = 69;
inline constexpr uint16_t TACT_RAW_WORDS = 2 + TACT_RAW_CH;

// CAN passthrough (write, then DEV_CMD_TACTILE_SEND). Firmware sends the bytes
// as one CAN FD frame (BRS off, DLC 8) without interpreting them.
inline constexpr uint16_t TACT_CMD_TIP   = 0x01B0;  // tip 0/1
inline constexpr uint16_t TACT_CMD_LEN   = 0x01B1;  // data length 1..8
inline constexpr uint16_t TACT_CMD_D0    = 0x01B2;  // data[0..7] -> 0x01B2..0x01B9
inline constexpr uint16_t TACT_CMD_CANID = 0x01BA;  // CAN ID 1..0x7FF, 0 = default 0x324

// Structured command (protocol v1.17.0/v1.18.0): write CODE/TIP/ARG, then
// DEV_CMD_TACTILE_CMD. DEV_STATUS OK = frame sent (the sensor does not ack).
inline constexpr uint16_t TACT_SCMD_CODE = 0x01BB;
inline constexpr uint16_t TACT_SCMD_TIP  = 0x01BC;  // tip bitmask, TACT_TIP_*
inline constexpr uint16_t TACT_SCMD_ARG  = 0x01BD;

inline constexpr uint16_t TACT_SCMD_ZERO          = 0x01;  // sensor no-load zero (do not press during it)
inline constexpr uint16_t TACT_SCMD_FACTORY_RESET = 0x05;  // ARG must be TACT_FACTORY_RESET_KEY
inline constexpr uint16_t TACT_FACTORY_RESET_KEY  = 0xA5A5;

inline constexpr uint16_t TACT_TIP_RIGHT = 0x01;  // tip 1
inline constexpr uint16_t TACT_TIP_LEFT  = 0x02;  // tip 2
inline constexpr uint16_t TACT_TIP_BOTH  = 0x03;

// ---------- Fast-poll compressed block (protocol v1.15.0, read-only) ----------
// Read-only copy of the essentials, refreshed atomically so ONE FC03 read of
// 10 words returns one consistent snapshot (~4 ms at 115200 bps):
//   0x02C0 STATUS   0x02C1 gPO   0x02C2 gCU
//   0x02C3..0x02C5 right tip: (Px8,Py8) (Fx8,Fy8) (Fz8,status8)   high byte first
//   0x02C6..0x02C8 left tip:  same layout
//   0x02C9 gFLT
// Px8/Fz8 are uint8, Py8/Fx8/Fy8 int8; 1 LSB = 0.2 mm / 0.2 N (draft scale).
// status8 = TACT_STATUS >> 8. Px8 is uint8 since v1.18.0 (int8 before).
inline constexpr uint16_t FAST_BLOCK    = 0x02C0;
inline constexpr uint16_t FAST_WORDS    = 10;
inline constexpr float    FAST_TACT_LSB = 0.2f;

} // namespace aidin::reg
