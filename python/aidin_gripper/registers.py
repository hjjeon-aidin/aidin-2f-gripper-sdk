"""Modbus register map for the AIDIN BLDC gripper (DH-Robotics RTU compatible).

Mirrors the firmware-side definitions in:
  - Aidin_gripper_functions/gripper/Inc/aidin_modbus.h
  - Aidin_gripper_functions/modbus/include/mbtask.h
"""
from typing import Final

# --------------- Action request (master -> slave) ---------------
ACTION:   Final[int] = 0x0100   # rACT | rHOM | rGTO | rATR | rADR
SPEED:    Final[int] = 0x0101   # rSP  0..255
FORCE:    Final[int] = 0x0102   # rFR  0..255
POSITION: Final[int] = 0x0103   # rPR  0=open, 255=close

# Action register bit masks
RACT: Final[int] = 1 << 0
RHOM: Final[int] = 1 << 1
RGTO: Final[int] = 1 << 3
RATR: Final[int] = 1 << 4
RADR: Final[int] = 1 << 5

# Full-initialization opcode (protocol v1.12.0, DH-Robotics compatible):
# writing exactly this VALUE to ACTION triggers stroke calibration (open
# homing + close-contact stroke measurement, flash-persisted). Includes the
# RACT bit, so a single write from the inactive state activates + calibrates.
# Edge-triggered on the written value (previous write != 0xA5 -> 0xA5).
ACTION_FULL_INIT: Final[int] = 0x00A5

# --------------- Gripper status (slave -> master) ---------------
STATUS:      Final[int] = 0x0200
FAULT:       Final[int] = 0x0201
POS_ECHO:    Final[int] = 0x0202
POS_ACTUAL:  Final[int] = 0x0203
CUR_ACTUAL:  Final[int] = 0x0204
SPD_ACTUAL:  Final[int] = 0x0205
VOLTAGE:     Final[int] = 0x0206
FAULT_LATCH: Final[int] = 0x0207

# Status register bit masks
GACT:        Final[int] = 1 << 0
GHOM:        Final[int] = 1 << 1
GGTO:        Final[int] = 1 << 3
GSTA_MASK:   Final[int] = 0x3 << 4
GSTA_RESET:  Final[int] = 0x0 << 4
GSTA_INIT:   Final[int] = 0x1 << 4
GSTA_ACTIVE: Final[int] = 0x3 << 4
GOBJ_MASK:   Final[int] = 0x3 << 6
GOBJ_MOVING: Final[int] = 0x0 << 6
GOBJ_OPEN:   Final[int] = 0x1 << 6
GOBJ_CLOSE:  Final[int] = 0x2 << 6
GOBJ_TARGET: Final[int] = 0x3 << 6

# --------------- Configuration (factory / installer) ---------------
CFG_MB_ADDR:    Final[int] = 0x0182   # slave address 1..247
CFG_MB_BAUD_LO: Final[int] = 0x0183   # baud [15:0]
CFG_MB_BAUD_HI: Final[int] = 0x0184   # baud [31:16]
CFG_CAN_ID:     Final[int] = 0x0185
CFG_CAN_RATE:   Final[int] = 0x0186
CFG_DEV_TYPE:   Final[int] = 0x0187   # 101=Aidin, 102=Apicoo, 103=AFT200, 104=AFT150
CFG_SERIAL_LO:  Final[int] = 0x0188
CFG_SERIAL_HI:  Final[int] = 0x0189
CFG_SENSOR_TYPE: Final[int] = 0x018A  # external sensor: 0=none, 1=AFT, 2=tactile
CFG_STROKE:     Final[int] = 0x018B   # measured stroke [rad x 1000] (read-only)

# CFG_SENSOR_TYPE values. Applied at boot: write -> SAVE_CONFIG -> power cycle.
SENSOR_NONE:    Final[int] = 0
SENSOR_AFT:     Final[int] = 1
SENSOR_TACTILE: Final[int] = 2

# Baud rate (CFG_MB_BAUD_LO/HI, protocol v1.16.0): raw bps split into LO/HI
# words - NOT a code. Supported: 9600, 19200, 38400, 57600, 115200 (default),
# 230400, 460800, 921600, 1000000 (>= 460800 needs an adapter + RS485 line that
# can carry it).
# Write LO/HI -> DEV_CMD_SAVE_CONFIG -> power cycle; the UART switches
# at the next boot. SAVE_CONFIG rejects an unsupported value (DEV_STATUS_ERROR,
# nothing saved). Firmware <= v1.15.x stored the value but always ran at 115200.

# Trajectory limit registers (protocol v1.11.0). rSP scales against these:
#   vel = TRAJ_MAX_VEL * rSP/255, acc = TRAJ_MAX_ACC * rSP/255.
# Writes take effect only via DEV_CMD_TRAJ_APPLY; RAM only (reboot restores
# firmware defaults 100 / 400).
TRAJ_MAX_VEL: Final[int] = 0x019B  # uint16 [rad/s],   valid 1..150, boot default 100
TRAJ_MAX_ACC: Final[int] = 0x019C  # uint16 [rad/s^2], valid 1..800, boot default 400

DEV_CMD:    Final[int] = 0x0180
DEV_STATUS: Final[int] = 0x0181

# DEV_CMD codes (docs/protocol/protocol.md §3 - full list lives there)
DEV_CMD_SAVE_CONFIG:  Final[int] = 0x02  # persist config regs (0x0182~) to flash
DEV_CMD_FAULT_CLEAR:  Final[int] = 0x07
DEV_CMD_SENSOR_ZERO:  Final[int] = 0x08  # software tare of sensor Fx/Fy/Fz (RAM only)
DEV_CMD_TACTILE_SEND: Final[int] = 0x09  # tactile CAN passthrough (TACT_CMD_*)
DEV_CMD_STROKE_CAL:   Final[int] = 0x0A  # stroke calibration (~20 s, needs rACT=1)
DEV_CMD_TRAJ_APPLY:   Final[int] = 0x0E  # apply TRAJ_MAX_VEL/ACC (RAM only)
DEV_CMD_TACTILE_CMD:  Final[int] = 0x0F  # tactile structured command (TACT_SCMD_*)

# DEV_STATUS codes
DEV_STATUS_IDLE:    Final[int] = 0x00
DEV_STATUS_RUNNING: Final[int] = 0x01
DEV_STATUS_OK:      Final[int] = 0x02
DEV_STATUS_ERROR:   Final[int] = 0x03

# --------------- 3-axis tactile sensor (CFG_SENSOR_TYPE = 2) ---------------
# Contact resultant per fingertip (protocol v1.13.0, gripper frame v1.18.0).
# Tip 1 = RIGHT finger, tip 2 = LEFT finger. Read both tips as one 16-word block
# from TACT1_STATUS. Per tip: [status][posX][posY][fx][fy][fz][rate][rxcnt]
#   pos [mm x 100]: X = 0..3200 from the wrist-side edge of the cell area toward
#                   the fingertip, 0 = no contact point; Y = +-1250 across the pad
#   f   [N x 100] after tare, fz > 0 = pressing; rate [Hz] (~100); rxcnt wraps
TACT1_STATUS:   Final[int] = 0x0220   # right finger (tip 1), 0x0220..0x0227
TACT2_STATUS:   Final[int] = 0x0228   # left finger (tip 2),  0x0228..0x022F
TACT_TIP_WORDS: Final[int] = 8
TACT_POS_SCALE:   Final[float] = 100.0  # register / 100 = mm
TACT_FORCE_SCALE: Final[float] = 100.0  # register / 100 = N

# TACT_STATUS bitfield (0x0220 / 0x0228). Low byte = active sensor type (2).
TACT_ST_TYPE_MASK:  Final[int] = 0x00FF
TACT_ST_VALID:      Final[int] = 0x0100  # result frame within 50 ms and valid
TACT_ST_SATURATED:  Final[int] = 0x0200  # a field hit int16 limits / sensor over-range
TACT_ST_TIMEOUT:    Final[int] = 0x0400  # no result frame for 50 ms
TACT_ST_CONNECTED:  Final[int] = 0x0800  # any frame from this tip within 200 ms
TACT_ST_RAW_STREAM: Final[int] = 0x1000  # connected, but streaming raw (no result frame)
TACT_ST_CONTACT:    Final[int] = 0x2000  # valid and Fz >= 0.5 N
TACT_ST_SEEN:       Final[int] = 0x4000  # detected at least once since boot
TACT_ST_RAW_FRESH:  Final[int] = 0x8000  # raw block (TACTx_RAW) updated within 200 ms

# Raw 69-channel diagnostic stream (protocol v1.14.0) - bring-up only. Per tip,
# 71 words: [sets][diag][C0..C68] (int16, unitless). Read only when RAW_FRESH.
TACT1_RAW:      Final[int] = 0x0230   # right finger, 0x0230..0x0276
TACT2_RAW:      Final[int] = 0x0277   # left finger,  0x0277..0x02BD
TACT_RAW_CH:    Final[int] = 69
TACT_RAW_WORDS: Final[int] = 2 + TACT_RAW_CH

# CAN passthrough (write, then DEV_CMD_TACTILE_SEND). Firmware sends the bytes
# as one CAN FD frame (BRS off, DLC 8) without interpreting them.
TACT_CMD_TIP:   Final[int] = 0x01B0   # tip 0/1
TACT_CMD_LEN:   Final[int] = 0x01B1   # data length 1..8
TACT_CMD_D0:    Final[int] = 0x01B2   # data[0..7] -> 0x01B2..0x01B9
TACT_CMD_CANID: Final[int] = 0x01BA   # CAN ID 1..0x7FF, 0 = default 0x324

# Structured command (protocol v1.17.0/v1.18.0): write CODE/TIP/ARG, then
# DEV_CMD_TACTILE_CMD. DEV_STATUS OK = frame sent (the sensor does not ack).
TACT_SCMD_CODE: Final[int] = 0x01BB
TACT_SCMD_TIP:  Final[int] = 0x01BC   # tip bitmask, TACT_TIP_*
TACT_SCMD_ARG:  Final[int] = 0x01BD

TACT_SCMD_ZERO:          Final[int] = 0x01  # sensor no-load zero (do not press during it)
TACT_SCMD_FACTORY_RESET: Final[int] = 0x05  # ARG must be TACT_FACTORY_RESET_KEY
TACT_FACTORY_RESET_KEY:  Final[int] = 0xA5A5

TACT_TIP_RIGHT: Final[int] = 0x01  # tip 1
TACT_TIP_LEFT:  Final[int] = 0x02  # tip 2
TACT_TIP_BOTH:  Final[int] = 0x03

# --------------- Fast-poll compressed block (protocol v1.15.0, read-only) ---------------
# Read-only copy of the essentials, refreshed atomically so ONE FC03 read of
# 10 words returns one consistent snapshot (~4 ms at 115200 bps):
#   0x02C0 STATUS   0x02C1 gPO   0x02C2 gCU
#   0x02C3..0x02C5 right tip: (Px8,Py8) (Fx8,Fy8) (Fz8,status8)   high byte first
#   0x02C6..0x02C8 left tip:  same layout
#   0x02C9 gFLT
# Px8/Fz8 are uint8, Py8/Fx8/Fy8 int8. Each field's full scale maps to the end
# of its byte range (protocol v1.19.0): uint8 255, int8 +-127 (-128 unused):
#   Px = byte * 32 / 255 mm        Fz     = byte * 70 / 255 N
#   Py = int8 * 12.5 / 127 mm      Fx, Fy = int8 * 35 / 127 N
# status8 = TACT_STATUS >> 8. Px8 is uint8 since v1.18.0 (int8 before).
FAST_BLOCK: Final[int] = 0x02C0
FAST_WORDS: Final[int] = 10
FAST_PX_MAX:  Final[float] = 32.0   # mm,  uint8 0..255
FAST_PY_MAX:  Final[float] = 12.5   # mm,  int8 +-127
FAST_FXY_MAX: Final[float] = 35.0   # N,   int8 +-127 (provisional: half of Fz)
FAST_FZ_MAX:  Final[float] = 70.0   # N,   uint8 0..255
