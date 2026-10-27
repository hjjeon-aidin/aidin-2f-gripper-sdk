# AIDIN BLDC Gripper - snapshot of slave-side status (DH-Robotics map)
# Published by aidin_gripper_driver at the configured rate (default 50 Hz).
# All numeric fields mirror the firmware's holding registers 0x0200..0x0207.

std_msgs/Header header

# ----- Stage enum (gSTA) -----
uint8 STAGE_RESET      = 0
uint8 STAGE_ACTIVATING = 1
uint8 STAGE_ACTIVE     = 3

# ----- Object enum (gOBJ) -----
uint8 OBJECT_MOVING         = 0
uint8 OBJECT_OPEN_CONTACT   = 1   # reserved — current firmware never reports contact via gOBJ
uint8 OBJECT_CLOSED_CONTACT = 2   # reserved — current firmware never reports contact via gOBJ
uint8 OBJECT_AT_TARGET      = 3   # move finished (also when an object stopped the fingers); grasp = position < target

# ----- Status bits -----
bool activated          # gACT
bool homed              # gHOM
bool go_to_active       # gGTO

uint8 stage             # gSTA — use STAGE_* constants
uint8 object_state      # gOBJ — use OBJECT_* constants

# ----- Faults -----
uint8 fault             # gFLT  (live fault, 0 = OK, see the fault code table)
uint8 latched_fault     # gFLTO (fault history: kept until ~/fault_clear, re-activating does not clear it)

# ----- Live measurements -----
uint8   position_echo   # gPR  (echo of last commanded rPR)
uint8   position        # gPO  (0 = fully open, 255 = fully closed)
uint8   current         # gCU  (0..255 raw)
uint8   speed           # gSP  (0..255 raw)
float32 voltage         # gV * 0.1  [V]
