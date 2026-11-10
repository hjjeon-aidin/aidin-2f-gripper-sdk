# One fingertip's contact resultant from the 3-axis tactile sensor, in the
# gripper frame (firmware protocol >= v1.27.0). Mirrors TACTx_* 0x0220~0x022F.
# Frame (pad contact face seen head-on, fingertip end up): origin = centre of
# the 32 mm cell area, +X = toward the wrist, +Y = left. Forces use the same axes.
# pos_x/pos_y are NaN when there is no contact point (test with isnan);
# forces are 0 while valid is false.

# ----- Link enum (derived from the status bits) -----
uint8 LINK_NOT_DETECTED = 0   # never seen since boot: not fitted, wiring, CAN ID, or sensor type != tactile
uint8 LINK_DISCONNECTED = 1   # was seen, frames stopped
uint8 LINK_RAW_STREAM   = 2   # connected, but the sensor sends raw channels instead of results
uint8 LINK_WAITING      = 3   # results arrive but are invalid: sensor boot zero (~4.5 s) or after a zero (~1.25 s)
uint8 LINK_LIVE         = 4   # valid values

uint8 link              # LINK_*
bool  valid
bool  contact           # valid and fz >= 0.5 N
bool  saturated         # a value hit its limit / sensor over-range

float32 pos_x           # [mm] +-16, -16 = fingertip end, + = toward the wrist; NaN = no contact point (0 = centre)
float32 pos_y           # [mm] +-12.5 across the pad width, + = left; NaN = no contact point
float32 fx              # [N] shear along +pos_x (toward the wrist), after tare
float32 fy              # [N] shear along +pos_y (left), after tare
float32 fz              # [N] normal force, > 0 = pressing, after tare

uint16 rate_hz          # result frames received in the last second (~100 when healthy)
uint16 status           # raw TACT_STATUS register (bitfield, see the driver README)
