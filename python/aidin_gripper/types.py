"""Public types for the AIDIN gripper Python SDK."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class GripperStage(IntEnum):
    RESET      = 0
    ACTIVATING = 1
    ACTIVE     = 3


class ObjectState(IntEnum):
    MOVING         = 0
    OPEN_CONTACT   = 1   # object detected while opening
    CLOSED_CONTACT = 2   # object detected while closing
    AT_TARGET      = 3


@dataclass
class GripperState:
    activated:     bool         = False
    homed:         bool         = False
    go_to_active:  bool         = False
    stage:         GripperStage = GripperStage.RESET
    object_state:  ObjectState  = ObjectState.MOVING
    fault:         int          = 0      # gFLT
    position_echo: int          = 0      # gPR
    position:      int          = 0      # gPO 0..255 (0=open, 255=close)
    current:       int          = 0      # gCU 0..255
    speed:         int          = 0      # gSP 0..255
    voltage:       float        = 0.0    # V (gV * 0.1)
    latched_fault: int          = 0      # gFLTO

    def __str__(self) -> str:
        return (
            f"{{stage={self.stage.name} obj={self.object_state.name} "
            f"pos={self.position} cur={self.current} spd={self.speed} "
            f"V={self.voltage:.1f} flt={self.fault}}}"
        )


class TactileLink(IntEnum):
    """Per-tip connection state, derived from the TACT_STATUS bits."""
    NOT_DETECTED = 0   # never seen since boot: not fitted, wiring, CAN ID mismatch, or sensor type != tactile
    DISCONNECTED = 1   # was seen, frames stopped
    RAW_STREAM   = 2   # connected, but the sensor firmware sends raw channels, not results
    WAITING      = 3   # results arrive but are invalid: sensor boot zero (~4.5 s) or after a zero (~1.25 s)
    LIVE         = 4   # valid values


@dataclass
class TactileTip:
    """Contact resultant of one fingertip, in the gripper frame (firmware >= v1.18.0).

    pos_x: 0..32 mm from the wrist-side edge of the cell area toward the
    fingertip; 0 = no contact point (also when Fz is too small to locate one).
    pos_y: +-12.5 mm across the pad width. Forces are after tare, fz > 0 = pressing.
    Values are 0 while ``valid`` is False.
    """
    valid:       bool  = False
    saturated:   bool  = False
    timeout:     bool  = False
    connected:   bool  = False
    raw_stream:  bool  = False
    contact:     bool  = False   # valid and Fz >= 0.5 N
    seen:        bool  = False
    raw_fresh:   bool  = False
    pos_x:       float = 0.0     # mm
    pos_y:       float = 0.0     # mm
    fx:          float = 0.0     # N
    fy:          float = 0.0     # N
    fz:          float = 0.0     # N
    rate_hz:     int   = 0       # result frames in the last second; read_tactile() only
    rx_count:    int   = 0       # result frames since boot (wraps); read_tactile() only
    status:      int   = 0       # raw TACT_STATUS word (read_tactile) or status8 << 8 (read_fast_state)

    @property
    def link(self) -> TactileLink:
        if self.valid:
            return TactileLink.LIVE
        if self.connected:
            return TactileLink.RAW_STREAM if self.raw_stream else TactileLink.WAITING
        return TactileLink.DISCONNECTED if self.seen else TactileLink.NOT_DETECTED

    @property
    def has_contact_point(self) -> bool:
        return self.valid and self.pos_x > 0.0

    def __str__(self) -> str:
        return (
            f"{{{self.link.name} pos=({self.pos_x:.2f},{self.pos_y:.2f})mm "
            f"F=({self.fx:.2f},{self.fy:.2f},{self.fz:.2f})N"
            f"{' contact' if self.contact else ''}{' SAT' if self.saturated else ''}}}"
        )


@dataclass
class TactileRaw:
    """Raw 69-channel diagnostic stream of one tip (bring-up only, unitless)."""
    sets:     int = 0                                  # complete sets received (wraps)
    diag:     int = 0                                  # sensor AFE diagnostic word
    channels: list[int] = field(default_factory=list)  # C0..C68, int16 deltas, ~0 unloaded
    groups:   list[int] = field(default_factory=list)  # G1..G23 = sum of 3 channels each


@dataclass
class FastState:
    """One snapshot of the fast-poll block 0x02C0..0x02C9 (single FC03 read).

    Gripper fields are copies of 0x0200/0x0201/0x0203/0x0204. Tactile values
    are 8-bit compressed (0.2 mm / 0.2 N per LSB) - use read_tactile() when
    you need 0.01 resolution, rate_hz or rx_count.
    """
    activated:    bool         = False
    homed:        bool         = False
    go_to_active: bool         = False
    stage:        GripperStage = GripperStage.RESET
    object_state: ObjectState  = ObjectState.MOVING
    position:     int          = 0      # gPO
    current:      int          = 0      # gCU
    fault:        int          = 0      # gFLT
    right:        TactileTip   = field(default_factory=TactileTip)   # tip 1
    left:         TactileTip   = field(default_factory=TactileTip)   # tip 2

    def __str__(self) -> str:
        return (
            f"{{stage={self.stage.name} obj={self.object_state.name} "
            f"pos={self.position} cur={self.current} flt={self.fault} "
            f"R={self.right} L={self.left}}}"
        )


class GripperError(RuntimeError):
    """Raised on Modbus failure or command timeout."""
    pass
