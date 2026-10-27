"""AIDIN BLDC Gripper - Python SDK.

>>> from aidin_gripper import Gripper
>>> with Gripper() as g:
...     g.connect("/dev/ttyUSB0")
...     g.activate()
...     g.home()
...     g.move_to(255, speed=200, blocking=True)
"""
from .gripper import Gripper
from .types import (
    FastState,
    FirmwareInfo,
    GripperError,
    GripperStage,
    GripperState,
    ObjectState,
    TactileLink,
    TactileRaw,
    TactileTip,
)
from . import registers

__all__ = [
    "Gripper",
    "GripperError",
    "GripperState",
    "GripperStage",
    "ObjectState",
    "FastState",
    "FirmwareInfo",
    "TactileLink",
    "TactileRaw",
    "TactileTip",
    "registers",
]

__version__ = "1.2.0"
