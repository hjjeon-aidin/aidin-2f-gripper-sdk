"""High-level Python wrapper for the AIDIN BLDC gripper (Modbus RTU).

Mirrors the C++ SDK API:
    g = Gripper()
    g.connect("/dev/ttyUSB0")
    g.activate()
    g.home()
    g.move_to(255, speed=200, force=128, blocking=True)
"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from typing import Optional

from pymodbus.client import ModbusSerialClient

from . import registers as reg
from .types import (
    FastState,
    GripperError,
    GripperStage,
    GripperState,
    ObjectState,
    TactileRaw,
    TactileTip,
)


_POLL_INTERVAL_S = 0.02
_DEFAULT_TIMEOUT_S = 0.5


class _SharedPort:
    """One physical serial connection, shared by every Gripper on it.

    RS485 is a half-duplex multi-drop bus: several grippers with distinct
    slave IDs can sit on the same wire, but only one Modbus transaction
    may be in flight at a time. Gripper.connect() looks up (or creates)
    one of these per port path so several Gripper instances - each with
    its own slave_id - share one pymodbus client instead of each trying
    to open the OS serial handle for themselves (which fails outright on
    Windows, and silently corrupts the bus on Linux).
    """

    def __init__(self, client: ModbusSerialClient, baudrate: int, parity: str) -> None:
        self.client = client
        self.baudrate = baudrate
        self.parity = parity
        self.lock = threading.Lock()
        self.ref_count = 0


_port_registry: dict[str, _SharedPort] = {}
_registry_lock = threading.Lock()


class Gripper:
    """Controls one AIDIN BLDC gripper over Modbus RTU.

    Several Gripper instances may connect() to the same port with
    different slave_id values to drive multiple grippers wired on one
    RS485 multi-drop bus - the underlying serial connection is opened
    once and shared (see _SharedPort). Each device must already have a
    distinct slave address programmed (registers.CFG_MB_ADDR) before
    being wired onto a shared bus.
    """

    def __init__(self) -> None:
        self._client: Optional[ModbusSerialClient] = None
        self._shared: Optional[_SharedPort] = None
        self._port: Optional[str] = None
        self._slave_id: int = 1

    @property
    def slave_id(self) -> int:
        return self._slave_id

    # ---------------------------- Lifecycle ----------------------------
    def connect(
        self,
        port: str,
        baudrate: int = 115200,
        parity: str = "N",
        slave_id: int = 1,
        response_timeout_s: float = _DEFAULT_TIMEOUT_S,
    ) -> None:
        """Open serial port and verify the slave responds.

        port: "/dev/ttyUSB0" on Linux, "COM3" on Windows.

        If another Gripper already has this port open (e.g. a second
        device at a different slave_id on the same RS485 bus), the
        existing connection is reused instead of opening the port again -
        baudrate/parity must match, since those belong to the shared
        physical link, not to one slave_id. response_timeout_s only takes
        effect on the connect() call that actually opens the port.
        """
        self.disconnect()

        with _registry_lock:
            shared = _port_registry.get(port)
            if shared is None:
                client = ModbusSerialClient(
                    port=port,
                    baudrate=baudrate,
                    parity=parity,
                    stopbits=1,
                    bytesize=8,
                    timeout=response_timeout_s,
                )
                if not client.connect():
                    raise GripperError(f"Cannot open serial port {port!r}")
                shared = _SharedPort(client, baudrate, parity)
                _port_registry[port] = shared
            elif shared.baudrate != baudrate or shared.parity != parity:
                raise GripperError(
                    f"{port!r} is already open at {shared.baudrate} baud "
                    f"parity={shared.parity!r}; every device on one RS485 "
                    f"bus must share the same baudrate/parity"
                )
            shared.ref_count += 1

        self._shared = shared
        self._client = shared.client
        self._port = port
        self._slave_id = slave_id

        # Sanity check: a single read to confirm the slave is alive.
        try:
            self._read_block(reg.STATUS, 1)
        except GripperError:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if self._shared is None:
            return
        port, shared = self._port, self._shared
        with _registry_lock:
            shared.ref_count -= 1
            if shared.ref_count <= 0:
                try:
                    shared.client.close()
                except Exception:
                    pass
                if port is not None:
                    _port_registry.pop(port, None)
        self._shared = None
        self._client = None
        self._port = None

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.connected

    def __enter__(self) -> "Gripper":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ---------------------------- Commands ----------------------------
    def activate(self, timeout_s: float = 3.0) -> None:
        """Set rACT=1 and wait until gSTA reports ACTIVE."""
        self._write1(reg.ACTION, reg.RACT)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read_state().stage == GripperStage.ACTIVE:
                return
            time.sleep(_POLL_INTERVAL_S)
        raise GripperError("activate timeout (gSTA never reached ACTIVE)")

    def deactivate(self) -> None:
        self._write1(reg.ACTION, 0)

    def home(self, timeout_s: float = 15.0) -> None:
        """Trigger homing and block until gHOM=1.

        The firmware homes on the rising edge of rHOM. We clear rHOM first,
        wait briefly, then assert it.
        """
        self._write1(reg.ACTION, reg.RACT)              # ensure rHOM low
        time.sleep(0.02)
        self._write1(reg.ACTION, reg.RACT | reg.RHOM)    # rising edge
        time.sleep(0.10)                                  # let firmware clear gHOM

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            if s.fault:
                raise GripperError(f"homing fault code={s.fault}")
            if s.homed:
                self._write1(reg.ACTION, reg.RACT)        # release rHOM
                return
            time.sleep(0.05)
        raise GripperError("home timeout")

    def full_init(self, timeout_s: float = 30.0) -> None:
        """Full initialization (protocol v1.12.0, DH-compatible ACTION=0xA5).

        Activate + stroke calibration: open homing, close-contact stroke
        measurement, flash-persisted (readable at ``reg.CFG_STROKE``). Works
        from the inactive state. Blocks until DEV_STATUS=OK and gHOM=1.
        Fingers sweep the FULL stroke twice — keep the workspace clear.
        """
        # The trigger is edge-detected on the written VALUE, so write plain
        # rACT first to guarantee a fresh edge regardless of prior state.
        self._write1(reg.ACTION, reg.RACT)
        time.sleep(0.02)
        self._write1(reg.ACTION, reg.ACTION_FULL_INIT)

        # DEV_STATUS may still hold a previous command's OK — only trust a
        # terminal value after RUNNING has been observed. A rejection (already
        # homing/calibrating) reports ERROR without ever showing RUNNING.
        saw_running = False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            if s.fault:
                raise GripperError(f"full-init fault code={s.fault}")
            dev = self.read_register(reg.DEV_STATUS)
            if dev == reg.DEV_STATUS_RUNNING:
                saw_running = True
            if dev == reg.DEV_STATUS_ERROR:
                self._write1(reg.ACTION, reg.RACT)
                raise GripperError("full-init rejected or failed (DEV_STATUS=ERROR)")
            if saw_running and dev == reg.DEV_STATUS_OK and s.homed:
                self._write1(reg.ACTION, reg.RACT)    # fresh value edge for next call
                return
            time.sleep(0.05)
        self._write1(reg.ACTION, reg.RACT)
        raise GripperError("full-init timeout")

    def set_traj_limits(self, max_vel: int, max_acc: int) -> None:
        """Set trajectory generator limits (protocol v1.11.0, RAM only).

        ``max_vel`` [rad/s] 1..150, ``max_acc`` [rad/s^2] 1..800. Applied via
        DEV_CMD TRAJ_APPLY; reboot restores firmware defaults (100/400).
        Raises if the firmware rejects (out of range, or homing/calibrating).
        """
        self._write1(reg.TRAJ_MAX_VEL, max_vel)
        self._write1(reg.TRAJ_MAX_ACC, max_acc)
        self._run_dev_cmd(reg.DEV_CMD_TRAJ_APPLY, "traj-apply",
                          "vel 1..150, acc 1..800, not during homing/cal")

    def move_to(
        self,
        position: int,
        speed: int = 128,
        force: int = 128,
        blocking: bool = False,
        timeout_s: float = 5.0,
    ) -> None:
        """Move to absolute position. 0 = fully open, 255 = fully closed.

        speed, force: 0..255.  blocking=True polls until gOBJ != MOVING.
        """
        _u8(position, "position")
        _u8(speed,    "speed")
        _u8(force,    "force")

        self._write1(reg.SPEED,    speed)
        self._write1(reg.FORCE,    force)
        self._write1(reg.POSITION, position)
        self._write1(reg.ACTION,   reg.RACT | reg.RGTO)

        if not blocking:
            return

        time.sleep(0.03)  # let the firmware leave the previous AT_TARGET state
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            if s.fault:
                raise GripperError(f"move fault code={s.fault}")
            if s.object_state != ObjectState.MOVING:
                return
            time.sleep(_POLL_INTERVAL_S)
        raise GripperError("move_to timeout")

    def emergency_release(self, close_direction: bool = False) -> None:
        bits = reg.RATR | (reg.RADR if close_direction else 0)
        self._write1(reg.ACTION, bits)

    # ---------------------------- Status ----------------------------
    def read_state(self) -> GripperState:
        """Read the full status block (0x0200..0x0207) in one transaction."""
        regs = self._read_block(reg.STATUS, 8)
        st = regs[0]
        return GripperState(
            activated     = bool(st & reg.GACT),
            homed         = bool(st & reg.GHOM),
            go_to_active  = bool(st & reg.GGTO),
            stage         = _stage(st),
            object_state  = _object_state(st),
            fault         = regs[1] & 0xFF,
            position_echo = regs[2] & 0xFF,
            position      = regs[3] & 0xFF,
            current       = regs[4] & 0xFF,
            speed         = regs[5] & 0xFF,
            voltage       = regs[6] * 0.1,
            latched_fault = regs[7] & 0xFF,
        )

    def is_at_target(self) -> bool:
        return self.read_state().object_state == ObjectState.AT_TARGET

    def has_fault(self) -> bool:
        return self.read_state().fault != 0

    def read_fast_state(self) -> FastState:
        """Read the fast-poll block (0x02C0..0x02C9) in ONE transaction.

        Gripper status + position + current + fault and both fingertips'
        tactile resultant, refreshed atomically by the firmware, so the reply
        is one consistent snapshot (~4 ms at 115200 bps). Tactile values are
        8-bit compressed (0.2 mm / 0.2 N). Firmware protocol >= v1.15.0;
        pos_x decoding assumes >= v1.18.0 (uint8).
        """
        regs = self._read_block(reg.FAST_BLOCK, reg.FAST_WORDS)
        st = regs[0]
        return FastState(
            activated    = bool(st & reg.GACT),
            homed        = bool(st & reg.GHOM),
            go_to_active = bool(st & reg.GGTO),
            stage        = _stage(st),
            object_state = _object_state(st),
            position     = regs[1] & 0xFF,
            current      = regs[2] & 0xFF,
            fault        = regs[9] & 0xFF,
            right        = _fast_tip(regs[3:6]),
            left         = _fast_tip(regs[6:9]),
        )

    # ---------------------------- Tactile sensor ----------------------------
    # Needs CFG_SENSOR_TYPE = SENSOR_TACTILE (2) - set it, SAVE_CONFIG, power cycle.
    def read_tactile(self) -> tuple[TactileTip, TactileTip]:
        """Read both fingertips' contact resultant (0x0220..0x022F, one transaction).

        Returns ``(right, left)`` = (tip 1, tip 2): contact point [mm] and
        Fx/Fy/Fz [N], 0.01 resolution, plus the per-tip link state. With
        another sensor type every tip reads NOT_DETECTED with zero values.
        """
        regs = self._read_block(reg.TACT1_STATUS, 2 * reg.TACT_TIP_WORDS)
        tips = []
        for o in (0, reg.TACT_TIP_WORDS):
            tips.append(_tactile_tip(
                regs[o],
                _s16(regs[o + 1]) / reg.TACT_POS_SCALE,
                _s16(regs[o + 2]) / reg.TACT_POS_SCALE,
                _s16(regs[o + 3]) / reg.TACT_FORCE_SCALE,
                _s16(regs[o + 4]) / reg.TACT_FORCE_SCALE,
                _s16(regs[o + 5]) / reg.TACT_FORCE_SCALE,
                rate_hz=regs[o + 6],
                rx_count=regs[o + 7],
            ))
        return tips[0], tips[1]

    def read_tactile_raw(self, tip: int) -> TactileRaw:
        """Read one tip's raw 69-channel diagnostic stream (bring-up only).

        ``tip`` = ``reg.TACT_TIP_RIGHT`` or ``reg.TACT_TIP_LEFT``. The block
        keeps its last values when the stream stops, so trust it only while
        that tip's ``raw_fresh`` is True.
        """
        if tip not in (reg.TACT_TIP_RIGHT, reg.TACT_TIP_LEFT):
            raise ValueError("tip must be TACT_TIP_RIGHT or TACT_TIP_LEFT")
        addr = reg.TACT1_RAW if tip == reg.TACT_TIP_RIGHT else reg.TACT2_RAW
        regs = self._read_block(addr, reg.TACT_RAW_WORDS)
        ch = [_s16(v) for v in regs[2:]]
        groups = [ch[i] + ch[i + 1] + ch[i + 2] for i in range(0, reg.TACT_RAW_CH, 3)]
        return TactileRaw(sets=regs[0], diag=regs[1], channels=ch, groups=groups)

    def tactile_zero(self, tips: int = reg.TACT_TIP_BOTH) -> None:
        """Sensor no-load zero (structured command ZERO, firmware >= v1.17.0).

        Keep the pads untouched. After OK the tip reads WAITING for ~1.25 s,
        then LIVE again; a tip whose zero CAN ID is not known falls back to the
        firmware software tare. Raises if rejected: sensor type is not tactile,
        values invalid, or force control with sensor feedback is running.
        """
        self.tactile_command(reg.TACT_SCMD_ZERO, tips)

    def tactile_command(self, code: int, tips: int, arg: int = 0) -> None:
        """Send a structured tactile command (``reg.TACT_SCMD_*``) and wait for OK.

        ``tips`` is a bitmask of ``reg.TACT_TIP_*``. OK means the CAN frame was
        sent; the sensor does not acknowledge.
        """
        if not (1 <= tips <= reg.TACT_TIP_BOTH):
            raise ValueError("tips must be a TACT_TIP_* bitmask (1..3)")
        self._write_block(reg.TACT_SCMD_CODE, [code, tips, arg])
        self._run_dev_cmd(reg.DEV_CMD_TACTILE_CMD, "tactile command")

    def sensor_zero(self) -> None:
        """Firmware software tare (DEV_CMD 0x08) of the fingertip sensors.

        Captures the current Fx/Fy/Fz of every valid tip as the new zero
        (AFT F/T or tactile). RAM only - cleared on reboot.
        """
        self._run_dev_cmd(reg.DEV_CMD_SENSOR_ZERO, "sensor-zero")

    # ---------------------------- Low-level ----------------------------
    def read_register(self, pdu_addr: int) -> int:
        """Read a single holding register at its protocol-level PDU address."""
        return self._read_block(pdu_addr, 1)[0]

    def write_register(self, pdu_addr: int, value: int) -> None:
        self._write1(pdu_addr, value)

    # ---------------------------- internals ----------------------------
    def _require_shared(self) -> _SharedPort:
        if self._shared is None:
            raise GripperError("not connected")
        return self._shared

    def _write1(self, addr: int, value: int) -> None:
        shared = self._require_shared()
        with shared.lock:
            rr = shared.client.write_register(addr, value & 0xFFFF, slave=self._slave_id)
        if rr.isError():
            raise GripperError(
                f"write_register(0x{addr:04X}, 0x{value:04X}) failed: {rr}")

    def _write_block(self, addr: int, values: list[int]) -> None:
        shared = self._require_shared()
        with shared.lock:
            rr = shared.client.write_registers(
                addr, [v & 0xFFFF for v in values], slave=self._slave_id)
        if rr.isError():
            raise GripperError(
                f"write_registers(0x{addr:04X}, {len(values)}) failed: {rr}")

    def _read_block(self, addr: int, count: int) -> list[int]:
        shared = self._require_shared()
        with shared.lock:
            rr = shared.client.read_holding_registers(addr, count=count, slave=self._slave_id)
        if rr.isError():
            raise GripperError(
                f"read_holding_registers(0x{addr:04X}, {count}) failed: {rr}")
        return list(rr.registers)

    def _run_dev_cmd(self, code: int, name: str, reject_hint: str = "DEV_STATUS=ERROR") -> None:
        """Issue an instant DEV_CMD and wait for OK; raise on ERROR or timeout."""
        self._write1(reg.DEV_CMD, code)
        # The firmware runs DEV_CMD in its main loop and only overwrites
        # DEV_STATUS with the result (never resets it on write) - give it a
        # moment so we don't read back a previous command's status.
        time.sleep(0.02)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            dev = self.read_register(reg.DEV_STATUS)
            if dev == reg.DEV_STATUS_OK:
                return
            if dev == reg.DEV_STATUS_ERROR:
                raise GripperError(f"{name} rejected ({reject_hint})")
            time.sleep(0.02)
        raise GripperError(f"{name} timeout")


def _u8(v: int, name: str) -> None:
    if not (0 <= v <= 255):
        raise ValueError(f"{name} must be in 0..255, got {v}")


def _s16(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def _s8(v: int) -> int:
    return v - 0x100 if v & 0x80 else v


def _stage(st: int) -> GripperStage:
    return {
        reg.GSTA_RESET:  GripperStage.RESET,
        reg.GSTA_INIT:   GripperStage.ACTIVATING,
        reg.GSTA_ACTIVE: GripperStage.ACTIVE,
    }.get(st & reg.GSTA_MASK, GripperStage.RESET)


def _object_state(st: int) -> ObjectState:
    return {
        reg.GOBJ_MOVING: ObjectState.MOVING,
        reg.GOBJ_OPEN:   ObjectState.OPEN_CONTACT,
        reg.GOBJ_CLOSE:  ObjectState.CLOSED_CONTACT,
        reg.GOBJ_TARGET: ObjectState.AT_TARGET,
    }.get(st & reg.GOBJ_MASK, ObjectState.MOVING)


def _tactile_tip(status: int, pos_x: float, pos_y: float, fx: float, fy: float,
                 fz: float, rate_hz: int = 0, rx_count: int = 0) -> TactileTip:
    return TactileTip(
        valid      = bool(status & reg.TACT_ST_VALID),
        saturated  = bool(status & reg.TACT_ST_SATURATED),
        timeout    = bool(status & reg.TACT_ST_TIMEOUT),
        connected  = bool(status & reg.TACT_ST_CONNECTED),
        raw_stream = bool(status & reg.TACT_ST_RAW_STREAM),
        contact    = bool(status & reg.TACT_ST_CONTACT),
        seen       = bool(status & reg.TACT_ST_SEEN),
        raw_fresh  = bool(status & reg.TACT_ST_RAW_FRESH),
        pos_x=pos_x, pos_y=pos_y, fx=fx, fy=fy, fz=fz,
        rate_hz=rate_hz, rx_count=rx_count, status=status,
    )


def _fast_tip(w: list[int]) -> TactileTip:
    """Decode one tip of the fast block: (Px8,Py8) (Fx8,Fy8) (Fz8,status8)."""
    lsb = reg.FAST_TACT_LSB
    return _tactile_tip(
        (w[2] & 0xFF) << 8,                    # status8 = TACT_STATUS[15:8]
        round((w[0] >> 8) * lsb, 1),           # Px8 uint8
        round(_s8(w[0] & 0xFF) * lsb, 1),      # Py8 int8
        round(_s8(w[1] >> 8) * lsb, 1),        # Fx8 int8
        round(_s8(w[1] & 0xFF) * lsb, 1),      # Fy8 int8
        round((w[2] >> 8) * lsb, 1),           # Fz8 uint8
    )
