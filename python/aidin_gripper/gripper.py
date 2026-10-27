"""High-level Python wrapper for the AIDIN BLDC gripper (Modbus RTU).

Mirrors the C++ SDK API:
    g = Gripper()
    g.connect("/dev/ttyUSB0")
    g.activate()
    g.home()
    g.move_to(255, speed=200, force=128, blocking=True)
"""
from __future__ import annotations

import inspect
import threading
import time
from typing import Optional, Sequence

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

from . import registers as reg
from .types import (
    FastState,
    FirmwareInfo,
    GripperError,
    GripperStage,
    GripperState,
    ObjectState,
    TactileRaw,
    TactileTip,
)


_POLL_INTERVAL_S = 0.02
_DEFAULT_TIMEOUT_S = 0.5
# rHOM/rATR are rising-edge triggered and rGTO re-execution needs a 0->1
# pulse (protocol v1.9.1) - hold each edge for this long.
_EDGE_PULSE_S = 0.02
# activate(): how long gACT may still read 0 after the rACT write before the
# activation counts as refused. The firmware sets gACT=1 on the activating
# write itself and drops it back to 0 when the motor cannot start.
_ACTIVATE_GRACE_S = 0.3


def _device_id_kw() -> str:
    """pymodbus keyword for the slave address.

    pymodbus 3.10+ only accepts ``device_id=``; 3.1-3.9 only ``slave=``
    (3.1-3.6 would silently swallow an unknown keyword, so probing by trial is
    not safe). Picked once from the installed client's signature.
    """
    try:
        params = inspect.signature(ModbusSerialClient.read_holding_registers).parameters
    except (TypeError, ValueError):
        return "slave"
    return "device_id" if "device_id" in params else "slave"


_ID_KW = _device_id_kw()


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

    Every Modbus transaction is serialized per port, so separate Gripper
    instances may be used from separate threads. One instance is not meant
    to be driven from several threads at once (its commands are multi-step).
    """

    def __init__(self) -> None:
        self._client: Optional[ModbusSerialClient] = None
        self._shared: Optional[_SharedPort] = None
        self._port: Optional[str] = None
        self._slave_id: int = 1
        self._last_action: int = 0       # last ACTION written (rGTO pulse decision)
        self._last_position: int = -1    # last rPR written, -1 = none yet

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

        port: "/dev/ttyUSB0" on Linux, "COM3" on Windows. The firmware is
        fixed at 8N1, so keep parity="N".

        If another Gripper already has this port open (e.g. a second
        device at a different slave_id on the same RS485 bus), the
        existing connection is reused instead of opening the port again -
        baudrate/parity must match, since those belong to the shared
        physical link, not to one slave_id. response_timeout_s only takes
        effect on the connect() call that actually opens the port.
        """
        if not (1 <= slave_id <= 247):
            raise GripperError("slave id out of range (1..247)")
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
        self._last_action = 0
        self._last_position = -1

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
    # Every command writes rACT=1, so after a fault (firmware >= v1.23 clears
    # rACT) any command re-activates the gripper - but the firmware does not
    # execute the move/home/release bits of that first re-activating write.
    # Recovery: remove the cause -> activate() -> re-send the command. The
    # blocking waits below raise GripperError (naming gFLT/gFLTO) as soon as
    # the gripper reports gACT=0 or a fault, instead of running into a timeout.
    def activate(self, timeout_s: float = 3.0) -> None:
        """Set rACT=1 and wait until gSTA reports ACTIVE.

        This is also the fault recovery step (firmware >= v1.23): remove the
        cause, then activate() - FAULT_CLEAR is not needed. Fails fast,
        naming gFLT/gFLTO, when gACT falls back to 0 (the cause is still
        present, or the motor did not start within ~1.5 s).
        """
        self._write1(reg.ACTION, reg.RACT)
        start = time.monotonic()
        deadline = start + timeout_s
        seen_active = False
        while True:
            s = self.read_state()
            if s.stage == GripperStage.ACTIVE:
                return
            now = time.monotonic()
            if s.activated:
                seen_active = True
            elif seen_active or now - start >= _ACTIVATE_GRACE_S:
                raise GripperError(
                    f"activate failed: gACT fell back to 0 ({_state_detail(s)}) - "
                    "remove the cause and call activate() again (firmware <= v1.22: "
                    "FAULT_CLEAR, deactivate(), then activate())")
            if now >= deadline:
                raise GripperError(
                    f"activate timeout: gSTA never reached ACTIVE ({_state_detail(s)})")
            time.sleep(_POLL_INTERVAL_S)

    def deactivate(self) -> None:
        """Set rACT=0 (does not wait). Ends homing and force control (fw >= v1.25.1)."""
        self._write1(reg.ACTION, 0)

    def stop(self) -> None:
        """Stop an in-flight motion (rGTO=0). The motor stays active and holding."""
        self._write1(reg.ACTION, reg.RACT)

    def home(self, timeout_s: float = 15.0) -> None:
        """Trigger homing and block until gHOM=1.

        The firmware homes on the rising edge of rHOM. We clear rHOM first,
        wait briefly, then assert it. Homing runs at 100 % current whatever
        rFR is (firmware >= v1.25.1).
        """
        self._write1(reg.ACTION, reg.RACT)              # ensure rHOM low
        time.sleep(_EDGE_PULSE_S)
        self._write1(reg.ACTION, reg.RACT | reg.RHOM)    # rising edge
        time.sleep(0.10)                                  # let firmware clear gHOM

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            _raise_if_stopped(s, "home")
            if s.homed:
                self._write1(reg.ACTION, reg.RACT)        # release rHOM
                return
            time.sleep(0.05)
        self._write1(reg.ACTION, reg.RACT)                # don't leave rHOM set
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
        time.sleep(_EDGE_PULSE_S)
        self._write1(reg.ACTION, reg.ACTION_FULL_INIT)

        # DEV_STATUS may still hold a previous command's OK — only trust a
        # terminal value after RUNNING has been observed. A rejection (already
        # homing/calibrating) reports ERROR without ever showing RUNNING.
        saw_running = False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            _raise_if_stopped(s, "full-init")
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

        speed, force: 0..255 (force = current limit, rated current x force/255;
        0 = no force). Re-issuing the same position pulses rGTO 0->1 so the
        firmware plans a new trajectory (v1.9.1 rule). blocking=True polls
        until gOBJ != MOVING and raises if the move was not executed (gGTO=0:
        not homed, force control active, or the write re-activated the
        gripper after a fault - re-send it then).
        """
        _u8(position, "position")
        _u8(speed,    "speed")
        _u8(force,    "force")

        self._write1(reg.SPEED, speed)
        self._write1(reg.FORCE, force)

        # Firmware executes goto only on an rGTO rising edge OR an rPR change
        # (protocol v1.9.1). Re-issuing the same position while rGTO is
        # already high would be silently ignored — pulse rGTO 0->1 instead.
        same_pos = self._last_position == position
        if same_pos and (self._last_action & reg.RGTO):
            self._write1(reg.ACTION, reg.RACT)
            time.sleep(_EDGE_PULSE_S)
        elif not same_pos:
            self._write1(reg.POSITION, position)        # rPR first, ACTION last
        self._write1(reg.ACTION, reg.RACT | reg.RGTO)

        if not blocking:
            return

        time.sleep(0.03)  # let the firmware leave the previous AT_TARGET state
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.read_state()
            _raise_if_stopped(s, "move_to")
            if not s.go_to_active:
                raise GripperError(_not_executed_text(s))
            if s.object_state != ObjectState.MOVING:
                return
            time.sleep(_POLL_INTERVAL_S)
        raise GripperError("move_to timeout")

    def emergency_release(self, close_direction: bool = False) -> None:
        """Emergency auto-release: drive the fingers fully open (or closed).

        Pulses rATR with rACT held (rADR = direction) and returns without
        waiting. Force control ends (FC_MODE=0); the gripper stays active and
        the next move_to() works normally. After a fault the write only
        re-activates the gripper and the release is skipped (firmware
        >= v1.23) - call activate() first, then emergency_release().
        """
        # rATR is a rising edge processed only together with rACT=1 —
        # writing rATR alone would just deactivate the motor.
        bits = reg.RACT | reg.RATR | (reg.RADR if close_direction else 0)
        self._write1(reg.ACTION, bits)
        time.sleep(_EDGE_PULSE_S)
        self._write1(reg.ACTION, reg.RACT)

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
        8-bit compressed over each field's full scale (Px 32 mm, Py +-12.5 mm,
        Fx/Fy +-35 N, Fz 70 N). Firmware protocol >= v1.15.0; the scales
        decode correctly only with >= v1.19.0.
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

    # ---------------------------- Device information ----------------------------
    def read_serial_number(self) -> str:
        """Serial number ``"<product code>-<YYMM>-<NNNN>"``, e.g. ``"BLDC2FV1-2610-0001"``.

        Firmware >= v1.24.0. Returns ``""`` when nothing is recorded (all
        words 0). Firmware <= v1.23 has no product code: returns ``"YYMM-NNNN"``.
        """
        code = self._read_block(reg.CFG_PRODUCT_CODE, reg.CFG_PRODUCT_CODE_WORDS)
        number, yymm = self._read_block(reg.CFG_SERIAL_LO, 2)
        return _format_serial(code, yymm, number)

    def read_firmware_info(self) -> FirmwareInfo:
        """Firmware version + build info (0x01C4..0x01CB) in one transaction.

        Firmware >= v1.25.0. On older firmware every field is 0:
        ``has_version`` is False and ``str(info)`` reads
        "<= 1.24 (no version registers)".
        """
        return _decode_firmware_info(self._read_block(reg.FW_VER_MAJOR, reg.FW_INFO_WORDS))

    # ---------------------------- Tactile sensor ----------------------------
    # Needs CFG_SENSOR_TYPE = SENSOR_TACTILE (2) - write it, then SAVE_CONFIG
    # (applies immediately, no reboot).
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

    def _transact(self, what: str, method: str, *args, **kwargs):
        """Run one pymodbus request under the port lock; every failure -> GripperError.

        pymodbus <= 3.6 returns timeouts as error responses, 3.9+ raises them
        (ModbusIOException / ConnectionException); both end up here.
        """
        shared = self._require_shared()
        kwargs[_ID_KW] = self._slave_id
        try:
            with shared.lock:
                rr = getattr(shared.client, method)(*args, **kwargs)
        except (ModbusException, OSError) as exc:
            raise GripperError(f"{what} failed: {exc}") from exc
        if rr is None or rr.isError():
            raise GripperError(f"{what} failed: {rr}")
        return rr

    def _write1(self, addr: int, value: int) -> None:
        self._transact(f"write_register(0x{addr:04X}, 0x{value & 0xFFFF:04X})",
                       "write_register", addr, value & 0xFFFF)
        if addr == reg.ACTION:
            self._last_action = value & 0xFFFF
        elif addr == reg.POSITION:
            self._last_position = value & 0xFF

    def _write_block(self, addr: int, values: list[int]) -> None:
        self._transact(f"write_registers(0x{addr:04X}, {len(values)})",
                       "write_registers", addr, [v & 0xFFFF for v in values])

    def _read_block(self, addr: int, count: int) -> list[int]:
        rr = self._transact(f"read_holding_registers(0x{addr:04X}, {count})",
                            "read_holding_registers", addr, count=count)
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


_FAULT_NAMES = {
    0: "OK", 1: "not initialized", 2: "invalid command", 3: "invalid parameter",
    4: "over-current", 5: "over-temperature", 6: "under-voltage", 7: "motor stall",
    8: "sensor error", 9: "comm timeout", 10: "internal error", 11: "gate-driver fault",
}


def _fault_text(code: int) -> str:
    return f"{code} ({_FAULT_NAMES.get(code, 'unknown')})"


def _state_detail(s: GripperState) -> str:
    return (f"gACT={int(s.activated)}, gFLT={_fault_text(s.fault)}, "
            f"gFLTO={_fault_text(s.latched_fault)}")


def _raise_if_stopped(s: GripperState, what: str) -> None:
    """Blocking waits: stop as soon as the motor is off (gACT=0) or faulted."""
    if s.fault or not s.activated:
        raise GripperError(
            f"{what} stopped: gripper not active or faulted ({_state_detail(s)}) - "
            "remove the cause, call activate(), then re-send the command")


def _not_executed_text(s: GripperState) -> str:
    if not s.homed:
        return "move_to not executed: gripper not homed (gHOM=0) - call home() first"
    return ("move_to not executed (gGTO=0): force control is active (FC_MODE != 0), "
            "or this write re-activated the gripper after a fault - re-send the move")


def _decode_ascii(words: Sequence[int]) -> str:
    """2 chars per word, high byte first, ends at the first 0x00 byte."""
    out = []
    for w in words:
        for b in ((w >> 8) & 0xFF, w & 0xFF):
            if b == 0:
                return "".join(out)
            out.append(chr(b) if 0x20 <= b < 0x7F else "?")
    return "".join(out)


def _format_serial(code_words: Sequence[int], yymm: int, number: int) -> str:
    code = _decode_ascii(code_words)
    if not code and not yymm and not number:
        return ""                                   # not recorded
    tail = f"{yymm:04d}-{number:04d}"
    return f"{code}-{tail}" if code else tail       # no product code: firmware <= v1.23


def _decode_firmware_info(w: Sequence[int]) -> FirmwareInfo:
    flags = w[5]
    return FirmwareInfo(
        major=w[0], minor=w[1], patch=w[2],
        git_hash=((w[3] & 0xFFFF) << 16) | (w[4] & 0xFFFF),
        dirty=bool(flags & reg.FW_BUILD_DIRTY),
        no_info=bool(flags & reg.FW_BUILD_NO_INFO),
        build_yymm=w[6], build_ddhh=w[7],
    )


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
    return _tactile_tip(
        (w[2] & 0xFF) << 8,                                          # status8 = TACT_STATUS[15:8]
        round((w[0] >> 8) * reg.FAST_PX_MAX / 255, 2),               # Px8 uint8
        round(_s8(w[0] & 0xFF) * reg.FAST_PY_MAX / 127, 2),          # Py8 int8
        round(_s8(w[1] >> 8) * reg.FAST_FXY_MAX / 127, 2),           # Fx8 int8
        round(_s8(w[1] & 0xFF) * reg.FAST_FXY_MAX / 127, 2),         # Fy8 int8
        round((w[2] >> 8) * reg.FAST_FZ_MAX / 255, 2),               # Fz8 uint8
    )
