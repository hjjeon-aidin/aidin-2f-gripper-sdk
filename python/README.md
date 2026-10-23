# AIDIN Gripper — Python SDK

Pure-Python SDK for the AIDIN BLDC gripper (DH-Robotics Modbus RTU). Runs
on Ubuntu and Windows with **one `pip install` step**. API surface is
deliberately kept close to the [C++ SDK](../cpp/) so the two are easy to
switch between.

```python
from aidin_gripper import Gripper

with Gripper() as g:
    g.connect("/dev/ttyUSB0")                      # Windows: "COM3"
    g.activate()
    g.home()
    g.move_to(255, speed=200, force=128, blocking=True)   # close
    g.move_to(0,   speed=200, force=128, blocking=True)   # open
    print(g.read_state())
```

## Requirements

| | Minimum |
|---|---|
| Python | 3.8+ |
| pymodbus | 3.0+ |
| pyserial | 3.5+ |
| Hardware | USB ↔ RS485 adapter (FTDI / CH340 / CP210x) |

Default link: **115200 baud, 8N1, slave ID 1**.

## Install

### Ubuntu / Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .          # from the python/ directory
```

If you can't access `/dev/ttyUSB0`:
```bash
sudo usermod -a -G dialout $USER     # then log out / log back in
```

### Windows

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .
```

USB-RS485 adapters need their vendor driver (FTDI / CH340 / CP210x).

### Just want the dependencies?

```bash
pip install "pymodbus>=3.0,<4.0" pyserial
```

## Run the examples

| File | Demonstrates |
|---|---|
| `examples/01_activate_and_home.py` | Minimal startup sequence: activate + home |
| `examples/02_open_and_close.py` | One close + one open move (assumes already activated/homed) |
| `examples/03_open_close_loop.py` | Repeated open/close cycles with blocking moves and SIGINT (CTRL+C) handling |
| `examples/04_get_status.py` | Read status once, or `--watch` to poll @ 100 ms and print on change |
| `examples/05_fault_and_emergency_release.py` | Inspect `fault`/`latched_fault`, call `emergency_release()`, recover with `activate()` |
| `examples/06_full_quickstart_demo.py` | End-to-end: connect → activate → home → close → open → status |
| `examples/07_multi_gripper_two_slaves.py` | Two grippers, one RS485 bus, two slave IDs (shared serial connection) |
| `examples/08_tactile_sensor.py` | Fingertip tactile sensors: link state, `--zero`, stream contact point + force; `--enable` sets the sensor type once |
| `examples/09_fast_poll.py` | Fast-poll block `0x02C0`: status + position + current + fault + both tips' tactile in one read, prints the achieved rate |

```bash
cd examples
python 01_activate_and_home.py           /dev/ttyUSB0
python 02_open_and_close.py              /dev/ttyUSB0
python 03_open_close_loop.py             /dev/ttyUSB0 5
python 04_get_status.py                  /dev/ttyUSB0 --watch
python 05_fault_and_emergency_release.py /dev/ttyUSB0 open
python 06_full_quickstart_demo.py        /dev/ttyUSB0
python 07_multi_gripper_two_slaves.py    /dev/ttyUSB0 1 2
python 08_tactile_sensor.py              /dev/ttyUSB0 --zero
python 09_fast_poll.py                   /dev/ttyUSB0
```

Windows: pass `COM3` (or your actual port) instead.

## Multiple grippers on one bus (RS485 multi-drop)

RS485 is a shared, half-duplex bus: several grippers can sit on the same
wire pair as long as each has a distinct Modbus slave address.

1. **Give each device a distinct address first**, one at a time, alone on
   the bus (two units left at the factory default of `1` would both
   answer at once and collide):
   ```python
   from aidin_gripper import Gripper, registers as reg

   with Gripper() as g:
       g.connect("/dev/ttyUSB0")             # default slave_id=1
       g.write_register(reg.CFG_MB_ADDR, 2)  # reassign this unit to slave 2
       # Persist to flash: write the SAVE_CONFIG developer command to
       # reg.DEV_CMD. The command code is factory-tool territory - see
       # "Changing baud rate / slave ID" in ../cpp/MANUAL.md and the
       # firmware protocol docs for the exact value.
   ```
   After the firmware saves and restarts its UART, reconnect at the new
   `slave_id`.

2. **Wire both units onto the same bus**, then just `connect()` two
   `Gripper` instances to the same port with different `slave_id`:
   ```python
   from aidin_gripper import Gripper

   with Gripper() as a, Gripper() as b:
       a.connect("/dev/ttyUSB0", slave_id=1)
       b.connect("/dev/ttyUSB0", slave_id=2)   # reuses a's serial connection
       print(a.read_state())
       print(b.read_state())
   ```
   The second `connect()` call to the same port path transparently shares
   the first one's already-open serial connection instead of opening the
   port again (which fails on Windows and can corrupt the bus on Linux).
   `baudrate`/`parity` must match across every `connect()` call for a
   given port — those are properties of the physical link, not of one
   device. Each Modbus transaction (one register read or write) is
   serialized with an internal lock, so it's safe to drive both grippers
   from separate threads too. See
   [`examples/07_multi_gripper_two_slaves.py`](examples/07_multi_gripper_two_slaves.py).

## API at a glance

| Method | Purpose |
|---|---|
| `connect(port, baudrate=115200, parity='N', slave_id=1)` | Open serial link (shared automatically with other `Gripper`s already on the same port) |
| `disconnect()`                                            | Close link (only actually closes the port once every sharer has disconnected) |
| `slave_id` (property)                                     | This instance's Modbus slave address |
| `activate(timeout_s=3.0)`                                 | rACT=1, wait for ACTIVE |
| `deactivate()`                                            | rACT=0 |
| `home(timeout_s=15.0)`                                    | Rising-edge homing, wait for gHOM=1 |
| `full_init(timeout_s=30.0)`                               | Full initialization (`ACTION=0xA5`, protocol v1.12.0): activate + stroke calibration — full stroke sweep ×2, stroke flash-saved (read at `0x018B`) |
| `set_traj_limits(max_vel, max_acc)`                       | Trajectory limits (protocol v1.11.0, RAM only): vel 1..150 rad/s, acc 1..800 rad/s² — applied via `DEV_CMD 0x0E` |
| `move_to(position, speed=128, force=128, blocking=False, timeout_s=5.0)` | Position 0=open … 255=close |
| `emergency_release(close_direction=False)`                | Latched release |
| `read_state() -> GripperState`                            | Full status snapshot |
| `is_at_target()` / `has_fault()`                          | Convenience predicates |
| `read_fast_state() -> FastState`                          | Fast-poll block `0x02C0` (fw ≥ v1.15.0): status, position, current, fault + both tips' tactile (8-bit) in **one** read |
| `read_tactile() -> (TactileTip, TactileTip)`              | Both fingertips' tactile resultant `(right, left)` from `0x0220~` (0.01 mm / 0.01 N) |
| `read_tactile_raw(tip) -> TactileRaw`                     | One tip's raw 69-channel stream (bring-up); `tip` = `registers.TACT_TIP_RIGHT` / `TACT_TIP_LEFT` |
| `tactile_zero(tips=TACT_TIP_BOTH)`                        | Sensor no-load zero (fw ≥ v1.17.0) — pads untouched; tip reads `WAITING` ~1.25 s, then `LIVE` |
| `tactile_command(code, tips, arg=0)`                      | Any structured tactile command (`registers.TACT_SCMD_*`) |
| `sensor_zero()`                                           | Firmware software tare (`DEV_CMD 0x08`) of Fx/Fy/Fz, RAM only |
| `read_register(pdu_addr)` / `write_register(pdu_addr, value)` | Raw Modbus access |

`Gripper` is a context manager — `with Gripper() as g:` auto-closes the port.

All failures raise `aidin_gripper.GripperError` (a `RuntimeError` subclass).

## `GripperState` dataclass

```python
@dataclass
class GripperState:
    activated:     bool
    homed:         bool
    go_to_active:  bool
    stage:         GripperStage   # RESET / ACTIVATING / ACTIVE
    object_state:  ObjectState    # MOVING / OPEN_CONTACT / CLOSED_CONTACT / AT_TARGET
    fault:         int            # 0 = OK
    position_echo: int            # gPR
    position:      int            # gPO (0..255)
    current:       int            # gCU (0..255)
    speed:         int            # gSP (0..255)
    voltage:       float          # V
    latched_fault: int            # gFLTO
```

## Tactile sensor

Needs firmware protocol ≥ v1.18.0 and `CFG_SENSOR_TYPE = 2` (tactile).
Set it once with `python examples/08_tactile_sensor.py PORT --enable`
(writes `0x018A`, `SAVE_CONFIG`), then power-cycle. Tip 1 = **right**
finger, tip 2 = **left** finger.

```python
from aidin_gripper import Gripper, TactileLink

with Gripper() as g:
    g.connect("/dev/ttyUSB0")
    right, left = g.read_tactile()          # one 16-register read
    if right.link == TactileLink.LIVE:
        print(right.pos_x, right.pos_y)      # contact point [mm]
        print(right.fx, right.fy, right.fz)  # force [N], fz > 0 = pressing
    g.tactile_zero()                         # pads untouched!
```

```python
@dataclass
class TactileTip:
    valid, saturated, timeout, connected, raw_stream, contact, seen, raw_fresh: bool
    pos_x:    float  # mm, 0..32 from the wrist-side edge of the cell area toward the fingertip; 0 = no contact point
    pos_y:    float  # mm, +-12.5 across the pad width
    fx, fy:   float  # N, shear along +pos_x / +pos_y (after tare)
    fz:       float  # N, normal force, > 0 = pressing (after tare)
    rate_hz:  int    # result frames in the last second (~100); read_tactile() only
    rx_count: int    # result frames since boot (wraps); read_tactile() only
    status:   int    # raw TACT_STATUS word
    link             # property -> TactileLink
    has_contact_point  # property: valid and pos_x > 0
```

`TactileLink`: `NOT_DETECTED` (never seen — not fitted / wiring / sensor
type ≠ 2) · `DISCONNECTED` (frames stopped) · `RAW_STREAM` (sensor sends
raw channels, not results) · `WAITING` (sensor zeroing: ~4.5 s after boot,
~1.25 s after `tactile_zero()`) · `LIVE`.

## Fast-poll block (`0x02C0`)

`read_fast_state()` reads the 10-register block in **one** transaction
(~4 ms at 115200 bps) and decodes it into:

```python
@dataclass
class FastState:
    activated, homed, go_to_active: bool
    stage:        GripperStage
    object_state: ObjectState
    position:     int          # gPO
    current:      int          # gCU
    fault:        int          # gFLT
    right:        TactileTip   # 8-bit: ~0.1 mm / ~0.27 N steps, rate_hz = rx_count = 0
    left:         TactileTip
```

Use it when you need gripper state and tactile together at a high rate;
use `read_state()` / `read_tactile()` when you need voltage, speed, the
latched fault or 0.01-resolution tactile values. Register layout and
decoding: [../README.md](../README.md#fast-poll-block--고속-폴링-블록-0x02c00x02c9).

## C++ vs Python API mapping

| C++                             | Python                          |
|---------------------------------|---------------------------------|
| `g.connect("/dev/ttyUSB0")`     | `g.connect("/dev/ttyUSB0")`     |
| `g.activate()`                  | `g.activate()`                  |
| `g.home()`                      | `g.home()`                      |
| `g.moveTo(255, 200, 128, true)` | `g.move_to(255, 200, 128, blocking=True)` |
| `g.readState()`                 | `g.read_state()`                |
| `g.isAtTarget()`                | `g.is_at_target()`              |
| `g.readFastState()`             | `g.read_fast_state()`           |
| `auto tips = g.readTactile();` (`tips[0]` right) | `right, left = g.read_tactile()` |
| `g.tactileZero()`               | `g.tactile_zero()`              |
| `aidin::reg::ACTION`            | `aidin_gripper.registers.ACTION` |
| `try { ... } catch (GripperError& e)` | `try: ... except GripperError as e:` |

The same register map, same default arguments, same blocking semantics.

## Register map

See [`aidin_gripper/registers.py`](aidin_gripper/registers.py). Mirrors the
firmware-side definitions in `aidin_modbus.h`. Full protocol details:
[`../cpp/MANUAL.md`](../cpp/MANUAL.md).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Cannot open serial port '/dev/ttyUSB0'`         | Wrong port or adapter not connected |
| `Cannot open serial port` (Linux, Permission)    | User not in `dialout` group |
| `read_holding_registers failed: ... timeout`     | Wrong baud / parity / slave ID; or RS485 A↔B swapped |
| `activate timeout`                               | Power supply low, or latched fault — read `state.latched_fault` |
| Gripper ignores `move_to` after one move         | `rATR` still latched after `emergency_release` — call `activate()` again |
| Tactile tip stays `NOT_DETECTED`                 | `CFG_SENSOR_TYPE` ≠ 2 (run example 08 with `--enable`, power-cycle), or sensor cable |
| Tactile tip stays `WAITING`                      | Sensor still zeroing (~4.5 s after power-up), or its data stream is broken |
| `tactile command rejected`                       | Sensor type not tactile, values invalid, or force control is using the sensor as feedback |
| `read_fast_state()` fails with illegal address   | Firmware older than protocol v1.15.0 — use `read_state()` / `read_tactile()` |

## License

MIT. `pymodbus` is BSD-3-Clause.
