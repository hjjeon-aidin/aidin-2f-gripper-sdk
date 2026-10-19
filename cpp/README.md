# Aidin Gripper C++ SDK

A C++17 SDK for the Aidin BLDC gripper firmware (DH-Robotics-compatible Modbus RTU).
Builds on Ubuntu and Windows with **one CMake command** — Modbus RTU framing is
implemented in-tree (`src/modbus_rtu.cpp`, termios/Win32 direct serial I/O), so
there are **no external dependencies** beyond a compiler and CMake.

> **Full developer manual:** [MANUAL.md](MANUAL.md) — protocol reference, API details, troubleshooting, FAQ.

```cpp
#include "aidin/gripper.hpp"

aidin::Gripper g;
g.connect("/dev/ttyUSB0");                // Windows: "COM3"
g.activate();
g.home();
g.moveTo(255, /*speed=*/200, /*force=*/128, /*blocking=*/true);   // close
g.moveTo(0,   200, 128, true);                                    // open
```

## Requirements

| | Minimum |
|---|---|
| CMake     | 3.16 |
| C++       | C++17 (GCC 9 / Clang 10 / MSVC 2019) |
| Hardware  | USB ↔ RS485 adapter (FTDI / CH340 / etc.) wired to the gripper's UART1 |

Default link parameters: **115200 baud, 8N1, slave ID 1**.

## Build on Ubuntu

```bash
sudo apt install build-essential cmake git
git clone <this-repo> aidin-2f-gripper-sdk
cd aidin-2f-gripper-sdk/cpp
cmake -B build
cmake --build build -j
```

Run an example:
```bash
./build/01_activate_and_home /dev/ttyUSB0
```

If you get `Permission denied` on the serial port:
```bash
sudo usermod -a -G dialout $USER     # then log out / log back in
```

## Build on Windows

Install [CMake](https://cmake.org/download/) and Visual Studio 2019 / 2022
(Build Tools workload is enough), then:

```powershell
git clone <this-repo> aidin-2f-gripper-sdk
cd aidin-2f-gripper-sdk\cpp
cmake -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

Run an example (replace `COM3` with your adapter's port — check Device Manager):
```powershell
.\build\Release\01_activate_and_home.exe COM3
```

USB-RS485 adapters need their vendor driver installed (FTDI VCP / CH340 / CP210x).
Port numbers above `COM9` are handled automatically.

## Build options

| Option | Default | Effect |
|---|---|---|
| `AIDIN_BUILD_EXAMPLES`        | `ON`  | Build the three example executables |

Example:
```bash
cmake -B build -DAIDIN_BUILD_EXAMPLES=OFF
```

## Public API

| Method | Purpose |
|---|---|
| `connect(port, baud=115200, parity='N', slaveId=1)` | Open serial link (shared automatically with other `Gripper`s already on the same port - see "Multiple grippers on one bus" below) |
| `disconnect()`                                       | Close link (only actually closes the port once every sharer has disconnected) |
| `activate()`                                         | Set rACT=1, wait until gripper reports `Active` |
| `stop()`                                             | Stop an in-flight motion (rGTO=0), keep holding |
| `home(timeout=15s)`                                  | Trigger homing, block until `gHOM=1` |
| `fullInit(timeout=30s)`                              | Full initialization (`ACTION=0xA5`, protocol v1.12.0): activate + stroke calibration — full stroke sweep ×2, measured stroke flash-saved (read at `0x018B`) |
| `setTrajLimits(maxVel, maxAcc)`                      | Trajectory limits (protocol v1.11.0, RAM only): vel 1..150 rad/s, acc 1..800 rad/s² — applied via `DEV_CMD 0x0E` |
| `moveTo(pos, speed, force, blocking, timeout=5s)`    | Position 0=open … 255=close. Re-issuing the same position pulses rGTO (v1.9.1 semantics) |
| `emergencyRelease(closeDirection=false)`             | Pulse rATR while keeping rACT set |
| `readState() -> GripperState`                        | Snapshot gACT, gHOM, gSTA, gOBJ, gPO, gCU, gSP, gV, gFLT |
| `isAtTarget()` / `hasFault()`                        | Convenience predicates |
| `readFastState() -> FastState`                       | Fast-poll block `0x02C0` (fw ≥ v1.15.0): status, position, current, fault + both tips' tactile (8-bit) in **one** read |
| `readTactile() -> TactilePair`                       | Both fingertips' tactile resultant from `0x0220~`: `[0]` right, `[1]` left — contact point [mm], Fx/Fy/Fz [N], `link()` |
| `readTactileRaw(tip) -> TactileRaw`                  | One tip's raw 69-channel stream (bring-up); `tip` = `reg::TACT_TIP_RIGHT` / `TACT_TIP_LEFT` |
| `tactileZero(tips=both)`                             | Sensor no-load zero (fw ≥ v1.17.0) — pads untouched; tip reads `Waiting` ~1.25 s, then `Live` |
| `tactileCommand(code, tips, arg=0)`                  | Any structured tactile command (`reg::TACT_SCMD_*`) |
| `sensorZero()`                                       | Firmware software tare (`DEV_CMD 0x08`) of Fx/Fy/Fz, RAM only |
| `readRegister(addr)` / `writeRegister(addr, value)`  | Raw Modbus access (for installer / factory tooling) |

All methods throw `aidin::GripperError` (derived from `std::runtime_error`)
on Modbus failure or timeout.

## Multiple grippers on one bus (RS485 multi-drop)

RS485 is a shared, half-duplex bus: several grippers can sit on the same
wire pair as long as each has a distinct Modbus slave address (set once via
`CFG_MB_ADDR`, one device at a time, before wiring them together).

```cpp
aidin::Gripper a, b;
a.connect("/dev/ttyUSB0", 115200, 'N', /*slaveId=*/1);
b.connect("/dev/ttyUSB0", 115200, 'N', /*slaveId=*/2);   // reuses a's connection
```

The second `connect()` call to the same port path transparently shares the
first one's already-open serial connection instead of opening the port
again (which fails outright on Windows and can corrupt the bus on Linux).
`baudrate`/`parity` must match across every `connect()` call for a given
port - those belong to the physical link, not to one device - a mismatch
throws `GripperError`. Each Modbus transaction is serialized with an
internal mutex, so driving both grippers from separate threads is safe too.

## Examples

| File | Demonstrates |
|---|---|
| `examples/01_activate_and_home.cpp` | Minimal startup sequence |
| `examples/02_open_close_loop.cpp`   | Repeated open/close with blocking moves and SIGINT handling |
| `examples/03_status_monitor.cpp`    | 100 ms polling of state changes |
| `examples/04_tactile_monitor.cpp`   | Fingertip tactile sensors: link state, `--zero`, stream contact point + force @ 20 Hz |
| `examples/05_fast_poll.cpp`         | Fast-poll block `0x02C0` as fast as the bus allows, prints the achieved rate |

## Register map reference

The Modbus register layout used here (DH-Robotics compatible) lives in
[`include/aidin/registers.hpp`](include/aidin/registers.hpp) and mirrors the
firmware-side definitions (firmware repo `aidin-bldc-firmware`:
`Aidin_gripper_functions/gripper/Inc/aidin_modbus.h`). Key registers:

| Addr | Direction | Field |
|---|---|---|
| 0x0100 | write | rACT \| rHOM \| rGTO \| rATR \| rADR |
| 0x0101 | write | rSP (speed 0..255) |
| 0x0102 | write | rFR (force 0..255) |
| 0x0103 | write | rPR (position 0=open .. 255=close) |
| 0x0200 | read  | gACT \| gHOM \| gGTO \| gSTA \| gOBJ |
| 0x0201 | read  | gFLT (fault code) |
| 0x0203 | read  | gPO (actual position) |
| 0x0204 | read  | gCU (actual current) |
| 0x0205 | read  | gSP (actual speed) |
| 0x0206 | read  | gV  (supply voltage, 0.1 V units) |
| 0x0220~0x022F | read | tactile, right tip then left tip: status, posX/posY [mm×100], Fx/Fy/Fz [N×100], rate, count |
| 0x02C0~0x02C9 | read | fast-poll block: status, gPO, gCU, both tips' tactile (8-bit), gFLT |

Tactile and fast-block details (bits, coordinate frame, link states,
8-bit decoding): [MANUAL.md §7.5–7.6](MANUAL.md#75-tactile-sensor-registers)
and the [root README](../README.md#tactile-sensor--촉각-센서-0x0220).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `connect failed: open ...: No such file or directory` | Wrong port name or adapter not plugged in |
| `connect failed: open ...: Permission denied`         | (Linux) user not in `dialout` group |
| All reads time out (`response timeout`)               | Wrong baud rate, parity, or slave ID; or RS485 A/B swapped |
| `activate timeout` but link works                     | Firmware not powered or fault latched — read `gFLTO` |

## License

SDK code: MIT. No third-party components.
