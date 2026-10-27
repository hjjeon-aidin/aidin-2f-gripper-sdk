# Aidin Gripper C++ SDK — Developer Manual

This manual documents the C++ SDK in depth: protocol, API surface, state
semantics, and operational notes. For a quick build/run walk-through see
[README.md](README.md).

---

## Table of Contents

1. [Architecture overview](#1-architecture-overview)
2. [Installation](#2-installation)
3. [Quick start](#3-quick-start)
4. [Connection parameters](#4-connection-parameters)
5. [Operational state machine](#5-operational-state-machine)
6. [API reference](#6-api-reference)
7. [Protocol reference (Modbus registers)](#7-protocol-reference-modbus-registers)
8. [Error handling](#8-error-handling)
9. [Threading model](#9-threading-model)
10. [Examples walkthrough](#10-examples-walkthrough)
11. [Troubleshooting](#11-troubleshooting)
12. [FAQ](#12-faq)
13. [Extending the SDK](#13-extending-the-sdk)

---

## 1. Architecture overview

```
+---------------------------+               +----------------------------+
|  Your C++ application     |               |  Aidin BLDC gripper        |
|                           |   RS485       |  (STM32G431CB)             |
|  aidin::Gripper           |  Modbus RTU   |                            |
|  (this SDK)               |<------------->|  Modbus RTU slave          |
|       ^                   |  115200 8N1   |  (FC03/06/16/23)           |
|       |  PImpl            |               |  DH-Robotics register map  |
|  ModbusRtu (in-tree)      |               |  Holding regs 0x0100..02C9 |
+---------------------------+               +----------------------------+
        |
        |  USB-RS485 adapter (FTDI / CH340 / CP210x)
        |  Linux: /dev/ttyUSB0   Windows: COM3
```

- Modbus RTU framing is **implemented in-tree** (`aidin::ModbusRtu`,
  `src/modbus_rtu.cpp`): the SDK builds the ADU (function code + CRC16)
  itself and drives the serial port directly — termios + `poll()` on
  POSIX, Win32 COMM API on Windows. FC 0x03 / 0x06 / 0x10 are supported,
  with exception-frame handling, response-echo verification, a stale-RX
  flush before every request, and t3.5 inter-frame spacing.
- The PImpl idiom (`struct Gripper::Impl`) keeps the transport out of the
  public headers, so users only need to include `aidin/gripper.hpp` and
  link `aidin_gripper`.
- **No external dependencies**: a fresh checkout builds with zero extra
  system packages beyond a C++17 compiler and CMake — no FetchContent,
  no network access at configure time.

---

## 2. Installation

### 2.1 Ubuntu (20.04 / 22.04 / 24.04)

```bash
sudo apt update
sudo apt install -y build-essential cmake git
git clone https://github.com/hjjeon-aidin/aidin-2f-gripper-sdk.git
cd aidin-2f-gripper-sdk/cpp
cmake -B build
cmake --build build -j$(nproc)
```

Add yourself to the `dialout` group so you can access `/dev/ttyUSB*`
without `sudo`:
```bash
sudo usermod -a -G dialout $USER
# log out and back in
```

### 2.2 Windows 10/11

Prerequisites:
- [CMake](https://cmake.org/download/) ≥ 3.16 (during install, choose
  *"Add CMake to system PATH"*).
- [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/)
  with the *"Desktop development with C++"* workload (Build Tools alone
  is enough — IDE not required).
- USB-RS485 adapter driver (FTDI / CH340 / CP210x).

```powershell
git clone https://github.com/hjjeon-aidin/aidin-2f-gripper-sdk.git
cd aidin-2f-gripper-sdk\cpp
cmake -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

Resulting binaries: `build\Release\01_activate_and_home.exe`, etc.

### 2.3 Using the SDK from another CMake project

Include it as a subdirectory:
```cmake
add_subdirectory(path/to/aidin-2f-gripper-sdk/cpp)
target_link_libraries(my_app PRIVATE aidin::gripper)
```

Or via FetchContent in your own project:
```cmake
include(FetchContent)
FetchContent_Declare(aidin_sdk
    GIT_REPOSITORY https://github.com/hjjeon-aidin/aidin-2f-gripper-sdk.git
    GIT_TAG        main
    SOURCE_SUBDIR  cpp)
FetchContent_MakeAvailable(aidin_sdk)
target_link_libraries(my_app PRIVATE aidin::gripper)
```

### 2.4 Build options

| CMake option                   | Default | Effect |
|--------------------------------|---------|--------|
| `AIDIN_BUILD_EXAMPLES`         | `ON`    | Build the five example executables. Set `OFF` when consuming from a parent project. |
| `CMAKE_BUILD_TYPE`             | `Release` | Standard CMake — change to `Debug` for debug symbols. |

---

## 3. Quick start

```cpp
#include "aidin/gripper.hpp"
#include <iostream>

int main() try {
    aidin::Gripper g;
    g.connect("/dev/ttyUSB0");                      // Windows: "COM3"

    g.activate();                                   // rACT=1, wait Active
    g.home();                                       // block until homed

    g.moveTo(255, /*speed=*/200, /*force=*/128, /*blocking=*/true);  // close
    std::cout << g.readState() << "\n";

    g.moveTo(0,   200, 128, true);                  // open
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
}
```

---

## 4. Connection parameters

| Parameter   | Default     | Notes |
|-------------|-------------|-------|
| `port`      | (required)  | `/dev/ttyUSB0`, `/dev/ttyACM0` (Linux); `COM3`, `COM10` (Windows — `\\.\COM10` prefix is auto-applied). |
| `baudrate`  | `115200`    | Firmware default. Configurable via register `0x0183`/`0x0184` (firmware protocol ≥ v1.16.0). Supported: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000 (≥ 460800 needs an adapter and RS485 line that can carry it; Linux 1000000 needs `B1000000`). |
| `parity`    | `'N'`       | Keep `'N'` — the firmware is fixed at 8N1. The transport also accepts `'E'`/`'O'`, which the gripper does not answer. |
| `slaveId`   | `1`         | Firmware default. Configurable via register `0x0182` (range 1..247). |

Data bits and stop bits are fixed at 8 / 1 (`8N1`).

### Changing baud rate / slave ID

```cpp
g.deactivate();                                                // REBOOT is refused while active
g.writeRegister(aidin::reg::CFG_MB_ADDR,    7);                // slave 7
g.writeRegister(aidin::reg::CFG_MB_BAUD_LO, 230400 & 0xFFFF);  // raw bps, LO word
g.writeRegister(aidin::reg::CFG_MB_BAUD_HI, 230400 >> 16);     // raw bps, HI word
g.writeRegister(aidin::reg::DEV_CMD, aidin::reg::DEV_CMD_SAVE_CONFIG);  // persist to flash
std::this_thread::sleep_for(std::chrono::milliseconds(100));   // flash write ~25 ms
if (g.readRegister(aidin::reg::DEV_STATUS) != aidin::reg::DEV_STATUS_OK) { /* rejected */ }
g.writeRegister(aidin::reg::DEV_CMD, aidin::reg::DEV_CMD_REBOOT);       // ACK, then reset
g.disconnect();
std::this_thread::sleep_for(std::chrono::seconds(1));
g.connect(port, 230400, 'N', 7);                               // retry until it answers
```
The new address/baud take effect at the **next boot**: `DEV_CMD_REBOOT`
(firmware ≥ v1.20.0) resets the gripper right after acknowledging the write;
the first reply comes 0.5 – 0.9 s later. `REBOOT` ends with
`DEV_STATUS_ERROR` (and no reset) while the motor is active, and on
firmware ≤ v1.19 — power-cycle instead. Until the reboot the gripper keeps
talking at the old settings. If the baud is not in the supported list, or
the address is outside 1..247, `SAVE_CONFIG` ends with `DEV_STATUS_ERROR`,
nothing is saved, and the register reads back the previously configured
value. Every device on one RS485 bus must use the same baud. Firmware
older than protocol v1.16.0 stores the value but always runs at 115200 —
do not change the baud on it.

---

## 5. Operational state machine

```
   power-on
       |
       v
   [ Reset ] ---- activate() ----> [ Activating ] -- internal OK --> [ Active ]
       ^                                                                 |
       |                                                                 | home()
       |                                                                 v
       |                                                            [ Homing ]
       |                                                                 |
       |                                                                 | gHOM=1
       |                                                                 v
       +--------- deactivate() ----------------------------------- [ Operational ]
                                                                         |
                                                              moveTo() / emergencyRelease()
```

| Firmware reports | SDK enum                       | Meaning |
|------------------|--------------------------------|---------|
| `gSTA = 0`       | `GripperStage::Reset`          | Not activated. |
| `gSTA = 1`       | `GripperStage::Activating`     | Activation in progress. |
| `gSTA = 3`       | `GripperStage::Active`         | Fully active and ready for commands. |
| `gOBJ = 0`       | `ObjectState::Moving`          | Fingers in motion. |
| `gOBJ = 1`       | `ObjectState::OpenContact`     | Reserved — current firmware never reports it. |
| `gOBJ = 2`       | `ObjectState::ClosedContact`   | Reserved — current firmware never reports it. |
| `gOBJ = 3`       | `ObjectState::AtTarget`        | Move finished — also when an object stopped the fingers before the target. |

**Grasp detection.** Contact is not reported through `gOBJ` yet — support for
`OpenContact`/`ClosedContact` is planned for the next firmware version. Until then, after `AtTarget`,
compare `position` (gPO) with the commanded target (gPR): a closing move that
stopped clearly short of the target means an object is held, and `current`
(gCU) stays near the force limit. Choose the tolerance for your object.

**Edge-triggered commands.** The firmware reacts on the *rising edge* of
`rHOM` and `rATR`. The SDK handles this for you (it writes `rHOM=0`,
sleeps briefly, then `rHOM=1`), but if you bypass via `writeRegister()`
you must do the same.

**Faults** (firmware ≥ v1.23.0). A fault while active (codes 4, 5, 6, 8,
10, 11) — or a motor that does not start within 1.5 s of activation —
takes the gripper back to `Reset`: `gACT` = 0, `gSTA` = 0, `gGTO` = 0, and
the firmware clears `ACTION` (`rACT` reads 0). There is no automatic
restart. Deactivation, by a fault or by `deactivate()`, also stops homing
and stroke calibration and ends force control (`FC_MODE` = 0; v1.25.1); a
completed homing reference (`gHOM` = 1) is kept.

Recovery: **remove the cause → `activate()` → re-send the command.**
`FAULT_CLEAR` is not needed (it only clears the `gFLTO` history). Every
SDK command writes `rACT` = 1, so any command re-activates the gripper —
but the firmware does not execute `rGTO`/`rHOM`/`rATR` on that first
re-activating write (`rSP`/`rFR` and `0xA5` are applied). A normal
activation from `Reset` that the host started processes every bit of its
write (v1.25.1). On firmware v1.19.1 – v1.22.0 `rACT` stayed 1 after a
fault: there, `FAULT_CLEAR`, `deactivate()`, then `activate()`.

---

## 6. API reference

All classes live in namespace `aidin`. Headers:
- [`include/aidin/gripper.hpp`](include/aidin/gripper.hpp) — main API.
- [`include/aidin/types.hpp`](include/aidin/types.hpp) — `GripperState`, `FastState`, `FirmwareInfo`, `TactileTip`, `TactileRaw`, enums.
- [`include/aidin/registers.hpp`](include/aidin/registers.hpp) — register/bit constants.

### 6.1 Class `Gripper`

```cpp
class Gripper {
    Gripper();
    ~Gripper();
    Gripper(Gripper&&) noexcept;             // movable
    Gripper(const Gripper&) = delete;        // non-copyable

    void connect(const std::string& port,
                 int  baudrate = 115200,
                 char parity   = 'N',
                 int  slaveId  = 1);
    void disconnect() noexcept;
    bool isConnected() const noexcept;
    void setResponseTimeout(std::chrono::milliseconds t);

    void activate(std::chrono::milliseconds timeout = 3s);
    void deactivate();
    void stop();
    void home(std::chrono::milliseconds timeout = 15s);
    void fullInit(std::chrono::milliseconds timeout = 30s);
    void setTrajLimits(uint16_t maxVel, uint16_t maxAcc);
    void moveTo(uint8_t position,
                uint8_t speed   = 128,
                uint8_t force   = 128,
                bool    blocking = false,
                std::chrono::milliseconds timeout = 5s);
    void emergencyRelease(bool closeDirection = false);

    GripperState readState();
    bool         isAtTarget();
    bool         hasFault();
    FastState    readFastState();                 // 0x02C0 block, one read

    std::string  readSerialNumber();              // "BLDC2FV1-2610-0001", "" = not recorded
    FirmwareInfo readFirmwareInfo();              // 0x01C4 block, one read

    TactilePair  readTactile();                   // {right, left}
    TactileRaw   readTactileRaw(uint16_t tip);    // reg::TACT_TIP_RIGHT / _LEFT
    void         tactileZero(uint16_t tips = reg::TACT_TIP_BOTH);
    void         tactileCommand(uint16_t code, uint16_t tips, uint16_t arg = 0);
    void         sensorZero();

    uint16_t readRegister(uint16_t pduAddr);
    void     writeRegister(uint16_t pduAddr, uint16_t value);
};
```

#### `connect()`

Opens the serial port and configures the Modbus context. Throws
`aidin::GripperError` if the port cannot be opened, the slave does not
respond, or the parameters are invalid. Calling `connect()` twice on the
same instance silently disconnects the previous link first.

#### `activate(timeout)`

Writes `rACT=1` to register `0x0100`, then polls `0x0200..0x0207` every
20 ms until `gSTA` reports `Active` (`0x3 << 4`). The firmware sets
`gACT` = 1 on the activating write itself; if `gACT` falls back to 0 (a
fault whose cause is still present, or the motor did not start within
~1.5 s), or is still 0 after 300 ms, `activate()` throws at once with a
message naming `gFLT` and `gFLTO` — otherwise it throws on timeout. This
is the first thing to do after power-on, and the recovery step after a
fault (§5).

#### `deactivate()`

Writes `rACT=0`. Does **not** wait — the firmware will free the motor
and the next `readState()` will eventually report `Reset`. Homing, stroke
calibration and force control end (`FC_MODE` = 0, firmware ≥ v1.25.1).
Use this for a clean shutdown.

#### `stop()`

Writes `rACT` alone (`rGTO` 1→0): an in-flight motion stops and the
position regulator keeps holding where the fingers are. The gripper stays
active.

#### `home(timeout)`

Triggers a homing sequence. Internally:
1. Writes `rACT|0` (ensures `rHOM` is low).
2. Sleeps 20 ms so the firmware sees the low level.
3. Writes `rACT|rHOM` (rising edge).
4. Sleeps 100 ms (lets firmware clear `gHOM` before we poll).
5. Polls every 50 ms until `gHOM=1`, throwing as soon as `gACT = 0` or
   `gFLT != 0` (the message names `gFLT`/`gFLTO`), or on timeout.
6. Clears `rHOM` so the next call sees a fresh edge.

Default timeout is 15 s — homing on a closed gripper typically completes
in 1–5 s, but a cold start or a high-friction load can take longer.
Homing runs at 100 % current whatever `rFR` is, at half the `rSP` speed
(firmware ≥ v1.25.1; with a very low `rSP`, about < 10, homing from the
closed position may not finish within the firmware's 10 s timeout).

#### `fullInit(timeout)`

Writes `rACT`, then the value `0xA5` to `ACTION` (DH-compatible full
initialization, firmware ≥ v1.12.0): open homing, then a close-contact
stroke measurement saved to flash (`CFG_STROKE`, `0x018B`). Works from
the inactive state. Waits for `DEV_STATUS` RUNNING → OK and `gHOM` = 1;
throws on `DEV_STATUS` ERROR (rejected, or interrupted by a
deactivation), as soon as `gACT = 0` / `gFLT != 0`, or on timeout. The
fingers sweep the full stroke twice — keep the workspace clear.

#### `moveTo(position, speed, force, blocking, timeout)`

| Arg        | Range     | Meaning |
|------------|-----------|---------|
| `position` | 0..255    | 0 = fully open, 255 = fully closed. Linear mapping. |
| `speed`    | 0..255    | Velocity and acceleration limits = trajectory limits (`setTrajLimits`, default 100 rad/s · 400 rad/s²) × speed/255, each at least 1. |
| `force`    | 0..255    | Current limit = rated current × force/255. **0 = no force** — the fingers cannot move. |
| `blocking` | bool      | If `true`, polls until `gOBJ != Moving` (move finished — target reached or stopped on an object). |
| `timeout`  | duration  | Only used when `blocking=true`. |

The SDK writes `rSP` (`0x0101`), `rFR` (`0x0102`), then `rPR` (`0x0103`,
skipped when unchanged), then `rACT|rGTO` at `0x0100`. The firmware runs
a goto on an `rGTO` rising edge or an `rPR` change (v1.9.1), so when the
position equals the last one sent the SDK first writes `rACT` alone (an
`rGTO` 0→1 pulse) to make the firmware plan a fresh trajectory. While
active, `rSP`/`rFR` apply as soon as they are written; written while
inactive they apply at activation (v1.25.1).

With `blocking=true` the call throws as soon as `gACT = 0` or
`gFLT != 0`, and — instead of waiting for a timeout — when `gGTO` reads 0,
i.e. the firmware is not executing the move: not homed, force control
active (`FC_MODE != 0`), or the write re-activated the gripper after a
fault (re-send the move then).

#### `emergencyRelease(closeDirection)`

Writes `rACT|rATR` (`|rADR` to release in the closing direction), waits
20 ms, then writes `rACT`. The firmware drives the fingers fully open (or
closed) and ends force control (`FC_MODE` = 0). There is no latch: the
gripper stays active and the next `moveTo()` works normally. The call
does not wait for the fingers. After a fault the write only
re-activates the gripper and the release is skipped (firmware ≥ v1.23) —
call `activate()` first, then `emergencyRelease()`.

#### `readState()`

Performs one Modbus transaction (function `0x03`, 8 registers) starting
at `0x0200`, then parses into `GripperState`. This is the cheapest way
to learn everything at once — at 115200 baud it costs ≈ 4 ms over the
wire. Prefer this over multiple `readRegister()` calls.

`isAtTarget()` and `hasFault()` are convenience wrappers around
`readState()` — each costs one full transaction. If you call both,
prefer `readState()` directly and check the struct fields.

#### `readFastState()`

One Modbus transaction (function `0x03`, 10 registers) at `0x02C0` — the
fast-poll block (§7.6, firmware protocol ≥ v1.15.0). Returns `FastState`:
`gSTA`/`gOBJ`/`gACT`/`gHOM`/`gGTO`, `gPO`, `gCU`, `gFLT` and both
fingertips' tactile resultant in 8-bit resolution (full scale per field:
Px 32 mm, Py ±12.5 mm, Fx/Fy ±35 N, Fz 70 N). The
firmware rewrites the block atomically, so the result is one consistent
snapshot. ≈ 4 ms per call at 115200 baud (measured max ≈ 248 Hz). Use it
for high-rate loops; use `readState()` when you need `gSP`, `gV`,
`gFLTO` or `gPR`, and `readTactile()` for 0.01 resolution.

#### `readTactile()` / `readTactileRaw(tip)`

`readTactile()` reads 16 registers at `0x0220` (§7.5) in one transaction
and returns `TactilePair` = `{right (tip 1), left (tip 2)}`: contact
point [mm], Fx/Fy/Fz [N], frame rate, and the link state via
`TactileTip::link()`. Requires `CFG_SENSOR_TYPE = 2`; with another
sensor type both tips read `TactileLink::NotDetected` and zero values.

`readTactileRaw(tip)` reads one tip's raw 69-channel diagnostic block (71
registers) — bring-up only, meaningful while that tip's `rawFresh` is set.

#### `tactileZero(tips)` / `tactileCommand(code, tips, arg)` / `sensorZero()`

`tactileZero()` sends the sensor no-load zero (structured command `ZERO`,
firmware ≥ v1.17.0) to the tips in the bitmask (`reg::TACT_TIP_RIGHT`,
`TACT_TIP_LEFT`, default both): it writes `0x01BB~0x01BD` with one FC16,
then `DEV_CMD = 0x0F`, and returns once `DEV_STATUS = OK` (frame sent).
**Keep the pads untouched.** The tip then reads `Waiting` for ≈ 1.25 s and
returns to `Live` with the new zero. Throws if the firmware rejects it —
sensor type not tactile, invalid values, or force control running with
the sensor as feedback. `tactileCommand()` is the generic form
(`reg::TACT_SCMD_*`).

`sensorZero()` is the firmware's software tare (`DEV_CMD 0x08`): the
current Fx/Fy/Fz of every valid tip becomes zero (F/T or tactile). RAM
only — cleared on reboot.

#### `readSerialNumber()`

Two reads — the product code (`0x01C0`, 4 registers) and `0x0188..0x0189`
— assembled into `"<product code>-<YYMM>-<NNNN>"`, e.g.
`"BLDC2FV1-2610-0001"` (firmware ≥ v1.24.0, §7.7). Returns `""` when
nothing is recorded (all words 0). Firmware ≤ v1.23 has no product code,
so the result is just `"YYMM-NNNN"`. Non-printable bytes in the product
code show as `?`.

#### `readFirmwareInfo()`

One read of `0x01C4..0x01CB` (§7.7), decoded into `FirmwareInfo`
(§6.3): version, git hash, dirty / no-git flags, build time.
`toString()` gives `"1.25.1 (1a2b3c4d)"` or `"1.25.1 (1a2b3c4d-dirty)"`.
Firmware ≤ v1.24 reads all 0: `hasVersion()` is false and `toString()`
is `"<= 1.24 (no version registers)"`.

```cpp
const aidin::FirmwareInfo fw = g.readFirmwareInfo();
std::cout << "firmware " << fw << ", built " << fw.buildTime() << "\n";
const std::string sn = g.readSerialNumber();
std::cout << "serial " << (sn.empty() ? "(not recorded)" : sn) << "\n";
```

#### `readRegister()` / `writeRegister()`

Raw Modbus access for configuration registers (`0x0180..0x018C`,
`0x01C0..`), PID gains (`0x0190..0x0196`), force control (`0x01A0..`) and
anything else in the map. The address is the **protocol-level PDU
address**, identical to what is written in the register-map tables — no
offset arithmetic.

### 6.2 `struct GripperState`

```cpp
struct GripperState {
    bool         activated;     // gACT
    bool         homed;         // gHOM
    bool         goToActive;    // gGTO (echoes rGTO while active)
    GripperStage stage;         // gSTA -> Reset / Activating / Active
    ObjectState  object;        // gOBJ -> Moving / OpenContact / ClosedContact / AtTarget
    uint8_t      fault;         // gFLT (0 = OK)
    uint8_t      positionEcho;  // gPR  (echo of last rPR)
    uint8_t      position;      // gPO  0=open, 255=close
    uint8_t      current;       // gCU  0..255
    uint8_t      speed;         // gSP  0..255
    float        voltage;       // V (gV * 0.1)
    uint8_t      latchedFault;  // gFLTO (fault history; kept until DEV_CMD_FAULT_CLEAR, re-activating does not clear it)
};
```

`operator<<(std::ostream&, const GripperState&)` produces a one-line
debug dump.

### 6.3 Device info, tactile and fast-poll types

```cpp
struct FirmwareInfo {               // 0x01C4..0x01CB (fw >= v1.25.0; all 0 on older firmware)
    uint16_t major, minor, patch;   // = protocol Spec version, e.g. 1.25.1
    uint32_t gitHash;               // build commit, first 8 hex digits (0 when noInfo)
    bool     dirty;                 // built with uncommitted changes
    bool     noInfo;                // built without git info
    uint16_t buildYymm, buildDdhh;  // 2610 = 2026-10, 2714 = day 27, 14 h (build PC local time)
    bool        hasVersion() const; // false on firmware <= v1.24
    std::string version() const;    // "1.25.1"
    std::string toString() const;   // "1.25.1 (1a2b3c4d)", "1.25.1 (1a2b3c4d-dirty)", "<= 1.24 (no version registers)"
    std::string buildTime() const;  // "2026-10-27 14h", "" when not recorded
};
```


```cpp
enum class TactileLink : uint8_t {
    NotDetected, Disconnected, RawStream, Waiting, Live
};

struct TactileTip {                 // one fingertip, gripper frame (fw >= v1.18.0)
    bool     valid, saturated, timeout, connected,
             rawStream, contact, seen, rawFresh;   // TACT_STATUS bits
    float    posX;     // mm, 0..32 from the wrist-side edge of the cell area toward the fingertip; 0 = no contact point
    float    posY;     // mm, +-12.5 across the pad width
    float    fx, fy;   // N, shear along +posX / +posY (after tare)
    float    fz;       // N, normal force, > 0 = pressing (after tare)
    uint16_t rateHz;   // result frames in the last second (~100); readTactile() only
    uint16_t rxCount;  // result frames since boot (wraps); readTactile() only
    uint16_t status;   // raw TACT_STATUS word
    TactileLink link() const;
    bool        hasContactPoint() const;   // valid && posX > 0
};
using TactilePair = std::array<TactileTip, 2>;   // [0] right (tip 1), [1] left (tip 2)

struct TactileRaw {                 // raw diagnostic stream of one tip
    uint16_t sets, diag;
    std::array<int16_t, 69> channels;   // C0..C68
    std::array<int32_t, 23> groups;     // G1..G23 = sum of 3 channels
};

struct FastState {                  // 0x02C0..0x02C9 snapshot
    bool         activated, homed, goToActive;
    GripperStage stage;
    ObjectState  object;
    uint8_t      position, current, fault;      // gPO, gCU, gFLT
    TactileTip   right, left;                   // 8-bit: ~0.1 mm / ~0.27 N; rateHz = rxCount = 0
};
```

`TactileLink` maps the status bits to what to show a user:
`NotDetected` (never seen since boot — not fitted, wiring, or sensor type
≠ 2) · `Disconnected` (was seen, frames stopped) · `RawStream` (sensor
firmware sends raw channels instead of results) · `Waiting` (results
arrive but are invalid — sensor zeroing, ≈ 4.5 s after power-up or
≈ 1.25 s after `tactileZero()`) · `Live`. `toString(TactileLink)` and
`operator<<` for `TactileTip` / `FastState` / `FirmwareInfo` are provided.

---

## 7. Protocol reference (Modbus registers)

The firmware exposes a **DH-Robotics RTU compatible** holding-register
map. The constants in `aidin::reg::*` mirror these addresses 1:1. The full
reference is the Aidin Gripper Modbus RTU Protocol Manual Rev 2.6 (Spec
v1.25.1), provided with the product.

Transport (firmware ≥ v1.21): FC03 / FC06 / FC16 / FC23, anything else
gets exception 01. A request ends after max(3.5 characters, 750 µs) of
line silence and the reply starts ≈ 7 µs later — ≈ 0.76 ms above
19200 bps, no jitter (plus `CFG_MB_RESP_DELAY`). Flash writes stop the
firmware for ≈ 25 ms; requests sent meanwhile get no reply.

### 7.1 Command registers (master → slave, function 0x06 / 0x10)

| PDU addr | Field  | Range | Description |
|----------|--------|-------|-------------|
| `0x0100` | rACT   | bit 0 | 0=deactivate, 1=activate. The other bits are processed only in writes with rACT=1 while active; cleared by the firmware on a fault (v1.23) |
| `0x0100` | rHOM   | bit 1 | rising edge starts homing |
| `0x0100` | rGTO   | bit 3 | 0=stop, 1=go to `rPR` (runs on a 0→1 edge or an `rPR` change) |
| `0x0100` | rATR   | bit 4 | rising edge: emergency auto-release, written together with rACT |
| `0x0100` | rADR   | bit 5 | 0=release open, 1=release close |
| `0x0101` | rSP    | 0..255 | speed: velocity/acceleration limit = trajectory limit × rSP/255 (boot default 128) |
| `0x0102` | rFR    | 0..255 | force: current limit = rated current × rFR/255, 0 = no force (boot default 255) |
| `0x0103` | rPR    | 0..255 | target position (0=open, 255=close) |

`rSP`/`rFR` written while inactive apply at activation (v1.25.1).

### 7.2 Status registers (slave → master, function 0x03)

| PDU addr | Field  | Description |
|----------|--------|-------------|
| `0x0200` | gACT   | bit 0 — activation echo |
| `0x0200` | gHOM   | bit 1 — homing complete |
| `0x0200` | gGTO   | bit 3 — go-to echo |
| `0x0200` | gSTA   | bits 5:4 — 0=reset, 1=activating, 3=active |
| `0x0200` | gOBJ   | bits 7:6 — 0=moving, 3=move finished; 1/2 (contact) reserved, not reported |
| `0x0201` | gFLT   | current fault code |
| `0x0202` | gPR    | echo of last `rPR` |
| `0x0203` | gPO    | actual position (0..255) |
| `0x0204` | gCU    | actual current (0..255) |
| `0x0205` | gSP    | actual speed (0..255) |
| `0x0206` | gV     | supply voltage in 0.1 V units (e.g. 240 = 24.0 V) |
| `0x0207` | gFLTO  | fault history (kept until `DEV_CMD 0x07 FAULT_CLEAR`) |

### 7.3 Configuration registers (factory / installer)

| PDU addr | Field            | Description |
|----------|------------------|-------------|
| `0x0182` | `CFG_MB_ADDR`    | Slave address (1..247) |
| `0x0183` | `CFG_MB_BAUD_LO` | Baud rate (low 16 bits) |
| `0x0184` | `CFG_MB_BAUD_HI` | Baud rate (high 16 bits) |
| `0x0185` | `CFG_CAN_ID`     | FDCAN node ID |
| `0x0186` | `CFG_CAN_RATE`   | 1 = 1 Mbps, 2 = 500 kbps |
| `0x0187` | `CFG_DEV_TYPE`   | 101 = Aidin gripper, 102 = Apicoo, 103 = AFT200, 104 = AFT150 |
| `0x0188` | `CFG_SERIAL_LO`  | Serial number: production number NNNN, 1..9999 (v1.24.0, §7.7) |
| `0x0189` | `CFG_SERIAL_HI`  | Serial number: production YYMM as a decimal value, 2610 = 2026-10 (v1.24.0) |
| `0x018A` | `CFG_SENSOR_TYPE` | Fingertip sensor: 0 = none, 1 = AFT F/T, 2 = tactile — applied immediately on `SAVE_CONFIG` / `LOAD_CONFIG` |
| `0x018B` | `CFG_STROKE`     | Measured stroke [rad × 1000], read-only — set by `STROKE_CAL` / `fullInit()` |
| `0x018C` | `CFG_MB_RESP_DELAY` | Extra Modbus response delay [µs], 0..65535 (default 0) — applied on write (v1.22.0) |

Config writes land in RAM. Persist them with
`writeRegister(reg::DEV_CMD, reg::DEV_CMD_SAVE_CONFIG)`
(`DEV_CMD_LOAD_CONFIG` reloads the flash values); the slave address and
baud apply at the next boot (`DEV_CMD_REBOOT`, §4). The `DEV_CMD_*`
constants this SDK uses are in `registers.hpp`; the full command list is
in the Protocol Manual §3.

### 7.4 PID gain registers

| PDU addr | Field            |
|----------|------------------|
| `0x0190` | Position P (int16) |
| `0x0191` | Position I (int16) |
| `0x0192` | Position D (int16) |
| `0x0193` | Speed P    (int16) |
| `0x0194` | Speed I    (int16) |
| `0x0195` | Current P  (int16) |
| `0x0196` | Current I  (int16) |

### 7.5 Tactile sensor registers

Active when `CFG_SENSOR_TYPE` (`0x018A`) = 2 — write it, then `DEV_CMD
0x02 SAVE_CONFIG` (applies immediately, no reboot). Firmware protocol
≥ v1.13.0; the gripper-frame coordinates below need ≥ v1.18.0. Tip 1 =
right finger, tip 2 = left.

| Tip 1 | Tip 2 | Field | Description |
|-------|-------|-------|-------------|
| `0x0220` | `0x0228` | `TACT_STATUS` | b8 valid · b9 saturated · b10 timeout · b11 connected · b12 raw_stream · b13 contact (Fz ≥ 0.5 N) · b14 seen · b15 raw_fresh; low byte = 2 |
| `0x0221` | `0x0229` | `POSX` | int16 [mm × 100], 0..3200 from the wrist-side edge of the cell area toward the fingertip; 0 = no contact point |
| `0x0222` | `0x022A` | `POSY` | int16 [mm × 100], ±1250 across the pad width |
| `0x0223` | `0x022B` | `FX` | int16 [N × 100] shear along +X, after tare |
| `0x0224` | `0x022C` | `FY` | int16 [N × 100] shear along +Y, after tare |
| `0x0225` | `0x022D` | `FZ` | int16 [N × 100] normal, positive = pressing, after tare |
| `0x0226` | `0x022E` | `RATE` | result frames in the last second [Hz] (≈ 100) |
| `0x0227` | `0x022F` | `RXCNT` | result frames since boot (wraps) |

Values are 0 while `valid` = 0. Related registers:

| PDU addr | Field | Description |
|----------|-------|-------------|
| `0x0230~0x0276` / `0x0277~0x02BD` | `TACTx_RAW` | raw 69-channel stream per tip: `[sets][diag][C0..C68]` |
| `0x01B0~0x01BA` | `TACT_CMD_*` | CAN passthrough: tip, len (1..8), data[8], CAN ID (0 = default `0x324`) → `DEV_CMD 0x09` |
| `0x01BB~0x01BD` | `TACT_SCMD_*` | structured command: code (`0x01` ZERO, `0x05` FACTORY_RESET with arg `0xA5A5`), tip bitmask, arg → `DEV_CMD 0x0F` |

### 7.6 Fast-poll block (`0x02C0..0x02C9`)

Read-only copy of the essentials for **one** FC03 read (protocol ≥ v1.15.0).
The firmware rewrites all 10 words at once, so a reply never mixes update
cycles.

| PDU addr | High byte | Low byte |
|----------|-----------|----------|
| `0x02C0` | 0 | `STATUS` (same bits as `0x0200`) |
| `0x02C1` | 0 | `gPO` |
| `0x02C2` | 0 | `gCU` |
| `0x02C3` / `0x02C6` | right / left `Px8` (uint8) | `Py8` (int8) |
| `0x02C4` / `0x02C7` | `Fx8` (int8) | `Fy8` (int8) |
| `0x02C5` / `0x02C8` | `Fz8` (uint8) | `status8` = `TACT_STATUS >> 8` |
| `0x02C9` | 0 | `gFLT` |

`status8` bits: b0 valid · b1 saturated · b2 timeout · b3 connected ·
b4 raw_stream · b5 contact · b6 seen · b7 raw_fresh (the `TACT_STATUS`
bits of §7.5, shifted down by 8). 8-bit tactile values: each field's full
scale maps to the end of its byte range — `Px8` 0..255 ↔ 0..32 mm, `Py8`
±127 ↔ ±12.5 mm, `Fx8`/`Fy8` ±127 ↔ ±35 N (provisional), `Fz8` 0..255 ↔
0..70 N (protocol ≥ v1.19.0); out-of-range values clamp and set
`saturated`. `Px8` is uint8 since v1.18.0 (int8 before). With
`CFG_SENSOR_TYPE` ≠ 2 the six tactile words are 0. Packing rules, a
decoded wire example, PLC addressing and polling timing per baud rate:
[root README](../README.md#fast-poll-block--고속-폴링-블록-0x02c00x02c9).

### 7.7 Serial number and firmware info (`0x01C0..0x01CB`)

| PDU addr | Field | Description |
|----------|-------|-------------|
| `0x01C0~0x01C3` | `CFG_PRODUCT_CODE` | R/W. Serial-number product code, ASCII ≤ 8 chars, 2 per register, high byte first (`0x01C0` = `'B','L'` = `0x424C`), unused bytes 0 (v1.24.0) |
| `0x0189` / `0x0188` | `CFG_SERIAL_HI` / `_LO` | R/W. Production YYMM (decimal) / production number NNNN (§7.3) |
| `0x01C4~0x01C6` | `FW_VER_MAJOR` / `MINOR` / `PATCH` | R. Firmware version = protocol Spec version (v1.25.0) |
| `0x01C7` / `0x01C8` | `FW_GIT_HASH_HI` / `LO` | R. Build commit, first 8 hex digits, 4 per register |
| `0x01C9` | `FW_BUILD_FLAGS` | R. b0 `FW_BUILD_DIRTY` (uncommitted changes) · b1 `FW_BUILD_NO_INFO` (no git info, hash 0) |
| `0x01CA` / `0x01CB` | `FW_BUILD_YYMM` / `DDHH` | R. Build time, decimal: 2610 = 2026-10, 2714 = day 27, 14 h (build PC local time) |

Serial number = `<product code>-<YYMM>-<NNNN>`, e.g. `BLDC2FV1-2610-0001`;
all words 0 = not recorded. Write the three parts, then `SAVE_CONFIG`.
Firmware ≤ v1.23 has no product code (reads 0, not saved) and firmware
≤ v1.24 reads 0 in `0x01C4~0x01CB` (version 0.0.0). `readSerialNumber()`
and `readFirmwareInfo()` (§6.1) do the decoding.

---

## 8. Error handling

All methods that may fail throw `aidin::GripperError` (derived from
`std::runtime_error`). Failure modes:

| Where                           | Typical message                          |
|---------------------------------|------------------------------------------|
| `connect()`                     | `connect failed: open /dev/ttyUSB0: No such file or directory` |
| `connect()`                     | `connect failed: unsupported baud rate: ...` |
| any read/write                  | `read 0x0200 failed: response timeout`   |
| any read/write                  | `write 0x0100 failed: crc error` / `modbus exception code N` |
| `activate()`                    | `activate failed: gACT fell back to 0 (gACT=0, gFLT=6 (under-voltage), gFLTO=6 (under-voltage)) - remove the cause and call activate() again ...` |
| `home()` / `fullInit()` / blocking `moveTo()` | `home stopped: gripper not active or faulted (gACT=0, gFLT=..., gFLTO=...) - remove the cause, call activate(), then re-send the command` |
| blocking `moveTo()`             | `moveTo not executed (gGTO=0): ...` / `moveTo not executed: gripper not homed (gHOM=0) ...` |
| `fullInit()`                    | `full-init rejected or failed (DEV_STATUS=ERROR)` |
| `activate()` / `home()` / `fullInit()` / blocking `moveTo()` | `... timeout` |

The exception message embeds the OS error (`strerror` / Win32 error code),
the transport-level failure (timeout, CRC, Modbus exception code), or the
gripper state that stopped a blocking call (`gACT`, `gFLT` = live fault,
`gFLTO` = fault history), so it tells you precisely which layer failed.

**Fault recovery** (firmware ≥ v1.23): a fault switches the gripper off and
clears `rACT`. Remove the cause, call `activate()`, then re-send the
command — the first re-activating write does not execute
`rGTO`/`rHOM`/`rATR`, so the command that hit the fault (or any command
sent right after it) has to be sent again. `FAULT_CLEAR` is not needed;
it only clears the `gFLTO` history.

```cpp
try {
    g.moveTo(255, 200, 128, true);
} catch (const aidin::GripperError& e) {
    std::cerr << e.what() << "\n";     // names gFLT / gFLTO
    // remove the cause (supply, load, temperature), then:
    g.activate();                      // throws again if the fault is still present
    g.moveTo(255, 200, 128, true);     // re-send
}
```

Retry plain transport errors (timeouts, CRC) after a short pause — for
example a request that arrived during a ≈ 25 ms flash write gets no
reply. Do not blindly retry a "stopped" or "not executed" error: fix the
cause first.

---

## 9. Threading model

- **Transactions are serialized per port.** Every Modbus transaction (one
  register read or write) holds the mutex of the shared bus object for
  that port, so `Gripper` instances on the same port (multi-drop) never
  interleave bytes on the wire. The lock covers one transaction only — a
  15 s `home()` on one gripper does not block another gripper's polling.
- **One instance, one thread.** A single `Gripper` is not safe to drive
  from several threads at once: its commands are multi-step (`moveTo()`
  is up to five writes plus a sleep), and it keeps per-instance state
  (last `ACTION` / `rPR` written, for the `rGTO` pulse) without a lock.
  `connect()` / `disconnect()` must not race with calls on the same
  instance either. Use one instance per thread, or wrap your own mutex
  around every call (the ROS2 driver does this).
- **Multiple instances in parallel are fine** — on different ports, or on
  the same port with different slave IDs. The only global state is the
  process-wide port registry (mutex-protected) that lets instances share
  an open port; the port closes when the last instance using it
  disconnects.
- `setResponseTimeout()` changes the deadline for every instance on that
  port (it is a property of the shared bus).
- The SDK never spawns background threads.

---

## 10. Examples walkthrough

### 10.1 `01_activate_and_home.cpp`

The minimum viable sequence:
```cpp
g.connect(port);
g.activate();          // blocks until gSTA == Active
g.home();              // blocks until gHOM == 1
```
Useful for verifying wiring on a new bench.

### 10.2 `02_open_close_loop.cpp`

Demonstrates the blocking variant of `moveTo()` and SIGINT-safe
shutdown:
```cpp
std::signal(SIGINT, onSignal);
for (int i = 0; i < cycles && !g_stop; ++i) {
    g.moveTo(255, 200, 128, true);   // close
    g.moveTo(0,   200, 128, true);   // open
}
```
A clean `disconnect()` runs via the `Gripper` destructor.

### 10.3 `03_status_monitor.cpp`

Prints the firmware version and serial number (`readFirmwareInfo()`,
`readSerialNumber()`), then polls at 100 ms intervals and prints only when
something changes — useful as a live readout during commissioning.

### 10.4 `04_tactile_monitor.cpp`

Checks `CFG_SENSOR_TYPE`, waits until both tips are `Live` (or absent),
optionally zeroes them (`--zero`), then streams both tips at 20 Hz:
```cpp
const auto tips = g.readTactile();          // {right, left}
if (tips[0].link() == aidin::TactileLink::Live) {
    std::cout << tips[0].posX << " mm, Fz=" << tips[0].fz << " N\n";
}
g.tactileZero();                            // pads untouched
```

### 10.5 `05_fast_poll.cpp`

Calls `readFastState()` back-to-back and prints the achieved rate twice a
second — a quick way to see what the bus allows at your baud rate.

---

## 11. Troubleshooting

| Symptom                                                | Likely cause                                                      | Fix |
|--------------------------------------------------------|-------------------------------------------------------------------|-----|
| `connect failed: open ...: No such file or directory`  | Wrong port name or adapter not plugged in                          | Check `dmesg` (Linux) or Device Manager (Windows) |
| `connect failed: open ...: Permission denied`          | Linux: user not in `dialout`                                       | `sudo usermod -a -G dialout $USER`, re-login |
| All reads time out                                     | Wrong baud or slave ID; or RS485 A/B swapped                       | Swap A↔B, check the baud and `slaveId` (parity is fixed at `'N'`) |
| Reads succeed but values stay zero                     | Adapter on different RS485 segment / no termination                | Add 120 Ω termination at both ends |
| `activate failed: gACT fell back to 0 (...)`           | A fault is still present (or the motor did not start)              | The message names `gFLT` (live fault, `0x0201`): remove that cause (supply, load, temperature), then `activate()` again. `gFLTO` is only the history |
| `... stopped: gripper not active or faulted (...)`     | A fault switched the gripper off, or it was deactivated            | Remove the cause, `activate()`, re-send the command |
| `moveTo not executed (gGTO=0)`                         | Not homed, force control active, or the write re-activated the gripper after a fault | `home()`, set `FC_MODE` = 0, or re-send the move |
| `home timeout`                                         | Mechanical jam, backoff distance exceeds workspace, or a very low `rSP` | Manually back the fingers off and retry |
| `moveTo` to the current position returns at once       | The fingers are already there (`AtTarget` right away)              | Expected |
| Tactile tip stays `NotDetected`                        | `CFG_SENSOR_TYPE` ≠ 2, or sensor cable                             | Write `0x018A = 2`, then `DEV_CMD 0x02` (no reboot needed) |
| Tactile tip stays `Waiting`                            | Sensor still zeroing after power-up (≈ 4.5 s) or data stream broken | Wait; if it persists, check the sensor |
| `tactile command rejected`                             | Not tactile, invalid values, or force control uses the sensor      | Stop force control (`FC_MODE = 0`) before zeroing |
| `readFastState()` → `modbus exception code 2`          | Firmware older than protocol v1.15.0                               | Use `readState()` + `readTactile()` |

### Capturing a wire trace

For deeper protocol debugging, sniff the RS485 bus with a logic analyzer
(Saleae / DSLogic) at 115200 baud, then decode as Modbus RTU. A correct
exchange looks like:

```
TX: 01 03 02 00 00 08 4...    (master: read 8 regs from 0x0200)
RX: 01 03 10 33 00 00 00 ...  (slave: 16 bytes of data + CRC)
```

---

## 12. FAQ

**Q. Can I use the SDK from C instead of C++?**
A. Not directly — the public API is C++17. For C consumers, wrap the
class in your own `extern "C"` shim.

**Q. Does the SDK support Modbus TCP?**
A. No — the firmware itself is RTU-only and the in-tree transport
implements RTU framing over a serial port. If you need TCP, bridge
RTU↔TCP externally.

**Q. Why an in-tree Modbus implementation and not libmodbus?**
A. The SDK talks to exactly one register map with three function codes
(0x03/0x06/0x10) — the whole master fits in ~400 lines
(`src/modbus_rtu.cpp`). Owning it removes the external dependency
(FetchContent network access, LGPL linkage) and gives us direct control
over response deadlines, stale-RX flushing, and inter-frame timing,
which matters for high-rate polling from the ROS2 driver.

**Q. Can I tune PID gains at runtime?**
A. Yes — write `0x0190..0x0196`, then `DEV_CMD 0x04 SAVE_PID` (applies
and saves to flash; `0x05` LOAD_PID and `0x06` RESTORE_PID_DEFAULT undo
it). See §7.4 and the Protocol Manual §5.

**Q. How do I detect that an object has been grasped (not just that the
target was reached)?**
A. After a closing `moveTo()`, check `state.object`. `ClosedContact`
means an object stopped the fingers before reaching the target;
`AtTarget` means the fingers reached the commanded position with no
obstruction.

**Q. What's the maximum command rate?**
A. Each Modbus transaction at 115 200 baud takes 3–5 ms wall time,
mostly line time and USB-adapter latency (the firmware itself answers
≈ 0.76 ms after a request). A typical control loop of `moveTo` +
`readState` therefore runs at ≈ 100 Hz. For tighter control loops,
raise the baud rate (§4: `0x0183`/`0x0184`, `SAVE_CONFIG`, `REBOOT`) and
poll the fast block (`readFastState()`).

---

## 13. Extending the SDK

The PImpl boundary makes it easy to add wrappers without touching
public headers. For example, to add a "wait until current exceeds X"
helper:

```cpp
// in src/gripper.cpp
void Gripper::waitUntilCurrent(uint8_t threshold,
                               std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (readState().current >= threshold) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw GripperError("waitUntilCurrent timeout");
}
```
Add the declaration in `include/aidin/gripper.hpp`. No CMake changes
needed.

To add a new register group, extend `include/aidin/registers.hpp` with
new `constexpr` constants — keep them in sync with the Protocol Manual and
with `python/aidin_gripper/registers.py`.

---

*Manual revision: 1.2 — matches SDK [`v1.2.0`](CMakeLists.txt) and firmware Spec v1.25.1.*
