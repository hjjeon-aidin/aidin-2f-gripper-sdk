# AIDIN 2F Gripper — Host SDKs & ROS2 Driver

Host-side software for the **AIDIN 2-finger BLDC gripper** (DH-Robotics
compatible Modbus RTU, 115200 8N1, slave ID 1). Everything here talks to the
gripper over a USB↔RS485 adapter — no vendor DLLs, no external Modbus
library, no network access at build time.

AIDIN 2지 BLDC 그리퍼용 호스트 소프트웨어 모음입니다. USB-RS485 어댑터만
있으면 동작하며, 외부 Modbus 라이브러리 의존성이 없습니다.

## What's inside / 구성

| Directory | What it is | Docs |
|---|---|---|
| [`cpp/`](cpp/) | `aidin::Gripper` C++17 SDK — self-contained Modbus RTU framing (termios / Win32). Builds on Ubuntu & Windows with one CMake command | [README](cpp/README.md) · [MANUAL](cpp/MANUAL.md) |
| [`python/`](python/) | Pure-Python SDK (`pyserial`) — same API surface as the C++ SDK | [README](python/README.md) |
| [`ros2/`](ros2/) | ROS2 packages: `aidin_gripper_driver` (rclcpp node), `aidin_gripper_msgs`, `aidin_gripper_examples`. State topic @ 50 Hz + optional tactile topic + services (`activate`/`home`/`move_to`/`emergency_release`/`tactile_zero`/…) | [README](ros2/README.md) |
| [`urdf/`](urdf/) | `aidin_gripper_description` ROS2 package — URDF + meshes for the gripper end-effector | [ros2/README.md](ros2/README.md#workspace-layout) |

## Gripper operating procedure / 그리퍼 실행 절차

`cpp/`, `python/`, `ros2/` all drive the same firmware state machine in the
same order, regardless of which interface you pick.

세 인터페이스(`cpp/`, `python/`, `ros2/`) 모두 동일한 순서로 동일한 펌웨어
상태 머신을 구동합니다. 사용하는 인터페이스와 무관하게 아래 순서를
따르세요.

| # | Step / 단계 | Python | ROS2 | Protocol |
|---|---|---|---|---|
| 1 | Wire & power / 배선·전원 | USB↔RS485 adapter → gripper UART1, 115200 8N1 | 동일 | — |
| 2 | Connect / 연결 | `g.connect("/dev/ttyUSB0")` | `port:=` launch arg | opens the serial link |
| 3 | Activate / 활성화 | `g.activate()` | `ros2 service call .../activate std_srvs/srv/Trigger` | `rACT=1`, wait `gSTA=Active` |
| 4 | Home / 호밍 | `g.home()` | `.../home` service (or `auto_home:=true`) | `rHOM` rising edge, wait `gHOM=1` — **keep hands clear, fingers seek a hard stop / 손을 가까이 두지 마세요** |
| 5 | Move (open/close) / 이동 | `g.move_to(pos, speed, force, blocking=True)` | `.../move_to` (`MoveTo` service) | `rPR`/`rSP`/`rFR` + `rGTO` |
| 6 | Monitor status / 상태 모니터링 | `g.read_state()` | subscribe `/aidin_gripper_driver/state` | `gSTA`/`gOBJ`/`gPO`/`gFLT` |
| (opt) | Tactile sensors / 촉각 센서 | `g.read_tactile()` | `publish_tactile:=true` → `/aidin_gripper_driver/tactile` | `0x0220~` — see [Tactile sensor](#tactile-sensor--촉각-센서-0x0220) |
| (opt) | Fast poll / 고속 폴링 | `g.read_fast_state()` | — (SDK only) | `0x02C0~` — see [Fast-poll block](#fast-poll-block--고속-폴링-블록-0x02c00x02c9) |
| — | Recover from a fault/jam / 고장·끼임 복구 | `g.emergency_release(...)` then `g.activate()` again | `.../emergency_release` then `.../activate` | see each README's Troubleshooting table |

Minimal Python run-through (full script with progress logging:
[`python/examples/06_full_quickstart_demo.py`](python/examples/06_full_quickstart_demo.py)):

```python
from aidin_gripper import Gripper

with Gripper() as g:
    g.connect("/dev/ttyUSB0")     # 1-2. wire + connect
    g.activate()                  # 3. activate
    g.home()                      # 4. home (hands clear!)
    g.move_to(255, speed=200, force=128, blocking=True)   # 5. close
    g.move_to(0,   speed=200, force=128, blocking=True)   # 5. open
    print(g.read_state())         # 6. status
```

Full per-SDK instructions: [python/README.md](python/README.md) ·
[cpp/README.md](cpp/README.md) · [ros2/README.md](ros2/README.md).

## Quick start (Python)

```bash
git clone <this-repo> aidin-2f-gripper-sdk
cd aidin-2f-gripper-sdk/python

python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\Activate.ps1
pip install -e .

cd examples
python 06_full_quickstart_demo.py /dev/ttyUSB0   # Windows: COM3
```

More examples (activate/home, single open+close, cyclic loop, status,
fault + emergency release): [python/README.md](python/README.md#run-the-examples).

## Quick start (ROS2)

```bash
git clone <this-repo> aidin-2f-gripper-sdk
cd aidin-2f-gripper-sdk

mkdir -p ~/ros2_ws/src
ln -s $(pwd)/ros2/aidin_gripper_msgs     ~/ros2_ws/src/
ln -s $(pwd)/ros2/aidin_gripper_driver   ~/ros2_ws/src/
ln -s $(pwd)/ros2/aidin_gripper_examples ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch aidin_gripper_driver aidin_gripper.launch.py port:=/dev/ttyUSB0
```

Full instructions (parameters, services, troubleshooting): [ros2/README.md](ros2/README.md)

> The `aidin_gripper_driver` build pulls in [`cpp/`](cpp/) via
> `add_subdirectory` — keep the repo layout intact (or pass
> `-DAIDIN_SDK_DIR=/abs/path/to/cpp`).

## Safety / 주의

The gripper applies real torque to mechanical fingers. Keep hands clear
during homing (fingers seek a hard stop), and clear the workspace before
launching with `auto_home:=true`.

그리퍼는 실제 토크로 핑거를 구동합니다. 호밍 중에는 손을 가까이 두지 말고,
`auto_home:=true` 로 실행하기 전에 작업 공간을 비워 주세요.

## Reference for AI agents / AI 참고 자료

This section is a single, self-contained technical reference for an AI
assistant integrating this gripper into a *different* codebase (a new
language, a different framework) that doesn't want to explore this whole
repo. It restates the protocol precisely enough to write a correct client
from scratch, then points at the three reference implementations that
already exist here.

### What this device is

- AIDIN 2-finger BLDC gripper, DH-Robotics-compatible **Modbus RTU** slave.
- Transport: RS485 over a USB adapter. Default link: **115200 baud, 8 data
  bits, no parity, 1 stop bit (8N1)**, slave address **1** (changeable,
  1..247).
- Function codes the device accepts: **0x03** (Read Holding Registers),
  **0x06** (Write Single Register), **0x10** (Write Multiple Registers).
  The SDKs in this repo use 0x03/0x06, plus 0x10 for the tactile
  structured command. Every register below is a 16-bit holding register
  at its protocol-level (PDU) address.
- Detailed per-register documentation — bit maps, value scaling, usage
  sequences, Modbus frame examples — is in the **AIDIN Gripper Modbus RTU
  Protocol Manual** (`aidin_modbus_rtu_manual.pdf` / `.html`, Manual Rev 2.3,
  in the firmware repo `aidin-bldc-firmware/docs/`). The canonical register
  contract is `docs/protocol/protocol.md` in the same repo (spec v1.19.0).
  Neither is included here — the registers mirrored in
  [`cpp/include/aidin/registers.hpp`](cpp/include/aidin/registers.hpp) /
  [`python/aidin_gripper/registers.py`](python/aidin_gripper/registers.py)
  (motion, status, tactile sensor, fast-poll block) are exercised by this
  repo's SDKs; the other extended blocks below (config, force control,
  F/T sensor) are summarized from the manual.

### Register map / 레지스터 맵

> **📖 상세 설명은 매뉴얼을 참고하세요.** 아래 모든 레지스터의 비트맵, 값
> 스케일링(정규화 공식), 사용 시퀀스, Modbus 프레임 예제는 **AIDIN Gripper
> Modbus RTU Protocol Manual** (`aidin_modbus_rtu_manual.pdf` / `.html`)에
> 있습니다. 이 README는 요약본입니다.
> **For full details see the manual** — this README only summarizes each
> register in one line.

#### Motion control / 동작 제어 (Write, `0x0100~`)

| Addr | R/W | Field | Meaning |
|---|---|---|---|
| `0x0100` | W | `ACTION` | activation / homing / go-to / emergency-release bitfield — see below |
| `0x0101` | W | `SPEED` (`rSP`) | 0..255 (255 = 100% of `MAX_VELOCITY`; 0 still keeps an internal minimum) |
| `0x0102` | W | `FORCE` (`rFR`) | 0..255 current/force limit (0 = 25% hardware floor, 255 = 100%) |
| `0x0103` | W | `POSITION` (`rPR`) | 0..255 — **0 = fully open, 255 = fully closed** |

#### Status & telemetry / 상태·텔레메트리 (Read, `0x0200~`)

| Addr | R/W | Field | Meaning |
|---|---|---|---|
| `0x0200` | R | `STATUS` | bitfield — see below |
| `0x0201` | R | `FAULT` (`gFLT`) | current fault code, 0 = OK — see fault code table below |
| `0x0202` | R | `POS_ECHO` (`gPR`) | echo of the last `rPR` written |
| `0x0203` | R | `POS_ACTUAL` (`gPO`) | actual position 0..255 (reads 0 until homed) |
| `0x0204` | R | `CUR_ACTUAL` (`gCU`) | actual motor current 0..255 (normalized to `NOMINAL_CURRENT`) |
| `0x0205` | R | `SPD_ACTUAL` (`gSP`) | actual speed 0..255 (normalized to `MAX_VELOCITY`) |
| `0x0206` | R | `VOLTAGE` (`gV`) | supply voltage — register × 0.1 = volts |
| `0x0207` | R | `FAULT_LATCH` (`gFLTO`) | latched fault history — persists until cleared with `DEV_CMD 0x07 FAULT_CLEAR` |

`STATUS`..`FAULT_LATCH` (`0x0200`..`0x0207`, 8 registers) are meant to be
read **in one block transaction**, not one register at a time — that's
what every SDK's `read_state()` / `readState()` does.


### `ACTION` bits (write, `0x0100`)

| Bit | Name | Type | Meaning |
|---|---|---|---|
| 0 | `rACT` | level | 1 = request activation. The firmware ignores nearly every other command while this is 0. |
| 1 | `rHOM` | **rising edge** | trigger homing. Must go 0→1; holding it at 1 does nothing again. |
| 3 | `rGTO` | level, edge-sensitive re-trigger | 1 = go to `rPR`. Re-commanding the *same* position while `rGTO` is already 1 is ignored — pulse it 0→1 to force re-execution (firmware ≥ v1.9.1). |
| 4 | `rATR` | **rising edge** | emergency auto-release. Only processed while `rACT=1`. |
| 5 | `rADR` | level | auto-release direction: 0 = open, 1 = close. Set before/with the `rATR` pulse. |

Writing the exact **value `0xA5`** to `ACTION` is special (protocol
v1.12.0, DH-Robotics "full initialization" compatible): it triggers a
stroke calibration — activate + open homing + close-contact stroke
measurement, flash-saved and readable at `0x018B`. It works from the
inactive state with a single write (the value includes the `rACT` bit) and
is edge-detected on the written value. Every SDK here wraps it:
`full_init()` / `fullInit()` / the `/full_init` service.

Because `ACTION` is one register, every write replaces *all* of these bits
at once — there is no way to set one bit without knowing (or explicitly
re-asserting) the others. This is also why `emergency_release()` writing
`rATR` (without `rACT`) leaves the device with `rACT=0`: the *next*
`activate()` call is what clears the release latch and restores normal
`move_to()`/`moveTo()` behavior.

### `STATUS` bits (read, `0x0200`)

| Bits | Name | Meaning |
|---|---|---|
| 0 | `gACT` | activated — drops to 0 when a fault disables the motor (firmware ≥ v1.19.1, [below](#fault-deactivation--폴트-시-비활성화)) |
| 1 | `gHOM` | homed |
| 3 | `gGTO` | echo of `rGTO` |
| 4-5 | `gSTA` | `00`=Reset, `01`=Activating, `11`=Active |
| 6-7 | `gOBJ` | `00`=Moving, `01`=stopped on object while opening, `10`=stopped on object while closing, `11`=at target |

### Fault codes / 폴트 코드 (`gFLT` / `gFLTO`)

`gFLT` (0x0201) is the live fault, `gFLTO` (0x0207) is the latched
history — check it for faults that came and went, then clear with
`DEV_CMD 0x07 FAULT_CLEAR`. 원인·조치 상세는 매뉴얼 참고.

| Code | Name | Meaning / action |
|---|---|---|
| 0 | `OK` | normal operation |
| 1 | `NOT_INITIALIZED` | not activated — send `rACT = 1` |
| 2 | `INVALID_COMMAND` | unsupported command received |
| 3 | `INVALID_PARAMETER` | parameter out of range |
| 4 | `OVER_CURRENT` | over-current — check the load, power-cycle |
| 5 | `OVER_TEMPERATURE` | over-temperature — cool down, restart |
| 6 | `UNDER_VOLTAGE` | supply voltage too low — check the power supply |
| 7 | `MOTOR_STALL` | motor stall — check for a mechanical jam |
| 8 | `SENSOR_ERROR` | encoder signal error — check wiring |
| 9 | `COMM_TIMEOUT` | communication timeout — check cable / baud rate |
| 10 | `INTERNAL_ERROR` | internal error — power-cycle, contact A/S if persistent |
| 11 | `GATE_DRIVER_FAULT` | gate-driver hardware fault (OCP/OTSD/UVLO) — check supply/wiring, then `FAULT_CLEAR` |

#### Fault deactivation / 폴트 시 비활성화

Firmware protocol ≥ v1.19.1. If a fault disables the motor while the
gripper is active, the firmware stops the motor and reports **`gACT` = 0,
`gSTA` = 0 (Reset), `gGTO` = 0**, the same in `0x0200` and in the fast-poll
block `0x02C0`. That covers motor-controller faults — over-current,
over-temperature, under-voltage, encoder, internal (codes 4, 5, 6, 8,
10) — and gate-driver faults (11).

폴트로 모터 enable 이 풀리면 `gACT` 가 0 이 됩니다. 자동으로 다시 켜지지 않습니다.

- **No automatic restart.** `rACT` stays 1 in `ACTION`, and motion
  commands (move, home, force control, emergency release) are ignored
  until the gripper is re-activated.
- **Recovery**: remove the cause → `DEV_CMD 0x07 FAULT_CLEAR` → write
  `rACT` = 0, then `rACT` = 1.

  | | Python | C++ | ROS2 |
  |---|---|---|---|
  | Fault clear | `g.write_register(reg.DEV_CMD, reg.DEV_CMD_FAULT_CLEAR)` | `g.writeRegister(reg::DEV_CMD, reg::DEV_CMD_FAULT_CLEAR)` | `/aidin_gripper_driver/fault_clear` |
  | Re-activate | `g.deactivate()` then `g.activate()` | `g.deactivate()` then `g.activate()` | — |

  `activate()` alone writes `rACT` = 1 again, which is no change while
  `rACT` is still 1, so call `deactivate()` first. If the fault is still
  present, `gACT` drops back to 0 right after re-activation.
- `gFLT` is the live fault: it can return to 0 once, for example, the
  supply voltage recovers. `gACT` stays 0 until re-activation, and
  `gFLTO` keeps the code until `FAULT_CLEAR`.
- Firmware ≤ v1.19.0 kept `gACT` = 1 and `gSTA` = Active after such a fault
  although the motor was off. The recovery steps were the same.

#### Device config & developer channel / 장치 설정 (`0x0180~`)

Config writes land in RAM only — persist them with `DEV_CMD 0x02
SAVE_CONFIG`; address/baud changes apply after a power cycle. 상세 절차는
매뉴얼 참고.

| Addr | R/W | Field | Meaning |
|---|---|---|---|
| `0x0180` | W | `DEV_CMD` | developer/config command code — key codes below |
| `0x0181` | R | `DEV_STATUS` | 0 = IDLE · 1 = RUNNING · 2 = OK · 3 = ERROR |
| `0x0182` | R/W | `CFG_MB_ADDR` | Modbus slave address, 1..247 (see "Multiple grippers on one bus" in [python/README.md](python/README.md#multiple-grippers-on-one-bus-rs485-multi-drop)) |
| `0x0183`/`0x0184` | R/W | `CFG_MB_BAUD_LO/HI` | serial baud rate, raw bps as 32-bit LO/HI (e.g. 115200 → `0xC200`/`0x0001`). Changing the rate is **(개발중, 구현 예정)** — see [Baud rate change](#baud-rate-change--보율-변경-개발중-구현-예정); use 115200 for now |
| `0x0185`/`0x0186` | R/W | `CFG_CAN_ID` / `CFG_CAN_RATE` | FDCAN node ID / rate (1 = 1 Mbps, 2 = 500 kbps) |
| `0x0187` | R | `CFG_DEV_TYPE` | device type (101 = AIDIN_GRIPPER) |
| `0x0188`/`0x0189` | R | `CFG_SERIAL_LO/HI` | serial number, 32-bit LO/HI |
| `0x018A` | R/W | `CFG_SENSOR_TYPE` | fingertip sensor: 0 = none, 1 = AFT F/T, 2 = tactile — applied at boot |
| `0x018B` | R | `CFG_STROKE_MRAD` | calibrated stroke [rad × 1000] — set only by `DEV_CMD 0x0A STROKE_CAL` |
| `0x018C` | R/W | `CFG_MB_RESP_DELAY` | Modbus response delay [µs] 0–65535, default 0 — extra wait before the reply for masters that are slow to switch RS-485 back to receive (robot tool-flange ports); applied on write, persisted by `SAVE_CONFIG` (protocol v1.22.0) |
| `0x0190~0x0196` | R/W | `PID_*` | position/speed/current loop gains — tuning only, see manual |
| `0x0197~0x019A` | R/W | `FF_*` | position feedforward gains — applied via `DEV_CMD 0x0B/0x0C` |
| `0x019B`/`0x019C` | R/W | `TRAJ_MAX_VEL/ACC` | trajectory limits (protocol v1.11.0) — vel [rad/s] 1..150 (boot default 100), acc [rad/s²] 1..800 (boot default 400). Applied via `DEV_CMD 0x0E TRAJ_APPLY`; RAM only, reboot restores defaults |

#### (지속 업데이트) Force control / 힘 제어 (`0x01A0~0x01A9`)

Constant-force grasp, impedance/admittance compliance, hand-guiding. Gains
are int16 fixed-point (register = physical gain × 10). Write parameters
first, switch `FC_MODE` last, then **read `FC_MODE` back** — the firmware
resets it to 0 when entry is rejected (e.g. impedance/admittance before
homing). 모드별 파라미터, 단위계, 안전 폴백 등 상세는 매뉴얼 참고.

| Addr | R/W | Field | Meaning |
|---|---|---|---|
| `0x01A0` | R/W | `FC_MODE` | 0 = off · 1 = simple · 2 = impedance · 3 = admittance — readback = actual mode |
| `0x01A1` | W | `FC_FORCE` | target force (simple) / external force F_ext (admittance) [mA] |
| `0x01A2~0x01A4` | W | `FC_STIFFNESS` / `FC_DAMPING` / `FC_MASS` | Kx / Kb / Mv gains (× 10) |
| `0x01A5` | W | `FC_EQUIL` | equilibrium position 0..255 (0 = open) — live-updatable while active |
| `0x01A6` | W | `FC_SPEED_LIMIT` | speed safety limit [rad/s × 100], 0 = firmware default |
| `0x01A7` | W | `FC_FB_SRC` | feedback source: 0 = sensorless · 1/2 = sensor tip1/tip2 Fz (AFT or tactile) · 3 = tip1+tip2 · 4 = Iq self-sensing |
| `0x01A8` | R | `FC_FORCE_ACT` | measured force feedback — [N × 100] with F/T sensor source, else [mA] |
| `0x01A9` | R | `FC_POS_ACT` | position feedback 0..255 |

#### (개발중, 지속 업데이트 예정) Fingertip sensors / 핑거팁 센서 (`0x0210~`)

Fingertip sensor values relayed from CAN; select the sensor with
`CFG_SENSOR_TYPE` (0x018A: 1 = AFT F/T, 2 = tactile) + `SAVE_CONFIG` +
power cycle. Tip 1 = **right** finger, tip 2 = **left** finger.
상태 비트필드와 스케일링 상세는 매뉴얼 참고.

| Addr | R/W | Field | Meaning |
|---|---|---|---|
| `0x0210~0x0216` | R | `SENSOR_STATUS`, `FT_FX..FT_TZ` | tip 1 six-axis F/T — force [N × 100], torque [Nm × 1000]. With the tactile sensor: its Fx/Fy/Fz, torque 0 |
| `0x0217~0x021D` | R | `SENSOR2_STATUS`, `FT2_FX..FT2_TZ` | tip 2, same layout — poll both tips as one 14-register block read |
| `0x0220~0x0227` | R | `TACT1_*` | tip 1 tactile resultant: status, contact point X/Y [mm × 100], Fx/Fy/Fz [N × 100], rate [Hz], frame count — [details](#tactile-sensor--촉각-센서-0x0220) |
| `0x0228~0x022F` | R | `TACT2_*` | tip 2, same layout — poll both tips as one 16-register block read from `0x0220` |
| `0x0230~0x0276` / `0x0277~0x02BD` | R | `TACT1_RAW` / `TACT2_RAW` | raw 69-channel diagnostic stream per tip (bring-up only) — valid while status `raw_fresh` = 1 |
| `0x01B0~0x01BA` | W | `TACT_CMD_*` | tactile CAN passthrough (tip, len, data[8], CAN ID) — sent as CAN FD with `DEV_CMD 0x09` |
| `0x01BB~0x01BD` | W | `TACT_SCMD_*` | tactile structured command (code, tip bitmask, arg) — run with `DEV_CMD 0x0F` |
| `0x02C0~0x02C9` | R | `FAST_*` | fast-poll block: status + position + current + both tips' tactile + fault in one read — [details](#fast-poll-block--고속-폴링-블록-0x02c00x02c9) |

### Tactile sensor / 촉각 센서 (`0x0220~`)

Firmware protocol ≥ v1.13.0 (gripper-frame coordinates below: ≥ v1.18.0).
Each fingertip's 3-axis tactile sensor (23 cells) sends one **contact
resultant** at 100 Hz over CAN FD: where the contact is and how hard it
pushes. The firmware relays it as 8 registers per tip.

각 핑거팁의 3축 촉각 센서(23셀)는 100 Hz 로 **접촉 합력**(작용점 + 3축 힘)을
보내고, 펌웨어가 팁당 8개 레지스터로 전달합니다.

| Offset (tip 1 / tip 2) | Field | Type | Meaning |
|---|---|---|---|
| `0x0220` / `0x0228` | `TACT_STATUS` | bitfield | link / validity bits — table below. Low byte = sensor type (2) |
| `0x0221` / `0x0229` | `POSX` | int16 | contact point X [mm × 100]: **0..3200 from the wrist-side edge of the cell area toward the fingertip. 0 = no contact point** |
| `0x0222` / `0x022A` | `POSY` | int16 | contact point Y [mm × 100]: ±1250 across the pad width (centre = 0) |
| `0x0223` / `0x022B` | `FX` | int16 | shear force [N × 100] along +X (toward the fingertip), after tare |
| `0x0224` / `0x022C` | `FY` | int16 | shear force [N × 100] along +Y, after tare |
| `0x0225` / `0x022D` | `FZ` | int16 | normal force [N × 100], **positive = pressing**, after tare |
| `0x0226` / `0x022E` | `RATE` | uint16 | result frames received in the last second [Hz] (≈ 100 when healthy) |
| `0x0227` / `0x022F` | `RXCNT` | uint16 | result frames since boot (wraps at 65535) |

`mm = POS / 100`, `N = F / 100`. While `valid` = 0 every value reads 0.
With `valid` = 1 but a light touch (Fz < ~1 N) there is no contact point
and `POSX = POSY = 0` — a real contact point is always `POSX` ≥ ~2.7 mm,
so 0 never collides with a valid one. The pad is 50 × 25 mm; the cells
cover the 32 mm toward the fingertip.

**`TACT_STATUS` bits**

| Bit | Mask | Name | Meaning |
|---|---|---|---|
| 8 | `0x0100` | `valid` | result frame within the last 50 ms and its values are valid |
| 9 | `0x0200` | `saturated` | a value hit its int16 limit or the sensor reported over-range |
| 10 | `0x0400` | `timeout` | no result frame for 50 ms |
| 11 | `0x0800` | `connected` | any frame from this tip within the last 200 ms |
| 12 | `0x1000` | `raw_stream` | connected, but the sensor sends raw channels instead of results |
| 13 | `0x2000` | `contact` | `valid` and Fz ≥ 0.5 N |
| 14 | `0x4000` | `seen` | detected at least once since boot |
| 15 | `0x8000` | `raw_fresh` | raw diagnostic block (`0x0230~`) updated within 200 ms |

**Link state / 연결 상태** — what to show the user (the SDKs return this as `link`):

| seen | connected | valid | timeout | raw_stream | State | Meaning |
|:-:|:-:|:-:|:-:|:-:|---|---|
| 0 | 0 | 0 | 1 | 0 | `NOT_DETECTED` | never seen: not fitted, wiring, CAN ID mismatch, or `CFG_SENSOR_TYPE` ≠ 2 |
| 1 | 0 | 0 | 1 | 0 | `DISCONNECTED` | was connected, frames stopped |
| 1 | 1 | 0 | 1 | 1 | `RAW_STREAM` | connected, sensor firmware sends raw channels (no results) |
| 1 | 1 | 0 | 0 | 0 | `WAITING` | results arrive but are invalid: sensor boot zero (~4.5 s) or right after a zero (~1.25 s) |
| 1 | 1 | 1 | 0 | 0 | `LIVE` | normal |

**How to read it / 읽는 방법**

1. **Once**: write `CFG_SENSOR_TYPE` (`0x018A`) = 2 → `DEV_CMD` = `0x02`
   (`SAVE_CONFIG`) → power-cycle. (`python examples/08_tactile_sensor.py PORT --enable`)
2. After power-up each tip reads `WAITING` for ~4.5 s (sensor boot zero),
   then `LIVE`.
3. Read **16 registers from `0x0220`** (both tips, one transaction) at
   whatever rate you need — the sensor updates at 100 Hz.
4. **Zero** with the pads untouched: write `0x01BB` = `0x01` (ZERO),
   `0x01BC` = tip bitmask (`0x01` right, `0x02` left, `0x03` both),
   `0x01BD` = 0, then `DEV_CMD` = `0x0F`; `DEV_STATUS` OK = command sent.
   The tip shows `WAITING` for ~1.25 s, then `LIVE` with the new zero.
   It is rejected (ERROR) while force control uses the sensor as feedback.
   `DEV_CMD 0x08 SENSOR_ZERO` is the alternative software tare (captures the
   current Fx/Fy/Fz as zero; RAM only).

| Step | Python | C++ | ROS2 |
|---|---|---|---|
| Read both tips | `right, left = g.read_tactile()` | `auto tips = g.readTactile();` (`[0]` right, `[1]` left) | `publish_tactile:=true`, subscribe `/aidin_gripper_driver/tactile` (`TactileState`) |
| Link state | `right.link` → `TactileLink.LIVE` … | `tips[0].link()` → `TactileLink::Live` … | `msg.right.link` → `TactileTip.LINK_LIVE` … |
| Sensor zero | `g.tactile_zero()` | `g.tactileZero()` | `/aidin_gripper_driver/tactile_zero` (`Trigger`) |
| Software tare | `g.sensor_zero()` | `g.sensorZero()` | `/dev_cmd` `cmd: 8` |
| Raw 69 channels | `g.read_tactile_raw(reg.TACT_TIP_LEFT)` | `g.readTactileRaw(reg::TACT_TIP_LEFT)` | — |

> ⚠ Still being validated on hardware: the Fx/Fy sign convention assumes the
> sensor's force and position share one frame, and the right-hand sensor's
> zero CAN ID is not confirmed yet — for that tip the ZERO command falls back
> to the firmware software tare. Firmware older than v1.18.0 reports
> positions in the sensor's own frame (centre origin, rotated 180°).

### Fast-poll block / 고속 폴링 블록 (`0x02C0~0x02C9`)

Firmware protocol ≥ v1.15.0 (the 8-bit tactile scale below: ≥ v1.19.0).
A **read-only copy** of the essentials —
gripper status, position, current, fault and both fingertips' tactile
resultant — packed so **one FC03 read of 10 registers** returns them all.
The firmware rewrites the 10 words in one go, so every reply is a single
consistent snapshot. Writes are ignored (overwritten on the next update);
the original addresses (`0x0200~`, `0x0220~`) stay valid.

필수 값만 **FC03 한 번(10워드)** 으로 읽기 위한 특수 용도 읽기 전용 블록입니다.

#### Register layout / 레지스터 배치

| Addr | [15:8] | [7:0] |
|---|---|---|
| `0x02C0` | 0 | `STATUS` |
| `0x02C1` | 0 | `gPO` |
| `0x02C2` | 0 | `gCU` |
| `0x02C3` | `Px` R | `Py` R |
| `0x02C4` | `Fx` R | `Fy` R |
| `0x02C5` | `Fz` R | `status8` R |
| `0x02C6` | `Px` L | `Py` L |
| `0x02C7` | `Fx` L | `Fy` L |
| `0x02C8` | `Fz` L | `status8` L |
| `0x02C9` | 0 | `gFLT` |

R = right finger (tip 1), L = left finger (tip 2). PLC "4x" addresses:
40705..40714.

| Field | Register [bits] | Type | Resolution | Range | Copy of | Meaning |
|---|---|---|---|---|---|---|
| `STATUS` | `0x02C0` [7:0] | bits | — | — | `0x0200` | `gACT` · `gHOM` · `gGTO` · `gSTA` · `gOBJ`, same bits as [`0x0200`](#status-bits-read-0x0200) |
| `gPO` | `0x02C1` [7:0] | uint8 | — | 0..255 | `0x0203` | actual position, 0 = open, 255 = closed |
| `gCU` | `0x02C2` [7:0] | uint8 | — | 0..255 | `0x0204` | actual motor current (normalized) |
| `Px` | `0x02C3` / `0x02C6` [15:8] | uint8 | 32/255 ≈ 0.1255 mm | 0 .. 32.0 mm | `POSX` `0x0221` / `0x0229` | contact point X, from the wrist-side edge of the cell area toward the fingertip; **0 = no contact point** |
| `Py` | `0x02C3` / `0x02C6` [7:0] | int8 | 12.5/127 ≈ 0.0984 mm | −12.5 .. +12.5 mm | `POSY` `0x0222` / `0x022A` | contact point Y across the pad width (centre = 0) |
| `Fx` | `0x02C4` / `0x02C7` [15:8] | int8 | 35/127 ≈ 0.2756 N | −35 .. +35 N ⚠ | `FX` `0x0223` / `0x022B` | shear force along +X, after tare |
| `Fy` | `0x02C4` / `0x02C7` [7:0] | int8 | 35/127 ≈ 0.2756 N | −35 .. +35 N ⚠ | `FY` `0x0224` / `0x022C` | shear force along +Y, after tare |
| `Fz` | `0x02C5` / `0x02C8` [15:8] | uint8 | 70/255 ≈ 0.2745 N | 0 .. 70 N | `FZ` `0x0225` / `0x022D` | normal force, positive = pressing, after tare |
| `status8` | `0x02C5` / `0x02C8` [7:0] | — | — | — | — | tactile status — **(개발중, 구현 예정)**, see [below](#tactile-status-status8--개발중-구현-예정) |
| `gFLT` | `0x02C9` [7:0] | uint8 | — | 0..11 | `0x0201` | current fault code ([fault codes](#fault-codes--폴트-코드-gflt--gflto)); the latched `gFLTO` is not in the block |

- Modbus sends the high byte first, so each tip's bytes arrive as
  `Px, Py, Fx, Fy, Fz, status8`. Unused high bytes are 0.
- A tip without a valid reading has all five values at 0. With
  `CFG_SENSOR_TYPE` ≠ 2 all six tactile words are 0.
- Reading 9 registers (`0x02C0~0x02C8`) skips `gFLT`.
- Each field's full scale maps to the end of its byte range, so the
  resolution differs per field ([details](#tactile-8-bit-encoding--촉각-8비트-압축)).
  ⚠ The Fx / Fy range ±35 N is **provisional** (half of Fz) until the
  shear-force range is specified.

#### Tactile 8-bit encoding / 촉각 8비트 압축

The tactile source registers are int16 at a ×100 scale ([mm × 100],
[N × 100]). The block keeps each value in one byte, scaled so that the
**full scale of each field lands on the end of the byte range**
(uint8 → 255, int8 → ±127). The resolution — the value of one byte
step — therefore differs per field:

| Field | Full scale | Source at full scale | Byte | Byte range | Resolution |
|---|---|---|---|---|---|
| `Px` | 32.0 mm (cell area) | 3200 | uint8 | 0 .. 255 | 32/255 ≈ 0.1255 mm |
| `Py` | ±12.5 mm (pad width) | ±1250 | int8 | −127 .. +127 | 12.5/127 ≈ 0.0984 mm |
| `Fx`, `Fy` | ±35 N ⚠ | ±3500 | int8 | −127 .. +127 | 35/127 ≈ 0.2756 N |
| `Fz` | 70 N | 7000 | uint8 | 0 .. 255 | 70/255 ≈ 0.2745 N |

> ⚠ **Fx / Fy ±35 N is provisional** — set to half of the Fz range until
> the shear-force range is specified. Px, Py and Fz are final. The
> firmware keeps the full scales in `aidin_modbus.h` (`FAST_PX_FS`,
> `FAST_PY_FS`, `FAST_FXY_FS`, `FAST_FZ_FS`), the SDKs in
> `registers` (`FAST_PX_MAX`, `FAST_PY_MAX`, `FAST_FXY_MAX`, `FAST_FZ_MAX`)
> — change them together.
>
> Fx / Fy 범위(±35 N)는 전단력 사양 확정 전까지의 잠정값(Fz 범위의 절반)입니다.

**Firmware packing** (per field, integer arithmetic):

```
v    = source register                      # int16, ×100 scale
v    = max(v, 0)                            # Px and Fz only
n    = v × qmax                             # qmax = 255 (Px, Fz) · 127 (Py, Fx, Fy); fits int32
q    = (n >= 0) ? (n + fs/2) / fs           # ÷ full scale, round half away from zero
                : (n - fs/2) / fs           #   (C integer division truncates toward zero)
q    = clamp(q, lo, qmax)                   # Px, Fz: lo = 0 · Py, Fx, Fy: lo = −qmax
byte = q & 0xFF
```

`fs` is the source value at full scale: 3200 (Px) · 1250 (Py) ·
3500 (Fx, Fy) · 7000 (Fz). −128 (`0x80`) is never produced, so the signed
fields are symmetric.

**Host decoding**:

```
Px [mm] = byte × 32 / 255            Fz [N]     = byte × 70 / 255         # uint8
Py [mm] = int8(byte) × 12.5 / 127    Fx, Fy [N] = int8(byte) × 35 / 127   # int8: byte ≥ 128 → byte − 256
```

**Conversion examples**

| Field | Source register | Physical | × qmax / fs, rounded | After clamp | Byte | Decoded |
|---|---|---|---|---|---|---|
| `Px` | 1523 | 15.23 mm | 121.36 → 121 | 121 | `0x79` | 15.18 mm |
| `Px` | 270 | 2.70 mm (smallest real contact point) | 21.52 → 22 | 22 | `0x16` | 2.76 mm |
| `Px` | 0 | no contact point | 0 | 0 | `0x00` | 0 = no contact point |
| `Px` | 3200 | 32.00 mm (full scale) | 255 | 255 | `0xFF` | 32.00 mm |
| `Py` | −412 | −4.12 mm | −41.86 → −42 | −42 | `0xD6` | −4.13 mm |
| `Py` | −1250 | −12.50 mm (full scale) | −127 | −127 | `0x81` | −12.50 mm |
| `Fx` | −87 | −0.87 N | −3.16 → −3 | −3 | `0xFD` | −0.83 N |
| `Fy` | 230 | 2.30 N | 8.35 → 8 | 8 | `0x08` | 2.20 N |
| `Fz` | 1234 | 12.34 N | 44.95 → 45 | 45 | `0x2D` | 12.35 N |
| `Fz` | 13 | 0.13 N | 0.47 → 0 | 0 | `0x00` | 0.00 N |
| `Fz` | 14 | 0.14 N | 0.51 → 1 | 1 | `0x01` | 0.27 N |
| `Fz` | −15 | −0.15 N (after tare) | 0 (negative → 0 first) | 0 | `0x00` | 0.00 N |
| `Fz` | 6000 | 60.00 N | 218.57 → 219 | 219 | `0xDB` | 60.12 N |
| `Fx` | −4000 | −40.00 N (beyond range) | −145.14 → −145 | **−127** | `0x81` | −35.00 N |
| `Fz` | 8000 | 80.00 N (beyond range) | 291.43 → 291 | **255** | `0xFF` | 70.00 N |

- **Resolution**: inside the range the error is at most half the resolution —
  ±0.063 mm (`Px`), ±0.049 mm (`Py`), ±0.138 N (`Fx`, `Fy`), ±0.137 N
  (`Fz`). Smaller forces read 0. Position resolution is finer than the
  old 0.2 mm step; force resolution is coarser (0.27 N instead of 0.2 N)
  because the force range grew from 51 N to 70 N.
- **Range**: the position full scales are the cell area itself (X 0..32 mm,
  Y ±12.5 mm), so positions always fit. Forces beyond 70 N (Fz) or ±35 N
  (Fx, Fy) clamp and stay at the end value; how a clamp is flagged belongs
  to the tactile status (`status8`, under development).
- Fz is floored at 0, so a slightly negative reading after tare shows 0.

#### Tactile status (`status8`) — (개발중, 구현 예정)

The low byte of `0x02C5` / `0x02C8` is reserved for the tactile status
(validity, connection, saturation, contact). Its definition is not fixed
yet — **under development, to be implemented**. Don't rely on its bits for
now.

택타일 status(`status8`)는 아직 정의가 확정되지 않았습니다 — 개발 중이며 구현 예정입니다.

#### Wire example / 프레임 예

Slave 1, read 10 registers from `0x02C0`:

```
Request : 01 03 02 C0 00 0A C4 49
Response: 01 03 14 00 BB 00 C8 00 25 79 D6 FD 08 2D 69 00 00 00 00 00 48 00 00 49 F5
```

| Register | Word | Decoded |
|---|---|---|
| `0x02C0` | `00 BB` | STATUS: gACT 1, gHOM 1, gGTO 1, gSTA 3 (Active), gOBJ 2 (stopped on an object while closing) |
| `0x02C1` | `00 C8` | gPO = 200 |
| `0x02C2` | `00 25` | gCU = 37 |
| `0x02C3` | `79 D6` | right Px = 121 × 32/255 = **15.18 mm** · Py = int8(0xD6) = −42 → × 12.5/127 = **−4.13 mm** |
| `0x02C4` | `FD 08` | right Fx = int8(0xFD) = −3 → × 35/127 = **−0.83 N** · Fy = 8 → **2.20 N** |
| `0x02C5` | `2D 69` | right Fz = 45 × 70/255 = **12.35 N** · status8 `0x69` (개발중 — not defined yet) |
| `0x02C6`–`0x02C7` | `00 00 00 00` | left values 0 (no valid reading) |
| `0x02C8` | `00 48` | left Fz = 0 · status8 `0x48` (개발중 — not defined yet) |
| `0x02C9` | `00 00` | gFLT = 0 |

At full resolution (`0x0221~0x0225`) the right tip of this example reads
POSX 1523, POSY −412, FX −87, FY 230, FZ 1234 (15.23 mm, −4.12 mm,
−0.87 N, 2.30 N, 12.34 N).

#### Polling rate and bus timing / 폴링 주기

One poll is an 8-byte request plus a 25-byte response (23 bytes for 9
registers), 10 bits per byte (8N1), plus the firmware's response delay
and the host's own delay.

| Baud | Line time (10 regs) | + Firmware response delay | ≈ Minimum cycle | |
|---|---|---|---|---|
| 115200 | 2.87 ms | 0.65 – 0.9 ms | 3.5 – 3.8 ms | measured 4.0 ms |
| 230400 | 1.43 ms | 0.65 – 0.9 ms | 2.1 – 2.3 ms | estimate ⚠ |
| 460800 | 0.72 ms | 0.65 – 0.9 ms | 1.4 – 1.6 ms | estimate ⚠ |
| 921600 / 1000000 | 0.36 / 0.33 ms | 0.65 – 0.9 ms | 1.0 – 1.3 ms | estimate ⚠ |

- The firmware response delay comes from a fixed 500 µs end-of-frame
  timer plus the 5 kHz Modbus task tick. It does not shrink with the baud
  rate, so above ~460800 it dominates the cycle.
- Rows above 115200 depend on the baud-rate change, which is **under
  development** — see [Baud rate change](#baud-rate-change--보율-변경-개발중-구현-예정).
- **USB adapter latency**: FTDI adapters hold short replies for up to
  their latency timer, which defaults to **16 ms** — that alone caps
  polling near 60 Hz. Set it to 1 ms: Linux
  `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`
  (persistent udev rule: [ros2/README.md](ros2/README.md#troubleshooting));
  Windows: Device Manager → the COM port → Port Settings → Advanced →
  Latency Timer = 1.
- **C++ SDK**: waits the Modbus-standard 1.75 ms after each transaction
  before sending the next one, so back-to-back `readFastState()` calls take
  about cycle + 1.75 ms (≈ 5.3–5.8 ms, ~175–190 Hz at 115200 — estimate). The
  Python SDK's rate has not been measured.
- Tactile data arrives at 100 Hz, so 100–200 Hz polling captures every
  tactile frame; faster polling mainly helps gripper status. The block
  has no frame counter — read `RXCNT` (`0x0227` / `0x022F`) to tell a new
  tactile frame from a repeat.
- Several grippers on one RS485 bus share the wire: each gripper is one
  block read, so the cycle grows with the number of grippers.

#### Baud rate change / 보율 변경 (개발중, 구현 예정)

**Under development — to be implemented.** Use the default 115200 bps
(8N1) for now. Planned procedure:

1. Write the new rate as raw bps split into two words:
   `CFG_MB_BAUD_LO` (`0x0183`) = low 16 bits, `CFG_MB_BAUD_HI` (`0x0184`)
   = high 16 bits — e.g. 1000000 → `0x4240` / `0x000F`, 921600 →
   `0x1000` / `0x000E`.
2. Write `DEV_CMD` (`0x0180`) = `0x02` (`SAVE_CONFIG`) and check
   `DEV_STATUS` (`0x0181`) = 2 (OK). A rate outside the supported list
   fails (3 = ERROR) and nothing is saved.
3. Power-cycle the gripper and reconnect at the new rate — the UART
   switches only at the next boot.

- Planned rates: 9600 · 19200 · 38400 · 57600 · 115200 · 230400 · 460800 ·
  921600 · 1000000 bps, 8N1.
- Check that your USB-RS485 adapter supports the rate **before** saving
  it; otherwise you need an adapter that does to change it back.
- Every device on one RS485 bus must use the same rate.
- A higher rate only shortens the line time; the firmware response delay
  (0.65 – 0.9 ms) stays, so 1 Mbps brings a poll from ~4 ms to roughly
  1.0 – 1.3 ms (estimate), not a 9× speed-up.

보율 변경은 개발 중이며 구현 예정입니다. 지금은 기본 115200 bps(8N1)를 사용하세요.

### Key `DEV_CMD` codes (`0x0180`)

Write the code to `DEV_CMD` (0x0180), then poll `DEV_STATUS` (0x0181)
until it leaves RUNNING (→ OK or ERROR). 전체 코드 목록(전기각 보정,
PID 저장/복원 등)과 절차는 매뉴얼 참고.

| Code | Name | What it does |
|---|---|---|
| `0x02` | `SAVE_CONFIG` | persist the config registers (`0x0182~`) to flash |
| `0x07` | `FAULT_CLEAR` | acknowledge the motor fault + clear latched `gFLTO` |
| `0x08` | `SENSOR_ZERO` | software tare of the fingertip sensors — F/T Fx..Tz, tactile Fx/Fy/Fz (per-session, RAM only) |
| `0x09` | `TACTILE_SEND` | send the `TACT_CMD_*` bytes (`0x01B0~`) to the tactile sensor as one CAN FD frame, uninterpreted |
| `0x0A` | `STROKE_CAL` | re-measure the usable stroke after a fingertip change (~20 s, saved to flash) |
| `0x0E` | `TRAJ_APPLY` | apply `TRAJ_MAX_VEL/ACC` (`0x019B/0x019C`) to the trajectory generator — RAM only; rejected (ERROR) out of range or during homing/calibration |
| `0x0F` | `TACTILE_CMD` | run the tactile structured command in `0x01BB~0x01BD` — `0x01` ZERO (sensor no-load zero), `0x05` FACTORY_RESET (arg `0xA5A5`); `0x02~0x04` unsupported (ERROR) |

> Flash-writing commands (`SAVE_CONFIG`, `SAVE_PID`, `FF_SAVE`) stall
> Modbus for ~10 ms — use a response timeout ≥ 50 ms.

### Protocol invariants an integration needs to get right

- **Position is inverted from what you might guess**: `0` = open, `255` = closed.
- `rHOM`/`rATR` are edge-triggered, not level-triggered — write them low,
  then high (every SDK here waits ~20 ms between the two writes), or the
  firmware never sees a transition.
- Re-issuing a move to the *same* position while already moving there
  needs an `rGTO` 0→1 pulse, not just rewriting `rGTO=1`.
- `emergency_release()`/`emergencyRelease()` drops `rACT` — call
  `activate()` again afterward, or every subsequent command is ignored.
- If you use **force control** (`FC_*`): write the parameters first and
  switch `FC_MODE` last, then **read `FC_MODE` back** — the firmware
  silently resets it to 0 when entry is rejected (impedance/admittance
  require homing first) and when `rATR` force-terminates force control.
- Treat a full status read (8 registers, `0x0200`-`0x0207`) as one
  transaction; don't split it into 8 separate reads, both for latency and
  because the fields are meant to be read as one consistent snapshot.
  When you also need tactile data at a high rate, one read of the
  fast-poll block (`0x02C0`, 10 registers) replaces both reads.
- Tactile tip 1 is the **right** finger, tip 2 the **left**. A tactile
  `POSX` of 0 means "no contact point", not "at the edge". Only zero the
  sensor (`TACTILE_CMD` ZERO) with nothing touching the pads, and expect
  ~1.25 s of `WAITING` (valid = 0) afterwards.
- **RS485 multi-drop**: several grippers can share one physical bus if
  each has a distinct `CFG_MB_ADDR`, but only one Modbus transaction may
  be in flight on the wire at a time. All three SDKs in this repo share
  one underlying serial connection per port path and serialize
  transactions with a lock/mutex whenever more than one logical device
  object targets the same port — don't open the port twice from
  independent client objects, and don't fire transactions from multiple
  threads without equivalent serialization if you re-implement this
  elsewhere.

### Multi-drop: sharing one connection across slave IDs (the pattern used here)

Naively giving each logical gripper its own client object breaks as soon
as two of them target the same physical port: opening a COM port twice
fails outright on Windows, and on Linux it can silently interleave two
independent handles' bytes on the wire and corrupt every transaction.
This repo solves it the same way in both compiled SDKs — the pattern to
copy if you're writing a fourth client:

- A **process-wide registry keyed by port path** (`{port_path: shared_transport}`)
  holds one already-open transport per physical port.
- `connect(port, ..., slave_id)` looks the port up in the registry. Not
  found → open it, create the shared transport, register it. Found →
  reuse it, but reject the call if `baudrate`/`parity` don't match what's
  already open (those are properties of the wire, not of one device).
- Each logical `Gripper` object keeps only its own `slave_id` plus a
  reference to the shared transport — never its own OS handle.
- The shared transport owns a **mutex/lock held only around a single
  register read or write**, not around a whole `activate()`/`home()`
  call — so gripper A's 15-second homing doesn't block gripper B's status
  polling.
- The transport is closed automatically once every `Gripper` sharing it
  has disconnected (reference counting — Python does it explicitly with
  an int counter; C++ leans on `std::shared_ptr`/`std::weak_ptr` and gets
  this for free).

Reference implementations:
[`python/aidin_gripper/gripper.py`](python/aidin_gripper/gripper.py)
(`_SharedPort` / `_port_registry`) and
[`cpp/src/gripper.cpp`](cpp/src/gripper.cpp) (`SharedBus` /
`portRegistry()`), demonstrated end-to-end in
[`python/examples/07_multi_gripper_two_slaves.py`](python/examples/07_multi_gripper_two_slaves.py).

**Scope limit**: the registry lives in one process's memory, so this only
shares a port across multiple client objects *within the same process*.
Two independent OS processes (e.g. two separately-launched
`aidin_gripper_driver` ROS2 node instances) each still open the port for
themselves and will conflict if pointed at the same physical port — the
ROS2 driver as shipped is one node = one gripper = one port; running two
grippers on one bus under ROS2 currently means giving each its own
port/adapter, not sharing one, unless you extend the driver to own
multiple slave IDs itself.

### Minimal integration algorithm (language-agnostic)

```
open(port, baud=115200, 8N1)

write(ACTION, rACT)                            # request activation
poll read(STATUS) until gSTA == Active          # or timeout -> error

write(ACTION, rACT)                            # ensure rHOM low
sleep(20ms)
write(ACTION, rACT | rHOM)                     # rising edge
sleep(100ms)                                    # let firmware clear gHOM first
poll read(STATUS..FAULT_LATCH) until gHOM==1 or gFLT!=0   # or timeout -> error

write(SPEED, speed); write(FORCE, force); write(POSITION, pos)
write(ACTION, rACT | rGTO)
if blocking:
    poll read(STATUS..FAULT_LATCH) until gOBJ != Moving or gFLT!=0   # or timeout -> error

# status monitoring: read(STATUS..FAULT_LATCH) on whatever cadence you need
# fault/jam recovery: write(ACTION, rATR [| rADR]) briefly, then re-run the activate step
# tactile (CFG_SENSOR_TYPE=2): read(0x0220, 16) -> per tip [status][posX][posY][fx][fy][fz][rate][rxcnt]
# fast poll (fw >= v1.15): read(0x02C0, 10) -> status, gPO, gCU, both tips' tactile (8-bit), gFLT
```

### Same algorithm, three ways (already implemented here)

| Step | Python (`python/aidin_gripper`) | C++ (`aidin::Gripper`, `cpp/`) | ROS2 (`aidin_gripper_driver`) |
|---|---|---|---|
| Connect | `g.connect(port, baudrate, parity, slave_id)` | `g.connect(port, baud, parity, slaveId)` | `port`/`baudrate`/`parity`/`slave_id` params |
| Activate | `g.activate()` | `g.activate()` | `/activate` (`Trigger`) |
| Home | `g.home()` | `g.home()` | `/home` (`Trigger`) |
| Full init (activate + stroke cal, `ACTION=0xA5`) | `g.full_init()` | `g.fullInit()` | `/full_init` (`Trigger`) |
| Trajectory limits (RAM only) | `g.set_traj_limits(vel, acc)` | `g.setTrajLimits(vel, acc)` | registers + `/dev_cmd` (`0x0E`) |
| Move | `g.move_to(pos, speed, force, blocking)` | `g.moveTo(pos, speed, force, blocking)` | `/move_to` (`MoveTo`) |
| Status | `g.read_state()` → `GripperState` | `g.readState()` → `GripperState` | subscribe `/state` (`GripperState` msg, 50 Hz) |
| Fast poll (`0x02C0`, one read) | `g.read_fast_state()` → `FastState` | `g.readFastState()` → `FastState` | — |
| Tactile (both tips) | `g.read_tactile()` → `(right, left)` `TactileTip` | `g.readTactile()` → `TactilePair` | subscribe `/tactile` (`TactileState`, `publish_tactile:=true`) |
| Tactile zero | `g.tactile_zero()` | `g.tactileZero()` | `/tactile_zero` (`Trigger`) |
| Emergency release | `g.emergency_release(close_direction)` | `g.emergencyRelease(closeDirection)` | `/emergency_release` (`EmergencyRelease`) |
| Raw register access | `g.read_register()` / `g.write_register()` | `g.readRegister()` / `g.writeRegister()` | `/dev_cmd` (opt-out, factory only) |
| Errors | raises `GripperError` | throws `GripperError` | service `success=false` + `message` |

Full method signatures, defaults, and the `GripperState` field lists:
[python/README.md](python/README.md#api-at-a-glance) ·
[cpp/README.md](cpp/README.md#public-api) ·
[ros2/README.md](ros2/README.md#ros2-services). A reference
implementation of the raw Modbus RTU framing (CRC16, timing, retries) if
you need to write a client in a fourth language:
[`cpp/src/modbus_rtu.cpp`](cpp/src/modbus_rtu.cpp).

## License

MIT — see [LICENSE](LICENSE).

---

*AIDIN Robotics — BLDC 모터 기반 그리퍼/액추에이터 솔루션*
