"""Example 8: Read the 3-axis fingertip tactile sensors.

Each fingertip reports one contact resultant at 100 Hz: the contact point
(pos_x, pos_y) [mm] and the force (fx, fy, fz) [N]. Tip 1 = right finger,
tip 2 = left finger. Needs firmware protocol >= v1.18.0 and the sensor type
set to tactile (one-time setup, --enable below).

Usage:
    python 08_tactile_sensor.py /dev/ttyUSB0             # stream both tips @ 20 Hz
    python 08_tactile_sensor.py /dev/ttyUSB0 --zero      # sensor no-load zero first
    python 08_tactile_sensor.py /dev/ttyUSB0 --enable    # set CFG_SENSOR_TYPE=2, save, then power-cycle
    python 08_tactile_sensor.py COM3
"""
import signal
import sys
import time

from aidin_gripper import Gripper, TactileLink
from aidin_gripper import registers as reg


_stop = False


def _on_signal(_signum, _frame):
    global _stop
    _stop = True


def enable_tactile(g: Gripper) -> int:
    g.write_register(reg.CFG_SENSOR_TYPE, reg.SENSOR_TACTILE)
    g.write_register(reg.DEV_CMD, reg.DEV_CMD_SAVE_CONFIG)
    time.sleep(0.2)                          # flash write stalls Modbus ~10 ms
    if g.read_register(reg.DEV_STATUS) != reg.DEV_STATUS_OK:
        print("SAVE_CONFIG failed", file=sys.stderr)
        return 1
    print("Sensor type saved as tactile. Power-cycle the gripper, then run again.")
    return 0


def wait_live(g: Gripper, timeout_s: float = 6.0) -> None:
    """Wait until every detected tip is LIVE (sensor boot zero takes ~4.5 s)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tips = g.read_tactile()
        if all(t.link in (TactileLink.LIVE, TactileLink.NOT_DETECTED) for t in tips):
            break
        time.sleep(0.1)
    right, left = g.read_tactile()
    print(f"right: {right.link.name}   left: {left.link.name}")


def main() -> int:
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    default_port = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
    port = args[0] if args else default_port

    signal.signal(signal.SIGINT, _on_signal)

    try:
        with Gripper() as g:
            g.connect(port)

            if "--enable" in flags:
                return enable_tactile(g)

            sensor = g.read_register(reg.CFG_SENSOR_TYPE)
            if sensor != reg.SENSOR_TACTILE:
                print(f"CFG_SENSOR_TYPE = {sensor}, not tactile (2). "
                      f"Run with --enable once, then power-cycle.", file=sys.stderr)
                return 1

            wait_live(g)

            if "--zero" in flags:
                print("Zeroing - keep the pads untouched...")
                g.tactile_zero()             # both tips; WAITING ~1.25 s, then LIVE
                time.sleep(0.3)
                wait_live(g)

            print("Streaming (CTRL+C to quit). pos [mm], F [N]")
            while not _stop:
                right, left = g.read_tactile()
                print(f"R {right}  |  L {left}", end="\r", flush=True)
                time.sleep(0.05)
            print("\nStopped.")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
