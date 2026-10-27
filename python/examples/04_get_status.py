"""Example 4: Print firmware version + serial number, then read the gripper
status once, or watch it change over time.

Usage:
    python 04_get_status.py /dev/ttyUSB0            # read once and exit
    python 04_get_status.py /dev/ttyUSB0 --watch     # poll @ 100 ms until CTRL+C
    python 04_get_status.py COM3 --watch
"""
import signal
import sys
import time

from aidin_gripper import Gripper


_stop = False


def _on_signal(_signum, _frame):
    global _stop
    _stop = True


def main() -> int:
    args = [a for a in sys.argv[1:] if a != "--watch"]
    watch = "--watch" in sys.argv[1:]

    default_port = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
    port = args[0] if args else default_port

    signal.signal(signal.SIGINT, _on_signal)

    try:
        with Gripper() as g:
            g.connect(port)

            serial = g.read_serial_number()
            print(f"Firmware {g.read_firmware_info()}, serial {serial or '(not recorded)'}")

            if not watch:
                print(g.read_state())
                return 0

            print(f"Watching {port} (CTRL+C to quit)")
            prev_sig = None
            while not _stop:
                s = g.read_state()
                sig = (s.stage, s.object_state, s.position, s.fault,
                       s.current // 6)            # bucket current to dampen jitter
                if sig != prev_sig:
                    print(s)
                    prev_sig = sig
                time.sleep(0.1)
            print("Stopped.")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
