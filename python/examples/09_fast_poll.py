"""Example 9: High-rate polling with the fast block (0x02C0..0x02C9).

One FC03 read of 10 registers returns gripper status, position, current,
fault AND both fingertips' tactile resultant as one consistent snapshot
(~4 ms per read at 115200 bps). Tactile values are 8-bit compressed
(0.2 mm / 0.2 N per LSB). Needs firmware protocol >= v1.15.0 (>= v1.18.0
for pos_x); tactile fields are 0 unless the sensor type is tactile.

Usage:
    python 09_fast_poll.py /dev/ttyUSB0          # poll as fast as possible until CTRL+C
    python 09_fast_poll.py COM3
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
    default_port = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
    port = sys.argv[1] if len(sys.argv) > 1 else default_port

    signal.signal(signal.SIGINT, _on_signal)

    try:
        with Gripper() as g:
            g.connect(port)
            print(f"Polling {port} (CTRL+C to quit)")

            n, t0 = 0, time.monotonic()
            while not _stop:
                s = g.read_fast_state()
                n += 1
                now = time.monotonic()
                if now - t0 >= 0.5:          # print twice a second, keep polling flat out
                    print(f"{n / (now - t0):6.1f} Hz  pos={s.position:3d} cur={s.current:3d} "
                          f"flt={s.fault}  R={s.right}  L={s.left}", end="\r", flush=True)
                    n, t0 = 0, now
            print("\nStopped.")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
