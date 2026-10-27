"""Example 5: Inspect faults, recover, and use the emergency release.

Covers:
  - `fault` (gFLT, live) vs `latched_fault` (gFLTO, history) fields
  - Fault recovery (firmware >= v1.23): a fault switches the gripper off
    (gACT = 0, rACT cleared). Remove the cause, then `activate()` - no
    FAULT_CLEAR needed - and re-send your command. FAULT_CLEAR only clears
    the gFLTO history.
  - `emergency_release()` to free a stuck object: drives the fingers fully
    open (or closed) and ends force control. The gripper stays active, so
    the next move_to() works normally.

Usage:
    python 05_fault_and_emergency_release.py /dev/ttyUSB0 [open|close]
    python 05_fault_and_emergency_release.py COM3 close
"""
import sys
import time

from aidin_gripper import Gripper


def main() -> int:
    default_port = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
    port = sys.argv[1] if len(sys.argv) > 1 else default_port
    direction = sys.argv[2] if len(sys.argv) > 2 else "open"
    if direction not in ("open", "close"):
        print("direction must be 'open' or 'close'", file=sys.stderr)
        return 1

    try:
        with Gripper() as g:
            g.connect(port)

            s = g.read_state()
            print(f"Before: {s}")
            if s.fault:
                print(f"  live fault gFLT = {s.fault} - remove the cause first")
            if s.latched_fault:
                print(f"  fault history gFLTO = {s.latched_fault} (kept until "
                      "g.write_register(reg.DEV_CMD, reg.DEV_CMD_FAULT_CLEAR); "
                      "not needed to re-activate)")

            # Inactive (never activated, deactivated, or switched off by a
            # fault): activate() is the recovery step. The emergency release
            # is skipped on the write that re-activates after a fault, so
            # activate first and send the release afterwards.
            if not s.activated:
                print("Activating ...")
                g.activate()          # raises with gFLT/gFLTO if the cause is still present

            print(f"Emergency release ({direction}) ...")
            g.emergency_release(close_direction=(direction == "close"))
            time.sleep(1.0)           # the call does not wait for the fingers
            print(f"  -> {g.read_state()}")

            print("Done - the gripper is still active; move_to() works as usual.")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
