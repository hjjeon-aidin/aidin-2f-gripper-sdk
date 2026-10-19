"""Subscribe to /aidin_gripper_driver/tactile and print both fingertips.

Run the driver with publish_tactile:=true.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from aidin_gripper_msgs.msg import TactileState

_LINK = {0: "NOT_DETECTED", 1: "DISCONNECTED", 2: "RAW_STREAM", 3: "WAITING", 4: "LIVE"}


def _fmt(t) -> str:
    return (f"{_LINK.get(t.link, '?'):12s} pos=({t.pos_x:5.2f},{t.pos_y:6.2f})mm "
            f"F=({t.fx:5.2f},{t.fy:5.2f},{t.fz:5.2f})N {t.rate_hz:3d}Hz"
            f"{' contact' if t.contact else ''}")


class TactileListener(Node):
    def __init__(self):
        super().__init__("aidin_gripper_tactile_listener")
        self.count = 0
        self.create_subscription(
            TactileState,
            "/aidin_gripper_driver/tactile",
            self.cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Listening for /aidin_gripper_driver/tactile ...")

    def cb(self, msg: TactileState):
        self.count += 1
        if self.count % 10:          # ~5 lines/s at the default 50 Hz
            return
        self.get_logger().info(f"R {_fmt(msg.right)} | L {_fmt(msg.left)}")


def main():
    rclpy.init()
    try:
        rclpy.spin(TactileListener())
    finally:
        rclpy.shutdown()
