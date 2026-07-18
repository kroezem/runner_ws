import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Invert if physical response is backwards
INVERT_STEER = False
INVERT_THROT = False
INVERT_BRAKE = False
THROTTLE_DEADZONE = 0.05
JOY_TIMEOUT_S = 0.3

def _trigger(raw: float, invert: bool) -> float:
    """Normalise a trigger axis to 0.0–1.0. Handles both polarities."""
    v = (raw + 1.0) / 2.0   # -1…+1 → 0…1
    return 1.0 - v if invert else v

def _throttle_curve(t: float) -> float:
    if t < THROTTLE_DEADZONE:
        return 0.0
    scaled = (t - THROTTLE_DEADZONE) / (1.0 - THROTTLE_DEADZONE)
    return scaled ** 3

class TeleopNode(Node):
    def __init__(self):
        super().__init__("runner_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(0.05, self.publish_cmd)
        self.declare_parameter("axis_steer", 0)
        self.declare_parameter("axis_brake", 2)
        self.declare_parameter("axis_throttle", 5)
        self.declare_parameter("deadman_button", 5)
        self._axis_steer = self.get_parameter("axis_steer").value
        self._axis_brake = self.get_parameter("axis_brake").value
        self._axis_throttle = self.get_parameter("axis_throttle").value
        self._deadman_button = self.get_parameter("deadman_button").value
        self._steer = 0.0
        self._cmd   = 0.0
        self._brake_armed = False
        self._throttle_armed = False
        self._last_joy = time.monotonic()
        self._joy_timed_out = False
        self.get_logger().info(
            "runner_teleop ready  |  L-stick=steer  R2=throttle  L2=brake")

    def on_joy(self, msg: Joy):
        if self._joy_timed_out:
            self.get_logger().info("/joy recovered")
            self._joy_timed_out = False
        self._last_joy = time.monotonic()

        if (max(self._axis_steer, self._axis_brake, self._axis_throttle)
                >= len(msg.axes) or self._deadman_button >= len(msg.buttons)):
            self._cmd = 0.0
            return

        brake_raw = msg.axes[self._axis_brake]
        throttle_raw = msg.axes[self._axis_throttle]
        was_armed = self._brake_armed and self._throttle_armed
        self._brake_armed |= brake_raw <= -0.9
        self._throttle_armed |= throttle_raw <= -0.9
        is_armed = self._brake_armed and self._throttle_armed
        if is_armed and not was_armed:
            self.get_logger().info(
                f"trigger axes armed: brake={self._axis_brake}, "
                f"throttle={self._axis_throttle}")

        self._steer = msg.axes[self._axis_steer] * (-1.0 if INVERT_STEER else 1.0)
        if not is_armed or not msg.buttons[self._deadman_button]:
            self._cmd = 0.0
            return

        throt = _throttle_curve(_trigger(throttle_raw, INVERT_THROT))
        brake = _throttle_curve(_trigger(brake_raw, INVERT_BRAKE))
        self._cmd = -brake if brake > 0.0 else throt

    def publish_cmd(self):
        if time.monotonic() - self._last_joy > JOY_TIMEOUT_S:
            if not self._joy_timed_out:
                self.get_logger().warn("/joy watchdog timeout; output stopped")
                self._joy_timed_out = True
            self._cmd = 0.0
            self._steer = 0.0
            self._brake_armed = False
            self._throttle_armed = False
        msg = Twist()
        msg.linear.x  = self._cmd
        msg.angular.z = self._steer
        self.pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(TeleopNode()); rclpy.shutdown()
