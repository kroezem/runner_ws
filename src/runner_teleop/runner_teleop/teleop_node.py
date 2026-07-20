import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Invert if physical response is backwards
INVERT_STEER = False
THROTTLE_DEADZONE = 0.05

def _trigger(raw: float) -> float:
    """Normalise -1 pressed / +1 released to 1.0 pressed / 0.0 released."""
    return max(0.0, min(1.0, (1.0 - raw) / 2.0))

class TeleopNode(Node):
    def __init__(self):
        super().__init__("runner_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(0.05, self.publish_cmd)
        self.declare_parameter("axis_steer", 0)
        self.declare_parameter("axis_brake", 2)
        self.declare_parameter("axis_throttle", 5)
        self.declare_parameter("deadman_button", 0)
        self._axis_steer = self.get_parameter("axis_steer").value
        self._axis_brake = self.get_parameter("axis_brake").value
        self._axis_throttle = self.get_parameter("axis_throttle").value
        self._deadman_button = self.get_parameter("deadman_button").value
        self._steer = 0.0
        self._cmd   = 0.0
        self.get_logger().info(
            "runner_teleop ready  |  L-stick=steer  R2=throttle  L2=brake  "
            f"dead-man button index={self._deadman_button}")

    def on_joy(self, msg: Joy):
        if max(self._axis_steer, self._axis_brake, self._axis_throttle) >= len(msg.axes):
            self._cmd = 0.0
            return

        brake_raw = msg.axes[self._axis_brake]
        throttle_raw = msg.axes[self._axis_throttle]
        self._steer = msg.axes[self._axis_steer] * (-1.0 if INVERT_STEER else 1.0)
        throttle = _trigger(throttle_raw)
        brake = _trigger(brake_raw)
        deadman_held = (
            0 <= self._deadman_button < len(msg.buttons)
            and msg.buttons[self._deadman_button] == 1
        )
        self._cmd = (
            -brake if brake > THROTTLE_DEADZONE
            else throttle if deadman_held
            else 0.0
        )

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x  = self._cmd
        msg.angular.z = self._steer
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(TeleopNode())
    if rclpy.ok():
        rclpy.shutdown()
