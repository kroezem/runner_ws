import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Confirmed axis mapping — DualSense via hid-playstation, Ubuntu 24.04
AXIS_STEER = 0   # left stick X
AXIS_BRAKE = 2   # L2: +1.0=released, -1.0=full press
AXIS_THROT = 5   # R2: +1.0=released, -1.0=full press

# Invert if physical response is backwards
INVERT_STEER = False
INVERT_THROT = True
INVERT_BRAKE = True
THROTTLE_DEADZONE = 0.05

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
        self._steer = 0.0
        self._cmd   = 0.0
        self.get_logger().info(
            "runner_teleop ready  |  L-stick=steer  R2=throttle  L2=brake")

    def on_joy(self, msg: Joy):
        steer = msg.axes[AXIS_STEER] * (-1.0 if INVERT_STEER else 1.0)
        throt = _throttle_curve(_trigger(msg.axes[AXIS_THROT], INVERT_THROT))
        brake = _throttle_curve(_trigger(msg.axes[AXIS_BRAKE], INVERT_BRAKE))
        self._steer = steer
        self._cmd = -brake if brake > 0.0 else throt

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x  = self._cmd
        msg.angular.z = self._steer
        self.pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(TeleopNode()); rclpy.shutdown()
