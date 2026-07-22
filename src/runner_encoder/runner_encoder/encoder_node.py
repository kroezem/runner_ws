# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading

import lgpio
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


GLITCH_FILTER_US = 100
LARGE_VARIANCE = 1e6


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        self.declare_parameter('gpio_pin', 22)
        self.declare_parameter('metres_per_edge', 0.010282)
        self.declare_parameter('window_ms', 50)
        self._gpio_pin = self.get_parameter('gpio_pin').value
        self._metres_per_edge = self.get_parameter('metres_per_edge').value
        self._window_ms = self.get_parameter('window_ms').value

        self._edge_lock = threading.Lock()
        self._edge_count = 0
        self._sign = 0
        self._chip = None
        self._gpio_callback = None

        # RP1 header bank (verify with gpiodetect: pinctrl-rp1); chip 0 is brcmstb.
        self._chip = lgpio.gpiochip_open(4)
        try:
            lgpio.gpio_claim_alert(
                self._chip, self._gpio_pin, lgpio.BOTH_EDGES
            )
            lgpio.gpio_set_debounce_micros(
                self._chip, self._gpio_pin, GLITCH_FILTER_US
            )
            self._gpio_callback = lgpio.callback(
                self._chip,
                self._gpio_pin,
                lgpio.BOTH_EDGES,
                self._on_edge,
            )
        except Exception:
            lgpio.gpiochip_close(self._chip)
            self._chip = None
            raise

        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.create_subscription(
            Int8, '/motor/direction', self._on_direction, 10
        )
        self.create_timer(self._window_ms / 1000.0, self._publish_window)
        self.get_logger().info(
            f'encoder_node ready on GPIO {self._gpio_pin}'
        )

    def _on_edge(self, chip, gpio, level, tick):
        with self._edge_lock:
            self._edge_count += 1

    def _on_direction(self, msg: Int8):
        self._sign = msg.data

    def _publish_window(self):
        with self._edge_lock:
            edges = self._edge_count
            self._edge_count = 0

        # A single channel gives unsigned speed; take sign directly from the motor
        # FSM. BRAKE is +1, so normal braking stays sign-correct. A pre-stop
        # release/repress into reverse can briefly sign forward coast as reverse:
        # cosmetic below the creep floor while unfused; true low-speed direction
        # requires Phase 1 quadrature on GPIO 23.
        window_s = self._window_ms / 1000.0
        window_speed = edges * self._metres_per_edge / window_s
        signed_speed = window_speed * self._sign

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = LARGE_VARIANCE
        msg.twist.twist.linear.x = signed_speed
        msg.twist.covariance[0] = 0.01
        for index in (7, 14, 21, 28, 35):
            msg.twist.covariance[index] = LARGE_VARIANCE
        self._odom_pub.publish(msg)

    def close_gpio(self):
        if self._gpio_callback is not None:
            self._gpio_callback.cancel()
            self._gpio_callback = None
        if self._chip is not None:
            lgpio.gpiochip_close(self._chip)
            self._chip = None


def main():
    rclpy.init()
    node = None
    try:
        node = EncoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_gpio()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
