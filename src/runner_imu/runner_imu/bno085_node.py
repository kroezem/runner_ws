import time
import serial
import gpiod
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32
import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART

UART_PORT   = "/dev/ttyAMA2"
UART_BAUD   = 3_000_000
RESET_GPIO  = 26

class Bno085Node(Node):
    def __init__(self):
        super().__init__("bno085")
        self.pub = self.create_publisher(Imu, "/imu/data", 50)
        self.error_pub = self.create_publisher(UInt32, "/imu/read_errors", 10)
        self._consecutive_errors = 0
        self._dropped_reads = 0
        self._hardware_reset()
        self.uart = serial.Serial(UART_PORT, baudrate=UART_BAUD, timeout=10.01)
        self.bno = BNO08X_UART(self.uart)
        self._enable_features()
        time.sleep(0.1)
        self.create_timer(0.02, self.tick)  # 50 Hz
        self.create_timer(1.0, self.publish_read_errors)
        self.get_logger().info("BNO085 ready on UART")

    def _enable_features(self):
        for f in (adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR,
                  adafruit_bno08x.BNO_REPORT_GYROSCOPE,
                  adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION):
            self.bno.enable_feature(f)

    def _hardware_reset(self):
        chip = gpiod.Chip("pinctrl-rp1", gpiod.Chip.OPEN_BY_LABEL)
        line = chip.get_line(RESET_GPIO)
        line.request(consumer="bno_rst", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        time.sleep(0.02)
        line.set_value(1)
        time.sleep(0.06)

    def tick(self):
        try:
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "imu_link"
            x, y, z, w = self.bno.quaternion
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = x, y, z, w
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = self.bno.gyro
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
                self.bno.linear_acceleration
            m.orientation_covariance[8] = 0.005
            m.angular_velocity_covariance[8] = 0.005
            m.linear_acceleration_covariance[0] = 0.1
            m.linear_acceleration_covariance[4] = 0.1
            m.linear_acceleration_covariance[8] = 0.1
            self.pub.publish(m)
            self._consecutive_errors = 0
        except (KeyError, RuntimeError, OSError) as exc:
            self._consecutive_errors += 1
            self._dropped_reads += 1
            self.get_logger().warning(
                f"BNO085 read failed ({type(exc).__name__})"
            )
            if self._consecutive_errors > 10:
                self.get_logger().error("BNO085 read failures exceeded 10; reinitializing")
                self._reinitialize()

    def _reinitialize(self):
        try:
            self.uart.close()
            self._hardware_reset()
            self.uart = serial.Serial(UART_PORT, baudrate=UART_BAUD, timeout=10.01)
            self.bno = BNO08X_UART(self.uart)
            self._enable_features()
            time.sleep(0.1)
        except Exception as exc:
            self.get_logger().error(
                f"BNO085 reinitialization failed ({type(exc).__name__})"
            )

    def publish_read_errors(self):
        self.error_pub.publish(UInt32(data=self._dropped_reads))

def main():
    rclpy.init(); rclpy.spin(Bno085Node()); rclpy.shutdown()
