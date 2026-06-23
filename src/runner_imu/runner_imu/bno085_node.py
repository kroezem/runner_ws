import time
import serial
import gpiod
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART

UART_PORT   = "/dev/ttyAMA2"
UART_BAUD   = 3_000_000
RESET_GPIO  = 26

class Bno085Node(Node):
    def __init__(self):
        super().__init__("bno085")
        self.pub = self.create_publisher(Imu, "/imu/data", 50)
        self._hardware_reset()
        uart = serial.Serial(UART_PORT, baudrate=UART_BAUD, timeout=0.01)
        self.bno = BNO08X_UART(uart)
        for f in (adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR,
                  adafruit_bno08x.BNO_REPORT_GYROSCOPE,
                  adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION):
            self.bno.enable_feature(f)
        time.sleep(0.1)
        self.create_timer(0.02, self.tick)  # 50 Hz
        self.get_logger().info("BNO085 ready on UART")

    def _hardware_reset(self):
        chip = gpiod.Chip("gpiochip0")
        line = chip.get_line(RESET_GPIO)
        line.request(consumer="bno_rst", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        time.sleep(0.02)
        line.set_value(1)
        time.sleep(0.06)

    def tick(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "imu_link"
        x, y, z, w = self.bno.quaternion
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = x, y, z, w
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = self.bno.gyro
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
            self.bno.linear_acceleration
        self.pub.publish(m)

def main():
    rclpy.init(); rclpy.spin(Bno085Node()); rclpy.shutdown()