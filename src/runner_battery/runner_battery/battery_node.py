import fcntl
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


I2C_SLAVE = 0x0703
I2C_DEVICE = '/dev/i2c-1'
I2C_ADDRESS = 0x36


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.publisher = self.create_publisher(BatteryState, '/battery', 10)
        self.i2c = os.open(I2C_DEVICE, os.O_RDWR)
        fcntl.ioctl(self.i2c, I2C_SLAVE, I2C_ADDRESS)
        self.timer = self.create_timer(1.0, self.publish_battery)

    def destroy_node(self):
        os.close(self.i2c)
        super().destroy_node()

    def read_register(self, register):
        os.write(self.i2c, bytes([register]))
        data = os.read(self.i2c, 2)
        if len(data) != 2:
            raise OSError(f'short I2C read from register 0x{register:02x}')
        return data

    def publish_battery(self):
        try:
            vcell = self.read_register(0x02)
            soc = self.read_register(0x04)
        except OSError as error:
            self.get_logger().error(f'fuel gauge read failed: {error}')
            return

        raw_voltage = ((vcell[0] << 8) | vcell[1]) >> 4
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.voltage = raw_voltage * 0.00125
        message.current = math.nan
        message.charge = math.nan
        message.percentage = min(soc[0], 100) / 100.0
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
