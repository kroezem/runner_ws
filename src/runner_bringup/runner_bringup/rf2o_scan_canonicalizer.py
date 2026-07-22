"""Canonicalize full-circle LaserScan angular origin for RF2O diagnostics."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Rf2oScanCanonicalizer(Node):
    """Rotate a full scan so array index zero corresponds to minus pi."""

    def __init__(self):
        super().__init__('rf2o_scan_canonicalizer')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_rf2o_test')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback,
            qos_profile_sensor_data)
        self.logged_first_scan = False

    @staticmethod
    def rotate(values, offset):
        """Return a circularly rotated copy without changing element values."""
        return values[offset:] + values[:offset]

    def scan_callback(self, scan):
        count = len(scan.ranges)
        if count < 2:
            self.get_logger().error('Rejecting scan with fewer than two ranges')
            return
        if scan.angle_increment <= 0.0:
            self.get_logger().error('Rejecting non-positive angle_increment')
            return
        if scan.intensities and len(scan.intensities) != count:
            self.get_logger().error(
                'Rejecting scan: intensities and ranges lengths differ')
            return

        full_circle = 2.0 * math.pi
        fov = abs(scan.angle_max - scan.angle_min)
        if abs(fov - full_circle) > 1.5 * scan.angle_increment:
            self.get_logger().error(
                f'Rejecting non-full-circle scan with FOV {fov:.9f} rad')
            return

        offset = round((math.pi - scan.angle_min) / scan.angle_increment)
        offset %= count

        output = LaserScan()
        output.header = scan.header
        output.angle_min = -math.pi
        output.angle_increment = scan.angle_increment
        output.angle_max = output.angle_min + (count - 1) * output.angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = scan.range_min
        output.range_max = scan.range_max
        output.ranges = self.rotate(scan.ranges, offset)
        if scan.intensities:
            output.intensities = self.rotate(scan.intensities, offset)

        if not self.logged_first_scan:
            indices = (0, count // 2, count - 1)
            input_bearings = [
                scan.angle_min + index * scan.angle_increment
                for index in indices
            ]
            rf2o_bearings = [
                -0.5 * fov + index * fov / (count - 1)
                for index in indices
            ]
            endpoint_closure_error = count * scan.angle_increment - full_circle
            self.get_logger().info(
                'First scan: N=%d angle_min=%.9f angle_max=%.9f '
                'angle_increment=%.9f rotation_offset=%d indices=%s '
                'input_bearings=%s rf2o_assumed_bearings=%s '
                'endpoint_closure_error=%.9g' % (
                    count, scan.angle_min, scan.angle_max,
                    scan.angle_increment, offset, indices,
                    [round(value, 9) for value in input_bearings],
                    [round(value, 9) for value in rf2o_bearings],
                    endpoint_closure_error))
            if abs((count - 1) * scan.angle_increment - full_circle) \
                    <= 1.5 * scan.angle_increment:
                self.get_logger().warning(
                    'Input uses the LD19 endpoint-inclusive 0..2pi convention; '
                    'a lossless rotation has a one-bin discontinuity at the seam')
            self.logged_first_scan = True

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Rf2oScanCanonicalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
