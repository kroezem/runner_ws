import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from periphery import PWM

PWM_CHIP   = 0             # verify with: ls /sys/class/pwm/ after adding overlay
                           # change to 2 if pwmchip2 is the RP1 controller on your Pi 5
FRAME_NS   = 20_000_000   # 50 Hz
CMD_TIMEOUT_S = 0.2

NEUTRAL_US = 1500          # ESC neutral — no movement
THR_MAX_US = 2000          # conservative cap for early testing; raise toward 2000 for speed
BRK_MAX_US = 1000          # full brake

STEER_CTR  = 1500          # servo centre
STEER_US   = 500           # ± range around centre

def us_to_ns(us): return int(us * 1000)

class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.esc   = PWM(PWM_CHIP, 0)   # GPIO 12
        self.servo = PWM(PWM_CHIP, 1)   # GPIO 13
        for p in (self.esc, self.servo):
            p.period_ns = FRAME_NS
        self._write(NEUTRAL_US, STEER_CTR)
        self.esc.enable()
        self.servo.enable()
        self._arm_esc()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self._last_cmd = time.monotonic()
        self._cmd_timed_out = False
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info("motor_driver ready")

    def _arm_esc(self):
        self._write(BRK_MAX_US, STEER_CTR)  # low signal to wake ESC
        time.sleep(1.0)
        self._write(NEUTRAL_US, STEER_CTR)  # back to neutral
        time.sleep(1.0)
        self.get_logger().info("ESC armed")

    def _write(self, thr_us, steer_us):
        self.esc.duty_cycle_ns   = us_to_ns(thr_us)
        self.servo.duty_cycle_ns = us_to_ns(steer_us)

    def on_cmd(self, msg: Twist):
        if self._cmd_timed_out:
            self.get_logger().info("/cmd_vel recovered")
            self._cmd_timed_out = False
        self._last_cmd = time.monotonic()
        cmd = max(-1.0, min(1.0, msg.linear.x))
        if cmd > 0.0:
            # throttle: neutral → THR_MAX_US
            thr_us = int(NEUTRAL_US + cmd * (THR_MAX_US - NEUTRAL_US))
        elif cmd < -0.05:
            # brake: neutral → BRK_MAX_US
            thr_us = int(NEUTRAL_US + cmd * (NEUTRAL_US - BRK_MAX_US))
        else:
            thr_us = NEUTRAL_US

        steer_us = int(STEER_CTR + max(-1.0, min(1.0, msg.angular.z)) * STEER_US)
        self._write(thr_us, steer_us)

    def _watchdog(self):
        if time.monotonic() - self._last_cmd <= CMD_TIMEOUT_S:
            return
        if not self._cmd_timed_out:
            self.get_logger().warn("/cmd_vel watchdog timeout; ESC set to neutral")
            self._cmd_timed_out = True
        self._write(NEUTRAL_US, STEER_CTR)

    def stop(self):
        self._write(NEUTRAL_US, STEER_CTR)
        self.esc.disable()
        self.servo.disable()
        self.esc.close()
        self.servo.close()

def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()
