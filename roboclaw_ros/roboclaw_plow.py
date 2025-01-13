#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus
import roboclaw_ros.roboclaw_driver.roboclaw_driver as Roboclaw
from std_msgs.msg import String

class Movement:
    def __init__(self, address, max_duty):
        self.twist = None
        self.address = address
        self.MAX_DUTY = max_duty
        self.last_set_move_time = rclpy.time.Time()
        self.stopped = True
        self.last_yn0 = None
        self.last_y0 = None
        self.curent_yn0 = None
        self.curent_y0 = None

    def run(self):
        if self.twist is None:
            return
        self.last_set_move_time = rclpy.time.Time()
        if self.twist.linear.x != 0:
            self.stopped = False
            self.lastyn0 = self.curent_yn0
            self.curent_yn0 = rclpy.time.Time()

        linear_x = self.twist.linear.x

        try:
            if linear_x >= 0:
                Roboclaw.BackwardM1(self.address, int(linear_x * self.MAX_DUTY))
            elif linear_x < 0:
                Roboclaw.ForwardM1(self.address, self.MAX_DUTY)
        except OSError as e:
            print(f"DutyM1M2 OSError: {e}")

class RoboclawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_plow')
        
        self.ERRORS = {0x0000: (DiagnosticStatus.OK, "Normal"),
                       0x0001: (DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (DiagnosticStatus.WARN, "M2 over current"),
                       0x0008: (DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (DiagnosticStatus.OK, "M1 home"),
                       0x8000: (DiagnosticStatus.OK, "M2 home")}

        self.declare_parameter("dev", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("address", 130)
        self.declare_parameter("max_duty", 30)
        self.declare_parameter("stop_movement", True)

        self.dev_name = self.get_parameter("dev").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud").get_parameter_value().integer_value
        self.address = self.get_parameter("address").get_parameter_value().integer_value
        self.MAX_DUTY = self.get_parameter("max_duty").get_parameter_value().integer_value
        self.STOP_MOVEMENT = self.get_parameter("stop_movement").get_parameter_value().bool_value

        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            rclpy.shutdown()

        try:
            Roboclaw.Open(self.dev_name, self.baud_rate)
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to Roboclaw: {e}")
            rclpy.shutdown()

        self.movement = Movement(self.address, self.MAX_DUTY)
        self.last_set_move_time = rclpy.time.Time()

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "cmd_vel_plow",
            self.cmd_vel_callback,
            10
        )

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticStatus,
            "/diagnostics",
            10
        )

        self.timer = self.create_timer(1.0, self.run)

    def run(self):
        if self.STOP_MOVEMENT and not self.movement.stopped and (rclpy.time.Time() - self.movement.last_set_move_time).seconds > 1:
            self.get_logger().info("Did not get command for 1 second, stopping")
            try:
                Roboclaw.ForwardM1(self.address, 0)
            except OSError as e:
                self.get_logger().error(f"Could not stop: {e}")
            self.movement.stopped = True

        self.movement.run()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_move_time = rclpy.time.Time()
        self.movement.twist = twist

    def check_vitals(self):
        try:
            status = Roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn(f"Diagnostics OSError: {e}")
            return

        state, message = self.ERRORS.get(status, (DiagnosticStatus.ERROR, "Unknown error"))
        diag_msg = DiagnosticStatus()
        diag_msg.level = state
        diag_msg.message = message
        self.diagnostics_publisher.publish(diag_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down")
        try:
            Roboclaw.ForwardM1(self.address, 0)
            Roboclaw.ForwardM2(self.address, 0)
        except OSError as e:
            self.get_logger().error(f"Shutdown did not work: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoboclawNode()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
