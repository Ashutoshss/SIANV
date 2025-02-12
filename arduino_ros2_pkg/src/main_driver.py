import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Replace with your Arduino's port

    def cmd_vel_callback(self, msg):
        # Convert linear and angular velocity to motor speeds
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        wheel_separation = 0.2  # Distance between wheels (meters)
        max_pwm = 255           # Max PWM value for the motor

        left_speed = int((linear_x - angular_z * wheel_separation / 2) * max_pwm)
        right_speed = int((linear_x + angular_z * wheel_separation / 2) * max_pwm)

        # Clamp motor speeds
        left_speed = max(-max_pwm, min(max_pwm, left_speed))
        right_speed = max(-max_pwm, min(max_pwm, right_speed))

        # Send speed command to Arduino
        command = f"V {left_speed} {right_speed}\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

