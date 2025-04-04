import rclpy
from rclpy.node import Node

from hand_servo_interfaces.srv import ServoCommands
from std_msgs.msg import Float32MultiArray
import serial

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        
        # Declare parameters
        self.declare_parameter('usb_port', '/dev/ttyUSB1')
        self.declare_parameter('mode', 'cli')
        
        # Retrieve parameter values
        usb_port = self.get_parameter('usb_port').value
        self.mode = self.get_parameter('mode').value
        
        # Attempt to open the serial connection using the provided USB port
        try:
            self.serial = serial.Serial(usb_port, 115200, timeout=1)
            self.get_logger().info(f'Connected to USB port: {usb_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to USB port {usb_port}: {e}')
            rclpy.shutdown()
            return
        
        # Setup depending on mode
        if self.mode == 'cli':
            # Create the ROS service for CLI mode
            self.srv = self.create_service(
                ServoCommands,
                'set_servo_positions',
                self.handle_set_servo_positions
            )
            self.get_logger().info('CLI mode: service /set_servo_positions active.')
        
        elif self.mode == 'streaming':
            # Create subscription to /hand_servo_input for streaming mode
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/hand_servo_input',
                self.servo_input_callback,
                10
            )
            self.get_logger().info('Streaming mode: subscribed to /hand_servo_input.')
        
        self.get_logger().info('Servo control node initialized.')

    def handle_set_servo_positions(self, request, response):
        """Service callback for setting servo positions in CLI mode."""
        if len(request.channels) != len(request.positions):
            response.success = False
            self.get_logger().error('Mismatched channel and position arrays')
            return response
        
        commands = []
        for ch, pos in zip(request.channels, request.positions):
            commands.append(f"{ch},{pos}")
        command_string = ';'.join(commands) + '\n'
        
        try:
            self.serial.write(command_string.encode())
            # self.get_logger().info(f'Sent command: {command_string.strip()}')
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            response.success = False
        
        return response

    def servo_input_callback(self, msg):
        """Subscriber callback for streaming servo positions."""
        # Expect exactly 16 joint angles
        if len(msg.data) != 16:
            self.get_logger().error(
                f"Received {len(msg.data)} angles, but expected 16."
            )
            return
        
        # Channel from 0-15
        commands = []
        for i, pos in enumerate(msg.data):
            commands.append(f"{i},{pos}")
        command_string = ';'.join(commands) + '\n'
        
        try:
            self.serial.write(command_string.encode())
            # self.get_logger().info(f'Sent command: {command_string.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
