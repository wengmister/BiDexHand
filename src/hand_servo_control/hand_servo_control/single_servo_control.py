import rclpy
from rclpy.node import Node
from hand_servo_interfaces.srv import SetServo
import serial


class SingleServoNode(Node):
    def __init__(self):
        super().__init__('single_servo_control')

        # Initialize Serial Communication
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update port as needed

        # Create the Service
        self.srv = self.create_service(SetServo, 'set_single_servo_position', self.handle_set_servo_position)

    def handle_set_servo_position(self, request, response):
        # Validate inputs
        if not (0 <= request.channel < 16) or not (-90 <= request.position <= 90):
            response.success = False
            response.message = 'Invalid channel or position'
            return response

        # Format command: "channel,position\n"
        command = f"{request.channel},{request.position}\n"
        try:
            self.serial_port.write(command.encode())  # Send command to ESP32
            response.success = True
            response.message = 'Command sent successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to send command: {str(e)}'

        return response


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = SingleServoNode()

    try:
        rclpy.spin(servo_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
