import rclpy
from rclpy.node import Node
import socket
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from rcl_interfaces.msg import ParameterDescriptor

class HandAnglePublisher(Node):
    def __init__(self):
        super().__init__('hand_angle_publisher')
        
        # Define the desired order for joint angles
        self.order = [
            ('Hand_Thumb0', 'Flexion'),
            ('Hand_Thumb1', 'Flexion'),
            ('Hand_Thumb2', 'Flexion'),
            ('Hand_Thumb3', 'Flexion'),
            ('Hand_Index1', 'Flexion'),
            ('Hand_Index1', 'Abduction'),
            ('Hand_Index2', 'Flexion'),
            ('Hand_Middle1', 'Flexion'),
            ('Hand_Middle1', 'Abduction'),
            ('Hand_Middle2', 'Flexion'),
            ('Hand_Ring1', 'Flexion'),
            ('Hand_Ring1', 'Abduction'),
            ('Hand_Ring2', 'Flexion'),
            ('Hand_Pinky1', 'Flexion'),
            ('Hand_Pinky1', 'Abduction'),
            ('Hand_Pinky2', 'Flexion')
        ]
        
        # Create a parameter to select which hand to track
        self.declare_parameter(
            'hand', 
            'right', 
            ParameterDescriptor(description='Which hand to track: "left" or "right"')
        )
        
        # Setup publisher for hand joint angles
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_joint_angles', 10)
        
        # Store the latest data for both hands as lists
        self.left_hand_data = []
        self.right_hand_data = []
        
        # Setup UDP socket
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 9000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False)
        
        # Create timer to check for UDP messages
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        
        self.get_logger().info('Hand Angle Publisher has started')
        
    def timer_callback(self):
        try:
            # Process all available messages
            while True:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    message = data.decode('utf-8')
                    self.process_message(message)
                    self.publish_hand_angles()
                except BlockingIOError:
                    # No more messages available
                    break
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
    
    def process_message(self, message):
        """
        Processes the incoming UDP message in the format:
        "Right hand:, 24.1, -38.3, 37.9, 3.3, ... "
        or
        "Left hand:, 24.1, -38.3, 37.9, 3.3, ... "
        """
        if "Left hand:" in message:
            hand_type = "left"
        elif "Right hand:" in message:
            hand_type = "right"
        else:
            self.get_logger().error("Message does not contain a valid hand identifier")
            return
        
        # Remove the hand identifier and split the remaining string by commas
        message_clean = message.replace("Left hand:", "").replace("Right hand:", "")
        parts = [part.strip() for part in message_clean.split(",") if part.strip() != ""]
        
        try:
            angles = [float(value) for value in parts]
        except ValueError:
            self.get_logger().error("Error converting message parts to float.")
            return
        
        if len(angles) != len(self.order):
            self.get_logger().error(f"Expected {len(self.order)} angles, but got {len(angles)}.")
            return
        
        if hand_type == "left":
            self.left_hand_data = angles
        else:
            self.right_hand_data = angles
        
        self.get_logger().debug(f'Processed {hand_type} hand data')
    
    def map_received_data_to_order(self, hand_data):
        # In the new input format the data is already in the desired order,
        # so we simply return it.
        return hand_data
    
    def publish_hand_angles(self):
        # Select which hand data to publish based on the parameter value
        hand_param = self.get_parameter('hand').get_parameter_value().string_value.lower()
        
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "hand_joint_angles"
        dim.size = len(self.order)
        dim.stride = len(self.order)
        msg.layout.dim = [dim]
        
        if hand_param == 'left' and self.left_hand_data:
            msg.data = self.map_received_data_to_order(self.left_hand_data)
        elif hand_param == 'right' and self.right_hand_data:
            msg.data = self.map_received_data_to_order(self.right_hand_data)
        else:
            return  # Not enough data to publish
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
