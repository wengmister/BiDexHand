import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CalibratedControlNode(Node):
    def __init__(self):
        super().__init__('calibrated_control_node')
        
        # Declare parameters
        self.declare_parameter('calibration_file', 'servo_calibration.yaml')
        
        # Retrieve parameter values
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # Load calibration data
        self.calibration = self.load_calibration(self.calibration_file)
        if not self.calibration:
            self.get_logger().error(f'Failed to load calibration file: {self.calibration_file}')
            rclpy.shutdown()
            return
            
        # Create publisher for raw servo positions
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/hand_servo_input',  # This topic expects raw values
            10
        )
        
        # Create subscription to receive calibrated positions
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/calibrated_servo_input',  # This topic provides calibrated values
            self.calibrated_input_callback,
            10
        )
        
        self.get_logger().info('\033[92mCalibrated control node initialized.\033[0m')
        self.get_logger().debug(f'\033[94mLoaded calibration from: {self.calibration_file}\033[0m')
    
    def load_calibration(self, filename):
        """Load calibration data from YAML file."""
        try:
            # Try to find the file in the package share directory first
            try:
                package_share_dir = get_package_share_directory('hand_servo_control')
                config_path = os.path.join(package_share_dir, 'config', filename)
                if not os.path.exists(config_path):
                    # Fall back to absolute path
                    config_path = filename
            except Exception:
                # If package not found, use the filename directly
                config_path = filename
                
            with open(config_path, 'r') as file:
                calibration_data = yaml.safe_load(file)
                
            # Validate calibration data structure
            for i in range(16):
                channel_key = f'channel_{i}'
                if channel_key not in calibration_data:
                    self.get_logger().warn(f'Missing calibration for {channel_key}, using defaults (gain=1.0, offset=0.0)')
                    calibration_data[channel_key] = {'gain': 1.0, 'offset': 0.0}
                else:
                    # Ensure gain and offset exist
                    if 'gain' not in calibration_data[channel_key]:
                        self.get_logger().warn(f'Missing gain for {channel_key}, using default (1.0)')
                        calibration_data[channel_key]['gain'] = 1.0
                    if 'offset' not in calibration_data[channel_key]:
                        self.get_logger().warn(f'Missing offset for {channel_key}, using default (0.0)')
                        calibration_data[channel_key]['offset'] = 0.0
                        
            return calibration_data
            
        except Exception as e:
            self.get_logger().error(f'Error loading calibration file: {e}')
            return None
    
    def calibrated_input_callback(self, msg):
        """Convert calibrated positions to raw positions and publish to servo control."""
        # Expect exactly 16 joint angles
        if len(msg.data) != 16:
            self.get_logger().error(
                f"Received {len(msg.data)} angles, but expected 16."
            )
            return
        
        # The original calibration formula was: calibrated_pos = gain * raw_pos + offset
        # To convert from calibrated to raw: raw_pos = (calibrated_pos - offset) / gain
        
        raw_data = []
        for i, calibrated_pos in enumerate(msg.data):
            channel_key = f'channel_{i}'
            gain = self.calibration[channel_key]['gain']
            offset = self.calibration[channel_key]['offset']
            
            # log input calibrated_pos
            self.get_logger().debug(f'Calibrated position for {channel_key}: {calibrated_pos}')

            # Convert calibrated value back to raw value
            if abs(gain) < 1e-6:  # Check for near-zero gain to avoid division by zero
                self.get_logger().error(f'Gain for {channel_key} is too close to zero, cannot invert calibration - using calibrated value')
                raw_pos = calibrated_pos
            else:
                raw_pos = (calibrated_pos - offset) / gain
                
            raw_data.append(raw_pos)

            # log output raw_pos
            self.get_logger().debug(f'Raw position for {channel_key}: {raw_pos}')
        
        # Create message and publish
        output_msg = Float32MultiArray()
        output_msg.data = raw_data
        self.publisher.publish(output_msg)
        
        # Log the conversion for debugging purposes
        self.get_logger().debug(f'Converted calibrated positions to raw positions')

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()