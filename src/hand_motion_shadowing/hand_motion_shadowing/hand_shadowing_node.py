import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class HandShadowingNode(Node):
    def __init__(self):
        super().__init__('hand_shadowing_node')

        # --- Load Basic Parameters ---
        self.declare_parameter('scale_factor', 2.0)
        self.scale_factor = self.get_parameter('scale_factor').value

        self.declare_parameter('smoothing_factor', 0.3)
        self.smoothing_factor = self.get_parameter('smoothing_factor').value

        # --- Load Scaling Parameters (YAML config) ---
        self.declare_parameter('scaling.thumb_cmc_abd', 1.0)       # output index 0
        self.declare_parameter('scaling.thumb_mcp_abd', 1.0)         # output index 5
        self.declare_parameter('scaling.finger_mcp_abd', 1.0)        # output indices 1–4
        self.declare_parameter('scaling.finger_mcp_flex', 1.0)       # output indices 6–9
        self.declare_parameter('scaling.thumb_mcp_flex', 1.0)        # output index 10
        self.declare_parameter('scaling.finger_pip', 1.0)            # output indices 11–14
        self.declare_parameter('scaling.thumb_cmc_flex', 1.0)        # output index 15

        # --- Load Compensation Parameters (YAML config) ---
        self.declare_parameter('compensation.thumb_cmc_abd', 40.0)   # output index 0
        self.declare_parameter('compensation.thumb_mcp_abd', -35.0)    # output index 5
        self.declare_parameter('compensation.index_mcp_abd', -2.0)     # output index 1
        self.declare_parameter('compensation.middle_mcp_abd', -2.0)    # output index 2
        self.declare_parameter('compensation.ring_mcp_abd', -2.0)      # output index 3
        self.declare_parameter('compensation.pinky_mcp_abd', -2.0)     # output index 4
        self.declare_parameter('compensation.finger_mcp_flex', 0.0)    # output indices 6–9
        self.declare_parameter('compensation.thumb_mcp_flex', 0.0)     # output index 10
        self.declare_parameter('compensation.finger_pip', -53.0)       # output indices 11–14
        self.declare_parameter('compensation.thumb_cmc_flex', -25.0)   # output index 15

        # --- Load Clamping Parameters (YAML config) ---
        self.declare_parameter('clamping.thumb_cmc_abd', [-40.0, 40.0])   # output index 0
        self.declare_parameter('clamping.thumb_mcp_abd', [-40.0, 40.0])     # output index 5
        self.declare_parameter('clamping.finger_mcp_abd', [-25.0, 25.0])    # output indices 1–3
        self.declare_parameter('clamping.pinky_mcp_abd', [-40.0, 25.0])     # output index 4
        self.declare_parameter('clamping.finger_mcp_flex', [-90.0, 90.0])   # output indices 6–9
        self.declare_parameter('clamping.thumb_mcp_flex', [-50.0, 90.0])    # output index 10
        self.declare_parameter('clamping.finger_pip', [-50.0, 90.0])        # output indices 11–14
        self.declare_parameter('clamping.thumb_cmc_flex', [-85.0, 90.0])    # output index 15
        self.declare_parameter('clamping.default', [-90.0, 90.0])           # fallback

        # --- Store Scaling Parameters ---
        self.scaling = {
            'thumb_cmc_abd': self.get_parameter('scaling.thumb_cmc_abd').value,
            'thumb_mcp_abd': self.get_parameter('scaling.thumb_mcp_abd').value,
            'finger_mcp_abd': self.get_parameter('scaling.finger_mcp_abd').value,
            'finger_mcp_flex': self.get_parameter('scaling.finger_mcp_flex').value,
            'thumb_mcp_flex': self.get_parameter('scaling.thumb_mcp_flex').value,
            'finger_pip': self.get_parameter('scaling.finger_pip').value,
            'thumb_cmc_flex': self.get_parameter('scaling.thumb_cmc_flex').value,
        }

        # --- Store Compensation Parameters ---
        self.compensation = {
            'thumb_cmc_abd': self.get_parameter('compensation.thumb_cmc_abd').value,
            'thumb_mcp_abd': self.get_parameter('compensation.thumb_mcp_abd').value,
            'index_mcp_abd': self.get_parameter('compensation.index_mcp_abd').value,
            'middle_mcp_abd': self.get_parameter('compensation.middle_mcp_abd').value,
            'ring_mcp_abd': self.get_parameter('compensation.ring_mcp_abd').value,
            'pinky_mcp_abd': self.get_parameter('compensation.pinky_mcp_abd').value,
            'finger_mcp_flex': self.get_parameter('compensation.finger_mcp_flex').value,
            'thumb_mcp_flex': self.get_parameter('compensation.thumb_mcp_flex').value,
            'finger_pip': self.get_parameter('compensation.finger_pip').value,
            'thumb_cmc_flex': self.get_parameter('compensation.thumb_cmc_flex').value,
        }

        # --- Store Clamping Parameters ---
        self.clamping = {
            'thumb_cmc_abd': self.get_parameter('clamping.thumb_cmc_abd').value,
            'thumb_mcp_abd': self.get_parameter('clamping.thumb_mcp_abd').value,
            'finger_mcp_abd': self.get_parameter('clamping.finger_mcp_abd').value,
            'pinky_mcp_abd': self.get_parameter('clamping.pinky_mcp_abd').value,
            'finger_mcp_flex': self.get_parameter('clamping.finger_mcp_flex').value,
            'thumb_mcp_flex': self.get_parameter('clamping.thumb_mcp_flex').value,
            'finger_pip': self.get_parameter('clamping.finger_pip').value,
            'thumb_cmc_flex': self.get_parameter('clamping.thumb_cmc_flex').value,
            'default': self.get_parameter('clamping.default').value,
        }

        # Create subscription to /hand_joint_angles
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_joint_angles',
            self.joint_angles_callback,
            10
        )

        # Create publisher to /hand_servo_input
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_servo_input', 10)

        # Initialize a list to store smoothed angles (one for each servo output: 16 total)
        self.joint_angles = [0.0] * 16

        self.get_logger().info('Hand shadowing node initialized with parameters from config file.') 

    def joint_angles_callback(self, msg):
        """
        Callback function for /hand_joint_angles subscription.
        Maps the input message into 16 output servo angles, then applies
        compensation, scaling, clamping, and (if desired) smoothing before publishing.
        """
        if msg.data:
            # Prepare the output message with 16 angles
            out_msg = Float32MultiArray()
            out_msg.data = [0.0] * 16

            # --- Map from input (16 angles) to output (16 servo angles) ---
            out_msg.data[0]  = msg.data[1]   # thumb_cmc_abd
            out_msg.data[1]  = msg.data[5]   # index_mcp_abd
            out_msg.data[2]  = msg.data[8]   # middle_mcp_abd
            out_msg.data[3]  = msg.data[11]  # ring_mcp_abd
            out_msg.data[4]  = msg.data[14]  # pinky_mcp_abd

            out_msg.data[5]  = msg.data[2]   # thumb_mcp_abd
            out_msg.data[6]  = msg.data[4]   # index_mcp_flex
            out_msg.data[7]  = msg.data[7]   # middle_mcp_flex
            out_msg.data[8]  = msg.data[10]  # ring_mcp_flex
            out_msg.data[9]  = msg.data[13]  # pinky_mcp_flex

            out_msg.data[10] = msg.data[3]   # thumb_mcp_flex
            out_msg.data[11] = msg.data[6]   # index_pip
            out_msg.data[12] = msg.data[9]   # middle_pip
            out_msg.data[13] = msg.data[12]  # ring_pip
            out_msg.data[14] = msg.data[15]  # pinky_pip

            out_msg.data[15] = msg.data[0]   # thumb_cmc_flex

            # --- Apply compensation, scaling, clamping, and conversion ---
            for i in range(16):
                compensated_angle = self.compensate_angle_by_index(out_msg.data[i], i)
                scaled_angle = self.scale_angle_by_index(compensated_angle, i)
                clamped_angle = self.clamp_angle_by_index(scaled_angle, i)
                # (If using relative-to-absolute conversion, adjust here)
                converted_angle = self.convert_rel_to_abs(clamped_angle, i)
                self.joint_angles[i] = converted_angle
                out_msg.data[i] = converted_angle

            # Publish the updated angles
            self.publisher.publish(out_msg)
        else:
            self.get_logger().debug("Received empty joint angle message")

    def scale_angle_by_index(self, angle, index):
        """
        Scale the input angle by the configured scale factors.
        """
        if index == 0:
            return angle * self.scaling['thumb_cmc_abd']
        elif 1 <= index <= 4:
            return angle * self.scaling['finger_mcp_abd']
        elif index == 5:
            return angle * self.scaling['thumb_mcp_abd']
        elif 6 <= index <= 9:
            return angle * self.scaling['finger_mcp_flex']
        elif index == 10:
            return angle * self.scaling['thumb_mcp_flex']
        elif 11 <= index <= 14:
            return angle * self.scaling['finger_pip']
        elif index == 15:
            return angle * self.scaling['thumb_cmc_flex']
        else:
            return angle * self.scale_factor

    def compensate_angle_by_index(self, angle, index):
        """
        Compensate the angle based on the servo index using YAML parameters.
        """
        if index == 0:
            return angle + self.compensation['thumb_cmc_abd']
        elif index == 1:
            return angle + self.compensation['index_mcp_abd']
        elif index == 2:
            return angle + self.compensation['middle_mcp_abd']
        elif index == 3:
            return angle + self.compensation['ring_mcp_abd']
        elif index == 4:
            return angle + self.compensation['pinky_mcp_abd']
        elif index == 5:
            return angle + self.compensation['thumb_mcp_abd']
        elif 6 <= index <= 9:
            return angle + self.compensation['finger_mcp_flex']
        elif index == 10:
            return angle + self.compensation['thumb_mcp_flex']
        elif 11 <= index <= 14:
            return angle + self.compensation['finger_pip']
        elif index == 15:
            return angle + self.compensation['thumb_cmc_flex']
        else:
            return angle

    def clamp_angle_by_index(self, angle, index):
        """
        Clamp the angle based on the servo index using YAML-defined limits.
        """
        if index == 0:
            min_val, max_val = self.clamping['thumb_cmc_abd']
        elif 1 <= index <= 3:
            min_val, max_val = self.clamping['finger_mcp_abd']
        elif index == 4:
            min_val, max_val = self.clamping['pinky_mcp_abd']
        elif index == 5:
            min_val, max_val = self.clamping['thumb_mcp_abd']
        elif 6 <= index <= 9:
            min_val, max_val = self.clamping['finger_mcp_flex']
        elif index == 10:
            min_val, max_val = self.clamping['thumb_mcp_flex']
        elif 11 <= index <= 14:
            min_val, max_val = self.clamping['finger_pip']
        elif index == 15:
            min_val, max_val = self.clamping['thumb_cmc_flex']
        else:
            min_val, max_val = self.clamping['default']
        return max(min_val, min(angle, max_val))

    def convert_rel_to_abs(self, angle, index):
        """
        Convert relative pip angles to absolute angles based on the corresponding MCP flex angle.
        (This is applied only for pip joints.)
        """
        if 11 <= index <= 14:
            # For finger pips, add the corresponding mcp flex angle (output index = pip index - 5)
            return angle + self.joint_angles[index - 5] * 0.8
        elif index == 10:
            # For thumb pip conversion, add thumb mcp angle (output index 5)
            return angle + self.joint_angles[5] * 0.4
        else:
            return angle

def main(args=None):
    rclpy.init(args=args)
    node = HandShadowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
