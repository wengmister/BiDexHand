#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

class JointStateForwardingNode(Node):
    def __init__(self):
        super().__init__('joint_state_forwarding_node')
        
        # Publisher for the fused joint_states message
        self.joint_state_pub = self.create_publisher(JointState, 'hand/joint_states', 10)
        
        # Subscription to the hand servo input topic
        self.hand_sub = self.create_subscription(
            Float32MultiArray,
            '/hand_servo_input',
            self.hand_callback,
            10)

        self.hand_joint_names = [
            "tcf_joint",
            "tca_joint",
            "tma_joint",
            "tmf_joint",
            "tdf_joint",
            "ima_joint",
            "imf_joint",
            "ipf_joint",
            "idf_joint",
            "mma_joint",
            "mmf_joint",
            "mpf_joint",
            "mdf_joint",
            "rma_joint",
            "rmf_joint",
            "rpf_joint",
            "rdf_joint",
            "pma_joint",
            "pmf_joint",
            "ppf_joint",
            "pdf_joint"
        ]

        self.joint_names = self.hand_joint_names
        
        # Initialize hand joint positions (21 values)
        self.hand_joint_positions = [0.0] * 21
        
        # Create a timer to regularly publish the JointState message (e.g., at 50 Hz)
        self.timer = self.create_timer(0.02, self.publish_joint_state)

    def dip_from_pip(self, pip_angle):
        """
        Calculate the dip angle from the proximal interphalangeal (PIP) angle using the antiparallelgram model.

        See derivation at linkage_analysis/antiparallelgram_model.pdf
        """
        neutral_pip = 1.968 #rad
        # neutral_pip = 112.78 #deg
        neutral_dip = 0.3978 #rad
        # neutral_dip = 22.78 #deg
        return 2 * math.atan(0.87 / 2.87 / math.tan((neutral_pip - pip_angle)/2)) - neutral_dip


    def hand_callback(self, msg: Float32MultiArray):
        """
        Callback function for /hand_servo_input.
        1. Converts input angles from degrees to radians
        2. It remaps the 16 input angles to the correct order
        3. It calls a function to populate dip angles based on pip angles with the antiparallelgram model
        """
        
        if len(msg.data) < 16:
            self.get_logger().warn("Received hand servo input with insufficient data")
            return
        
        # Convert input angles from degrees to radians
        msg_in_radians = [math.radians(angle) for angle in msg.data]

        try:
            mapped = [0.0] * 21
            mapped[0]  = msg_in_radians[15]  # tcf_joint = servo.15
            mapped[1]  = msg_in_radians[0]   # tca_joint = servo.0
            mapped[2]  = msg_in_radians[5]   # tma_joint = servo.5
            mapped[3]  = msg_in_radians[10]  # tmf_joint = servo.10

            mapped[5]  = msg_in_radians[1]   # ima_joint = servo.1
            mapped[6]  = msg_in_radians[6]   # imf_joint = servo.6
            mapped[7]  = msg_in_radians[11]  # ipf_joint = servo.11

            mapped[9]  = msg_in_radians[2]   # mma_joint = servo.2
            mapped[10] = msg_in_radians[7]   # mmf_joint = servo.7
            mapped[11] = msg_in_radians[12]  # mpf_joint = servo.12

            mapped[13] = msg_in_radians[3]   # rma_joint = servo.3
            mapped[14] = msg_in_radians[8]   # rmf_joint = servo.8
            mapped[15] = msg_in_radians[13]  # rpf_joint = servo.13

            mapped[17] = msg_in_radians[4]   # pma_joint = servo.4
            mapped[18] = msg_in_radians[9]   # pmf_joint = servo.9
            mapped[19] = msg_in_radians[14]  # ppf_joint = servo.14


            # SIMULATION PIP NEEDS TO BE UNCOMPENSATED:
            mapped[7] = mapped[7] - mapped[6] * 0.7
            mapped[11] = mapped[11] - mapped[10] * 0.7
            mapped[15] = mapped[15] - mapped[14] * 0.7
            mapped[19] = mapped[19] - mapped[18] * 0.7 
            mapped[3] = mapped[3] + mapped[2] *0.7 + 1

            mapped[4]  = self.dip_from_pip(mapped[3])   # tdf_joint = dip_from_pip(servo.10)
            mapped[8]  = self.dip_from_pip(mapped[7])   # idf_joint = dip_from_pip(servo.11)
            mapped[12] = self.dip_from_pip(mapped[11])   # mdf_joint = dip_from_pip(servo.12)
            mapped[16] = self.dip_from_pip(mapped[15])   # rdf_joint = dip_from_pip(servo.13)
            mapped[20] = self.dip_from_pip(mapped[19])   # pdf_joint = dip_from_pip(servo.14)

            # temporary pip flexion multiplier: pending servo mapping calibration
            # all pip flexions gets 0.7x multiplier
            mapped[3] = mapped[3]*0.7
            mapped[7] = mapped[7]*0.7
            mapped[11] = mapped[11]*0.7
            mapped[15] = mapped[15]*0.7
            mapped[19] = mapped[19]*0.7
            mapped[1] = mapped[1]*0.5

        except IndexError:
            self.get_logger().error("Input data does not contain the expected indices")
            return
        except Exception as e:
            self.get_logger().error(f"Error while remapping hand servo input: {e}")
            return
        
        # Update the hand joint positions with the mapped values.
        self.hand_joint_positions = mapped

    def publish_joint_state(self):
        """
        Publish a JointState message that fuses the arm and hand joint data.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ""
        
        # The full positions list is formed by the static arm positions plus the updated hand positions.
        joint_state.name = self.joint_names
        joint_state.position = self.hand_joint_positions

        # Leaving velocity and effort empty for now
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_state_pub.publish(joint_state)
        self.get_logger().debug("Published joint state message")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateForwardingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
