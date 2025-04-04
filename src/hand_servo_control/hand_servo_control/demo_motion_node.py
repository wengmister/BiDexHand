import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import std_srvs.srv
import time

"""
Demo motion sequencer to generate trajectories for hand servos.
"""

class DemoMotionNode(Node):
    def __init__(self):
        super().__init__('demo_motion_node')
        
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_servo_input', 10)
        
        self.create_service(std_srvs.srv.Empty, 'flex_extend', self.flex_extend_callback)
        self.create_service(std_srvs.srv.Empty, 'home', self.home_callback)
        self.create_service(std_srvs.srv.Empty, 'thumb_up', self.thumb_up_callback)
        self.create_service(std_srvs.srv.Empty, 'v_sign', self.v_sign_callback)
        self.create_service(std_srvs.srv.Empty, 'ok', self.ok_callback)
        self.create_service(std_srvs.srv.Empty, 'all_extend', self.all_extend_callback)
        self.create_service(std_srvs.srv.Empty, 'all_flex', self.all_flex_callback)
        self.create_service(std_srvs.srv.Empty, 'grip', self.grip_callback)
        self.create_service(std_srvs.srv.Empty, 'thumb_sequence', self.thumb_sequence_callback)

        # Initialize joint angle array. Modify this object to change joint angles.
        self.joint_angle_array = Float32MultiArray()
        self.joint_angle_array.data = [0.0]*16

        self.get_logger().info('Demo motion node initialized.')

    def home_callback(self, request, response):
        """
        Home all hand joint angles.
        """
        self.joint_angle_array.data = [0.0]*16
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Homed all joints.')
        return response

    def flex_extend_callback(self, request, response):
        """
        Flex and extend hand joints with overlapped homing.
        Joint pairs (low, high):
        Pair 1: (9, 14)
        Pair 2: (8, 13)
        Pair 3: (7, 12)
        Pair 4: (6, 11)
        
        Sequence:
        -- Closing Phase --
        t = 0.0 sec: Close Pair 1 (set to closed values: low = -60, high = -90)
        t = 0.2 sec: Close Pair 2
        t = 0.4 sec: Close Pair 3
        t = 0.6 sec: Close Pair 4
        
        -- Overlapped Extension/Homing Phase --
        t = 0.8 sec: Open Pair 3 (set to open values: low = 60, high = 90)
                    and Home Pair 1 (set to 0)
        t = 1.0 sec: Open Pair 4 and Home Pair 2
        t = 1.2 sec: Home Pair 3
        t = 1.4 sec: Home Pair 4
        """
        # Define joint pairs.
        pairs = [(9, 14), (8, 13), (7, 12), (6, 11), (0, 10)]
        
        # Define values
        closed_low_value  = 60.0
        closed_high_value = 90.0
        open_low_value    = -60.0
        open_high_value   = -90.0
        closed_thumb_value = -50.0
        open_thumb_value   = 50.0
        abd_left_value    = 30.0
        abd_right_value   = -30.0
        home_value        = 0.0

        delay_between     = 0.2  # seconds

        self.get_logger().info("Starting flex/extend with overlapped homing sequence...")

        # -- Closing Phase --
        # t = 0.0: Close Pair 1.
        self.joint_angle_array.data[pairs[0][0]] = closed_low_value
        self.joint_angle_array.data[pairs[0][1]] = closed_high_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        # t = 0.2: Close Pair 2.
        self.joint_angle_array.data[pairs[1][0]] = closed_low_value
        self.joint_angle_array.data[pairs[1][1]] = closed_high_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        # t = 0.4: Close Pair 3.
        self.joint_angle_array.data[pairs[2][0]] = closed_low_value
        self.joint_angle_array.data[pairs[2][1]] = closed_high_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[3][0]] = closed_low_value
        self.joint_angle_array.data[pairs[3][1]] = closed_high_value
        self.joint_angle_array.data[pairs[0][0]] = open_low_value    # Open Pair 1 low
        self.joint_angle_array.data[pairs[0][1]] = open_high_value   # Open Pair 1 high
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[4][0]] = closed_thumb_value   
        self.joint_angle_array.data[pairs[4][1]] = closed_thumb_value   
        self.joint_angle_array.data[pairs[1][0]] = open_low_value    # Open Pair 2 low
        self.joint_angle_array.data[pairs[1][1]] = open_high_value   # Open Pair 2 high
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[2][0]] = open_low_value    # Open Pair 3 low
        self.joint_angle_array.data[pairs[2][1]] = open_high_value   # Open Pair 3 high
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[3][0]] = open_low_value    # Open Pair 4 low
        self.joint_angle_array.data[pairs[3][1]] = open_high_value   # Open Pair 4 high
        self.joint_angle_array.data[pairs[0][0]] = home_value         # Home Pair 1 low
        self.joint_angle_array.data[pairs[0][1]] = home_value         # Home Pair 1 high
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[4][0]] = open_thumb_value
        self.joint_angle_array.data[pairs[4][1]] = open_thumb_value
        self.joint_angle_array.data[pairs[1][0]] = home_value         # Home Pair 2 low
        self.joint_angle_array.data[pairs[1][1]] = home_value         # Home Pair 2 high
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        # t = 1.2: Home Pair 3.
        self.joint_angle_array.data[pairs[2][0]] = home_value
        self.joint_angle_array.data[pairs[2][1]] = home_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        # t = 1.4: Home Pair 4.
        self.joint_angle_array.data[pairs[3][0]] = home_value
        self.joint_angle_array.data[pairs[3][1]] = home_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[pairs[4][0]] = home_value
        self.joint_angle_array.data[pairs[4][1]] = home_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[1] = abd_left_value
        self.joint_angle_array.data[2] = abd_left_value
        self.joint_angle_array.data[3] = abd_left_value
        self.joint_angle_array.data[4] = abd_left_value
        self.joint_angle_array.data[5] = abd_left_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[1] = abd_right_value
        self.joint_angle_array.data[2] = abd_right_value
        self.joint_angle_array.data[3] = abd_right_value
        self.joint_angle_array.data[4] = abd_right_value
        self.joint_angle_array.data[5] = abd_right_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.joint_angle_array.data[1] = home_value
        self.joint_angle_array.data[2] = home_value
        self.joint_angle_array.data[3] = home_value
        self.joint_angle_array.data[4] = home_value
        self.joint_angle_array.data[5] = home_value
        self.publisher.publish(self.joint_angle_array)
        time.sleep(delay_between)

        self.get_logger().info("Overlapped flex/extend with homing motion complete.")
        return response
    
    def thumb_up_callback(self, request, response):
        """
        Thumb up motion.
        """
        
        self.joint_angle_array.data[0] = -90.0
        self.joint_angle_array.data[1] = 0.0
        self.joint_angle_array.data[2] = 0.0
        self.joint_angle_array.data[3] = 0.0
        self.joint_angle_array.data[4] = 0.0
        self.joint_angle_array.data[5] = 0.0
        self.joint_angle_array.data[6] = 60.0
        self.joint_angle_array.data[7] = 60.0
        self.joint_angle_array.data[8] = 60.0
        self.joint_angle_array.data[9] = 60.0
        self.joint_angle_array.data[10] = -70.0
        self.joint_angle_array.data[11] = 90.0
        self.joint_angle_array.data[12] = 90.0
        self.joint_angle_array.data[13] = 90.0
        self.joint_angle_array.data[14] = 90.0
        self.joint_angle_array.data[15] = 0.0
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Thumb up.')
        return response
    
    def v_sign_callback(self, request, response):
        """
        V sign motion.
        """
        
        self.joint_angle_array.data[0] = 40.0
        self.joint_angle_array.data[1] = 0.0
        self.joint_angle_array.data[2] = 0.0
        self.joint_angle_array.data[3] = 0.0
        self.joint_angle_array.data[4] = 0.0
        self.joint_angle_array.data[5] = 0.0
        self.joint_angle_array.data[6] = -20.0
        self.joint_angle_array.data[7] = -20.0
        self.joint_angle_array.data[8] = 60.0
        self.joint_angle_array.data[9] = 60.0
        self.joint_angle_array.data[10] = -50.0
        self.joint_angle_array.data[11] = -50.0
        self.joint_angle_array.data[12] = -50.0
        self.joint_angle_array.data[13] = 90.0
        self.joint_angle_array.data[14] = 90.0
        self.joint_angle_array.data[15] = 0.0
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('V sign.')
        return response
    
    def ok_callback(self, request, response):
        """
        OK sign motion.
        """
        self.joint_angle_array.data[0] = -20.0
        self.joint_angle_array.data[1] = 0.0
        self.joint_angle_array.data[2] = 0.0
        self.joint_angle_array.data[3] = 0.0
        self.joint_angle_array.data[4] = 0.0
        self.joint_angle_array.data[5] = 50.0
        self.joint_angle_array.data[6] = 50.0
        self.joint_angle_array.data[7] = -10.0
        self.joint_angle_array.data[8] = -10.0
        self.joint_angle_array.data[9] = -10.0
        self.joint_angle_array.data[10] = -60.0
        self.joint_angle_array.data[11] = 40.0
        self.joint_angle_array.data[12] = -50.0
        self.joint_angle_array.data[13] = -50.0
        self.joint_angle_array.data[14] = -50.0
        self.joint_angle_array.data[15] = -40.0
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('OK sign.')
        return response
    
    def all_extend_callback(self, request, response):
        """
        Extend all hand joints.
        """
        low_open = -60.0
        high_open = -90.0
        # open low: 6-9, open high: 11-14
        self.joint_angle_array.data[6] = low_open
        self.joint_angle_array.data[7] = low_open
        self.joint_angle_array.data[8] = low_open
        self.joint_angle_array.data[9] = low_open

        self.joint_angle_array.data[11] = high_open
        self.joint_angle_array.data[12] = high_open
        self.joint_angle_array.data[13] = high_open
        self.joint_angle_array.data[14] = high_open
        self.joint_angle_array.data[5] = -40.0
        self.joint_angle_array.data[15] = 0.0
        self.joint_angle_array.data[10] = -60.0
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Extended all joints.')
        return response
    
    def all_flex_callback(self, request, response):
        """
        Flex all hand joints.
        """
        low_close = 60.0
        high_close = 90.0
        # close low: 6-9, close high: 11-14
        self.joint_angle_array.data[6] = low_close
        self.joint_angle_array.data[7] = low_close
        self.joint_angle_array.data[8] = low_close
        self.joint_angle_array.data[9] = low_close

        self.joint_angle_array.data[11] = high_close
        self.joint_angle_array.data[12] = high_close
        self.joint_angle_array.data[13] = high_close
        self.joint_angle_array.data[14] = high_close
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Flexed all joints.')
        return response

    def grip_callback(self, request, response):
        """
        Grip demo motion
        """
        grip_low = 65.0
        grip_high = 15.0

        # grip low: 6-9, grip high: 11-14
        self.joint_angle_array.data[6] = grip_low
        self.joint_angle_array.data[7] = grip_low
        self.joint_angle_array.data[8] = grip_low
        self.joint_angle_array.data[9] = grip_low

        self.joint_angle_array.data[11] = grip_high
        self.joint_angle_array.data[12] = grip_high
        self.joint_angle_array.data[13] = grip_high
        self.joint_angle_array.data[14] = grip_high
        self.joint_angle_array.data[15] = -60.0
        self.joint_angle_array.data[10] = -40.0
        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Grip.')
        return response
    
    def thumb_sequence_callback(self, request, response):
        """Demonstrate thumb range of motion."""

        # reset all joints
        self.joint_angle_array.data = [0.0]*16
        self.publisher.publish(self.joint_angle_array)
        
        time.sleep(0.5)

        # pinky
        self.joint_angle_array.data[0] = 20.0
        self.joint_angle_array.data[5] = -30.0
        self.joint_angle_array.data[15] = -80.0
        self.publisher.publish(self.joint_angle_array)

        time.sleep(0.5)

        # reset all joints
        self.joint_angle_array.data = [0.0]*16
        self.publisher.publish(self.joint_angle_array)
        time.sleep(0.5)

        # ring
        self.joint_angle_array.data[0] = 40.0
        self.joint_angle_array.data[5] = 10.0
        self.joint_angle_array.data[10] = -40.0
        self.joint_angle_array.data[15] = -60.0
        self.publisher.publish(self.joint_angle_array)

        time.sleep(0.5)

        # reset all joints
        self.joint_angle_array.data = [0.0]*16
        self.publisher.publish(self.joint_angle_array)
        time.sleep(0.5)

        # middle
        self.joint_angle_array.data[0] = 40.0
        self.joint_angle_array.data[5] = 50.0
        self.joint_angle_array.data[10] = -80.0
        self.joint_angle_array.data[15] = -40.0
        self.publisher.publish(self.joint_angle_array)

        time.sleep(0.5)

        # reset all joints
        self.joint_angle_array.data = [0.0]*16
        self.publisher.publish(self.joint_angle_array)
        time.sleep(0.5)

        # index
        self.joint_angle_array.data[0] = -10.0
        self.joint_angle_array.data[5] = 60.0
        self.joint_angle_array.data[10] = -50.0
        self.joint_angle_array.data[15] = 0.0
        self.publisher.publish(self.joint_angle_array)

        time.sleep(0.5)

        # reset
        self.joint_angle_array.data = [0.0]*16

        self.publisher.publish(self.joint_angle_array)
        self.get_logger().info('Thumb sequence.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DemoMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
