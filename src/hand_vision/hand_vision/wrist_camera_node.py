import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class WristCameraNode(Node):
    def __init__(self):
        super().__init__('wrist_camera_node')
        self.get_logger().info('Wrist camera node started')

        # timer for broadcasting static transform
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Static broadcaster for robot ee frame to camera frame
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)


    def broadcast_wrist_tf(self):
        """Broadcasts static transform from robot wrist to camera frame"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'hand_base_link'
        transform.child_frame_id = 'wrist_link'
        transform.transform.translation.x = 0.009
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0353
        # R = [  0.0000000,  1.0000000,  0.0000000;
            #    0.0000000,  0.0000000,  1.0000000;
            #    1.0000000,  0.0000000,  0.0000000 ]. 
        # Maps to a correct camera orientation
        transform.transform.rotation.x = 0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = 0.5
        transform.transform.rotation.w = -0.5


        self.static_broadcaster.sendTransform(transform)

    def timer_callback(self):
        self.broadcast_wrist_tf()
    
def main(args=None):
    rclpy.init(args=args)
    wrist_camera_node = WristCameraNode()
    rclpy.spin(wrist_camera_node)
    wrist_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
