import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import mediapipe as mp
import cv2
import numpy as np
import transforms3d as tf3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Empty

class WristTrackerNode(Node):
    def __init__(self):
        super().__init__('wrist_tracker_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/wrist_twist', 10)

        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands.Hands(min_detection_confidence=0.8, 
                                                 min_tracking_confidence=0.8)
        self.mp_drawing = mp.solutions.drawing_utils

        self.initial_position = None
        self.initial_orientation = None
        self.scale_factor = 1.0
        self.angular_scale_factor = 1.0
        
        # Threshold (in normalized coordinates) below which we publish zero twist
        self.threshold_distance = 0.05

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        
        self.reset_service = self.create_service(Empty, 'reset', self.reset_callback)
        
    def get_orientation(self, hand_landmarks):
        """
        Compute the orientation angle of the hand using the wrist and index finger MCP.
        This angle (in radians) can be used to determine the relative rotation.
        """
        wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
        index_mcp = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP]
        dx = index_mcp.x - wrist.x
        dy = index_mcp.y - wrist.y
        angle = np.arctan2(dy, dx)
        return angle

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(image_rgb)

        twist_text = ""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        # Filter to process only the right hand
        right_hand_landmarks = None
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                if handedness.classification[0].label == "Left":
                    right_hand_landmarks = hand_landmarks
                    break

        if right_hand_landmarks is not None:
            wrist_landmark = right_hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
            wrist_coords = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])
            current_orientation = self.get_orientation(right_hand_landmarks)

            if self.initial_position is None:
                self.initial_position = wrist_coords
                self.initial_orientation = current_orientation

            # Compute linear (position) delta
            wrist_delta = (wrist_coords - self.initial_position) * self.scale_factor

            # Compute angular delta (assuming rotation around the z-axis)
            angular_delta = (current_orientation - self.initial_orientation) * self.angular_scale_factor

            # If the hand is very close to the initial position, publish zero twist
            if np.linalg.norm(wrist_coords - self.initial_position) < self.threshold_distance:
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                twist.twist.angular.z = 0.0
                twist_text = "Within threshold: Zero twist"
            else:
                twist.twist.linear.x = wrist_delta[0]
                twist.twist.linear.y = wrist_delta[1]
                # Apply scaling to z if required (note the multiplication factor)
                twist.twist.linear.z = wrist_delta[2] * 10000000  
                twist.twist.angular.z = angular_delta
                twist_text = (
                    f"Linear: {twist.twist.linear.x:.2f}, {twist.twist.linear.y:.2f}, {twist.twist.linear.z:.2f} "
                    f"\nAngular z: {angular_delta:.2f}"
                )

            # Draw hand landmarks and wrist position
            self.mp_drawing.draw_landmarks(image, right_hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
            h, w, _ = image.shape
            wrist_px = (int(wrist_landmark.x * w), int(wrist_landmark.y * h))
            cv2.circle(image, wrist_px, 10, (0, 255, 0), -1)
            
            self.publisher_.publish(twist)
        else:
            twist_text = "No right hand detected!"
            # publish zero twist if no right hand is detected
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.z = 0.0
            self.publisher_.publish(twist)

        # Draw initial position marker if available
        if self.initial_position is not None:
            h, w, _ = image.shape
            init_px = (int(self.initial_position[0] * w), int(self.initial_position[1] * h))
            cv2.circle(image, init_px, 10, (0, 0, 255), -1)

        # Enhanced text visualization: use HERSHEY_COMPLEX with an outline for better visibility
        font = cv2.FONT_HERSHEY_COMPLEX
        # Draw text outline (black)
        cv2.putText(image, twist_text, (10, 40), font, 1, (0, 0, 0), 3, cv2.LINE_AA)
        # Draw text (white)
        cv2.putText(image, twist_text, (10, 40), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("Wrist Tracker Debug", image)
        cv2.waitKey(1)

    def reset_callback(self, request, response):
        self.initial_position = None
        self.initial_orientation = None
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WristTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
