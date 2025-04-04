import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge

def calculate_angle(a, b, c):
    """
    Calculate the angle between vectors formed by three points a, b, and c.
    """
    ba = np.array(a) - np.array(b)
    bc = np.array(c) - np.array(b)
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))
    return angle

def neutralize_angle(angle):
    """
    Normalize input angle to 0.
    """
    return angle - 180

def project_vector_onto_plane(v, normal):
    """
    Project vector v onto a plane defined by the normal vector.
    """
    return v - np.dot(v, normal) * normal / np.linalg.norm(normal) ** 2

def calculate_mcp_felxion(origin, joint, distal, palm_normal):
    """
    Calculate MCP flexion by projecting motion onto the plane formed by the origin, joint, and distal.
    For non-thumb fingers, origin is the wrist; for thumb MCP flexion, origin is the thumb CMC.
    """
    v1 = np.array(joint) - np.array(origin)
    v2 = np.array(distal) - np.array(origin)
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    
    joint_distal_vector = np.array(distal) - np.array(joint)
    projected_vector = project_vector_onto_plane(joint_distal_vector, normal)
    reference_vector = project_vector_onto_plane(v1, normal)
    
    cosine_angle = np.dot(projected_vector, reference_vector) / (np.linalg.norm(projected_vector) * np.linalg.norm(reference_vector))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))
    
    # Ensure flexion is positive
    cross_product = np.cross(reference_vector, projected_vector)
    if np.dot(cross_product, palm_normal) < 0:
        angle = -angle
    
    return angle

def calculate_abduction_generic(origin, joint, reference, distal):
    """
    Calculate abduction at a joint. The angle is computed between:
      - the vector from 'origin' to the joint (the proximal segment)
      - the vector from the joint to 'distal' (the distal segment)
    The 'reference' point (e.g. thumb CMC for index finger or index finger MCP for thumb) is used
    to define the plane for projection.
    """
    v1 = np.array(joint) - np.array(origin)
    v2 = np.array(reference) - np.array(origin)
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    
    joint_distal_vector = np.array(distal) - np.array(joint)
    projected_vector = project_vector_onto_plane(joint_distal_vector, normal)
    reference_vector = project_vector_onto_plane(v1, normal)
    
    cosine_angle = np.dot(projected_vector, reference_vector) / (np.linalg.norm(projected_vector) * np.linalg.norm(reference_vector))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))
    
    cross_product = np.cross(reference_vector, projected_vector)
    if np.dot(cross_product, normal) < 0:
        angle = -angle
    
    return angle

class HandAngleNode(Node):
    def __init__(self):
        super().__init__('hand_angle_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_joint_angles', 10)
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(image_rgb)
        joint_angles = Float32MultiArray()

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Extract landmark coordinates as (x, y, z)
                landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
                
                # Draw landmarks on the image
                h, w, _ = image.shape
                for lm in hand_landmarks.landmark:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                
                # Draw red lines connecting landmarks (for visualization)
                finger_connections = [
                    (0,1), (0,5), (0,9), (0,13), (0,17),  # Palm to fingers
                    (1, 2), (2, 3), (3, 4),   # Thumb
                    (5, 6), (6, 7), (7, 8),   # Index finger
                    (9, 10), (10, 11), (11, 12),  # Middle finger
                    (13, 14), (14, 15), (15, 16),  # Ring finger
                    (17, 18), (18, 19), (19, 20)  # Pinky finger
                ]
                for connection in finger_connections:
                    x1, y1 = int(landmarks[connection[0]][0] * w), int(landmarks[connection[0]][1] * h)
                    x2, y2 = int(landmarks[connection[1]][0] * w), int(landmarks[connection[1]][1] * h)
                    cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                # Define palm reference normal (for flexion sign correction)
                palm_normal = np.array([0, 0, 1])
                wrist = landmarks[0]
                
                # Separate thumb landmarks (using Mediapipe indexing)
                # Thumb: 1 = CMC, 2 = MCP, 3 = IP, 4 = tip.
                thumb_cmc = landmarks[1]
                thumb_mcp = landmarks[2]
                thumb_ip  = landmarks[3]
                
                # Landmarks for the other four fingers:
                # Index: 5=MCP, 6=PIP, 7=DIP, 8=tip;
                # Middle: 9,10,11,12; Ring: 13,14,15,16; Pinky: 17,18,19,20.
                finger_mcps = [landmarks[5], landmarks[9], landmarks[13], landmarks[17]]
                finger_pips = [landmarks[6], landmarks[10], landmarks[14], landmarks[18]]
                finger_dips = [landmarks[7], landmarks[11], landmarks[15], landmarks[19]]
                
                angles = []
                # --- Thumb calculations (4 angles) ---
                # 1. Thumb CMC flexion: angle at thumb CMC computed using wrist, thumb CMC, and thumb MCP.
                thumb_cmc_flex = neutralize_angle(calculate_angle(wrist, thumb_cmc, thumb_mcp))
                # 2. Thumb CMC abduction: computed at thumb CMC using wrist as origin and using index finger MCP as reference.
                thumb_cmc_abd = calculate_abduction_generic(wrist, thumb_cmc, landmarks[5], thumb_mcp)
                # 3. Thumb MCP flexion: angle at thumb MCP computed using thumb CMC, thumb MCP, and thumb IP.
                thumb_mcp_flex = neutralize_angle(calculate_angle(thumb_cmc, thumb_mcp, thumb_ip))
                # 4. Thumb MCP abduction: computed at thumb MCP using thumb CMC as origin and index finger MCP as reference.
                thumb_mcp_abd = calculate_abduction_generic(thumb_cmc, thumb_mcp, landmarks[5], thumb_ip)
                angles.extend([thumb_cmc_flex, thumb_cmc_abd, thumb_mcp_flex, thumb_mcp_abd])
                
                # --- Other fingers (each 3 angles: MCP flexion, MCP abduction, PIP flexion) ---
                for i in range(4):
                    # For flexion at the finger MCP, use the wrist as origin.
                    mcp_flex = calculate_mcp_felxion(wrist, finger_mcps[i], finger_pips[i], palm_normal)
                    
                    # For abduction: for index finger (i==0), use thumb CMC as the reference; for others use previous finger's MCP.
                    if i == 0:
                        mcp_abd = calculate_abduction_generic(wrist, finger_mcps[i], thumb_cmc, finger_pips[i])
                    else:
                        mcp_abd = calculate_abduction_generic(wrist, finger_mcps[i], finger_mcps[i-1], finger_pips[i])
                    
                    # PIP flexion computed as the angle between MCP, PIP, and DIP.
                    pip_flex = neutralize_angle(calculate_angle(finger_mcps[i], finger_pips[i], finger_dips[i]))
                    
                    angles.extend([mcp_flex, mcp_abd, pip_flex])
                
                # Now angles is a list of 16 values (4 for thumb, 3 each for the 4 remaining fingers).
                joint_angles.data = angles
        
        self.publisher.publish(joint_angles)
        
        # Display the image with hand landmarks and connections.
        cv2.imshow("Hand Tracking", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandAngleNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
