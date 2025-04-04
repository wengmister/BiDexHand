import cv2
import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YoloNode(Node):
    """
    Use Yolo to identify objects in the scene
    """
    def __init__(self):
        super().__init__("pose")
        self.bridge = CvBridge()
        self.declare_parameter("model",
                               value="yolo11n.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value)
        self.create_subscription(Image, '/camera/wrist/color/image_rect_raw', self.yolo_callback, 10)
        self.image_pub = self.create_publisher(Image, 'image_yolo', 10)

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        results = self.model(cv_image)

        # Process the results (draw bounding boxes on the image)
        for result in results:
            cv_image = result.plot()

            # Check if there are any object detected
            if not result.boxes:
                self.get_logger().info("No object detected")
                continue
            for box in result.boxes:
                x, y, w, h = box.xywh[0]  # center x, center y, width, height
                self.get_logger().debug(f"Detected object at ({x}, {y}) with width {w} and height {h}")
                center_x = int(x)
                center_y = int(y)

                # Draw bounding box
                cv2.rectangle(cv_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
                # Draw center
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                # Label object using the 'cls' attribute
                class_id = int(box.cls)
                cv2.putText(cv_image, result.names[class_id], (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Convert back to ROS image
        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # publish
        self.image_pub.publish(new_msg)

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()