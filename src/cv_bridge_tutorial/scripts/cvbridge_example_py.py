#!/usr/bin/python3

# The client library ROS2 Python
import rclpy

# The ROS2 Python class node
from rclpy.node import Node

# The ROS2 image type
from sensor_msgs.msg import Image

# This library provides the bridge for converting ROS image <-> OpenCV
from cv_bridge import CvBridge

# The library used for image processing
import cv2


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__("image_processor_node")

        # Subscribe to topic /camera/image_raw
        self.subscription = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher to republish images after processing
        self.publisher = self.create_publisher(
            Image, "/processed_image_topic_python", 10
        )
        # Create a class attibute CVBridge
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Uses the imgmsg_to_cv2 method in the cv_bridge to convert the image
            # from the ROS image format to an OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # From here, you can use all cv2 methods on your image

        # Convert OpenCV image in grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Performs thresholding on the image and assigns the result to thresholded_image
        _, thresholded_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)

        # Convert OpenCV image to ROS image message
        processed_msg = self.cv_bridge.cv2_to_imgmsg(
            thresholded_image, encoding="mono8"
        )

        # Republie l'image sur ROS
        self.publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
