#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ProcessDepthNode(Node):
    def __init__(self):
        super().__init__('image_node')
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create a QoS profile for reliable communication
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Use RELIABLE for both publisher and subscriber
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Subscribe to regular image data (not compressed) with reliable QoS
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.depth_callback,
            qos_profile=depth_qos)
        
        # Publisher for processed image
        self.image_publisher = self.create_publisher(
            Image,
            '/processed/color/image',
            qos_profile=depth_qos)
            
        self.get_logger().info('Image processing node started')
        
    def depth_callback(self, data):
        """Callback to process and publish the image."""
        try:
            # Convert the received ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Example processing step: Convert the image to grayscale
            processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert the processed OpenCV image back to a ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='mono8')

            # Publish the processed image using the standard Image message
            self.image_publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ProcessDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
