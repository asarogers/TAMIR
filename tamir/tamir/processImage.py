#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ProcessDepthNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create a QoS profile for reliable communication
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Subscribe to regular image data (not compressed) with reliable QoS
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.compressed_image_callback, qos)
        
        # Publisher for processed image
        self.image_publisher = self.create_publisher(
            Image,
            '/processed/color/image',
            qos_profile=qos)
        
        self.latest_img = None
        self.processing = False
        
        self.get_logger().info('Image processing node started')
        
    def compressed_image_callback(self, msg):
        if self.processing:
            # skip if still processing image
            return
        try:
            self.processing = True
            # Quickly store the latest color image
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            ros_img.header = msg.header  # Preserve original timestamp

            self.image_publisher.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f'Error converting compressed image: {str(e)}')

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
