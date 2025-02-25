#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ProcessDepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')
        self.bridge = CvBridge()
        
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile=depth_qos)
            
        self.publisher = self.create_publisher(
            Image,
            '/processed/depth/image',
            qos_profile=depth_qos)
            
        self.get_logger().info('Depth node started with optimized processing')
        
    def depth_callback(self, msg):
        try:
            # Use local variables to speed up attribute lookups
            bridge = self.bridge
            publisher = self.publisher

            # Convert ROS Image to OpenCV image
            depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if not isinstance(depth_img, np.ndarray):
                self.get_logger().error('Converted image is not a numpy array')
                return
            
            # Convert to uint16 if necessary
            # if depth_img.dtype not in (np.uint8, np.uint16):
            #     if np.issubdtype(depth_img.dtype, np.floating):
            #         depth_img = (depth_img * 65535).astype(np.uint16)
            #     else:
            #         depth_img = depth_img.astype(np.uint16)
            
            # Apply threshold to remove low-depth values
            _, proc_img = cv2.threshold(depth_img, 10, 65535, cv2.THRESH_BINARY)
            
            # Convert the processed image back to a ROS message
            encoding = 'mono16'
            proc_msg = bridge.cv2_to_imgmsg(proc_img, encoding=encoding)
            proc_msg.header = msg.header  
            publisher.publish(proc_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

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
