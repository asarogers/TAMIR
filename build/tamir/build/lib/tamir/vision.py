"""
Object Detection with YOLO and Live Camera Feed Node.

This script defines a ROS 2 node for detecting objects in a live camera feed using a
YOLO model from the Ultralytics library. It uses the `sensor_msgs/Image` topic for 
camera data, runs inference on each frame, and displays the detections in an OpenCV window.

Dependencies
------------
- ROS 2
- OpenCV (for image processing and visualization)
- cv_bridge (for converting ROS image messages to OpenCV format)
- ultralytics (for YOLO model loading and inference)

Topics
------
- Subscribed:
  - `/camera/camera/color/image_raw`: RGB image stream.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

# Make sure you have 'ultralytics' installed in your environment: pip install ultralytics
from ultralytics import YOLO


class YoloVisualizer(Node):
    """
    ROS 2 Node for YOLO Object Detection and Visualization.

    This node subscribes to an RGB camera feed, processes each frame with a YOLO model, 
    and displays the detection results using OpenCV.

    Attributes
    ----------
    subscription : rclpy.subscription.Subscription
        Subscribes to the live RGB camera feed topic.
    bridge : CvBridge
        Converts ROS image messages to OpenCV format for processing.
    model : YOLO
        The YOLO model (ultralytics) for object detection.
    """

    def __init__(self):
        """Initialize the YOLO node and subscriptions."""
        super().__init__('yolo_visualizer')
        
        # Create a subscription to the camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )
        
        # For converting sensor_msgs/Image to OpenCV images
        self.bridge = CvBridge()
        
        # Load the YOLO model (ensure yolo11n.pt is in the correct path or specify absolute path)
        self.model = YOLO('yolo11n.pt')
        self.logger = self.get_logger().info
        self.logger("Initialized YOLO Visualizer Node with yolo11n.pt")

    def listener_callback(self, data):
        """
        Receive camera frames, run YOLO inference, and display bounding boxes on detections.

        :param data: The RGB image message from the camera.
        :type data: sensor_msgs.msg.Image
        """
        # Convert the ROS Image message to an OpenCV image (BGR format)
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Run YOLO inference on the current frame
        # By default, Ultralytics returns a list of "Results" objects, one per image
        results = self.model.predict(current_frame, verbose=False)  # or self.model(current_frame)

        # results[0] corresponds to the first (and only) image in this batch
        # Each result has a .boxes attribute which includes bounding box data
        # (x1, y1, x2, y2, confidence, class), etc.
        if len(results) > 0:
            detections = results[0].boxes
            for box in detections:
                # box.xyxy, box.conf, box.cls, etc. are possible attributes
                # xyxy format: [x1, y1, x2, y2]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                # Get class name from Ultralytics model (if available)
                # This depends on your custom model's class list
                class_name = self.model.names.get(cls_id, f'class_{cls_id}')

                # Draw the bounding box
                color = (0, 255, 0)  # green bounding box
                cv2.rectangle(current_frame, (x1, y1), (x2, y2), color, 2)
                
                # Display label and confidence
                label = f"{class_name} {conf:.2f}"
                cv2.putText(
                    current_frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

        # Show the result in a window
        cv2.imshow('Camera Feed with YOLO Detections', current_frame)
        cv2.waitKey(1)


def main(args=None):
    """Main entry point for the node. Initializes and spins the ROS 2 node."""
    rclpy.init(args=args)
    node = YoloVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
