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
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO
from tamir_interface.msg import Behaviors, BehaviorList

class YoloVisualizer(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')
        self.bridge = CvBridge()

        # Subscriptions to RGB and Depth topics
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.depth_image = None
        self.camera_info = None

        # YOLO model
        self.model = YOLO('yolo11n.pt')
        self.logger = self.get_logger().info
        self.logger("Initialized YOLO Visualizer Node with yolo11n.pt")
        self.door_width = 0.7112
        self.valid_names = ["dog"]
        self.y_min = 1000
        self.y_max = 0
        self.door_x = -0.291
        self.door_z = 1.9
        self.behavior= {
            "dogIsInBathroom" : False
        }

        self.behaviorPublisher = self.create_publisher(BehaviorList, "behavior_msg", 10)
    
    def checkBehavior(self,class_name, x, y, z):
        # "3D Coordinates of {class_name}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        # self.logger(f"3D Coords of {class_name}: x={x:.3f}")
        if z >= self.door_z:
            # self.logger("passed by z")
            door_x_min = self.door_x - (self.door_width / 2)
            door_x_max = self.door_x + (self.door_width / 2)
            if door_x_min <= x <= door_x_max:
                self.logger("Dog is in bathroom")
                self.behavior["dogIsInBathroom"] = True
        elif z < self.door_z:
            self.behavior["dogIsInBathroom"] = False


    def camera_info_callback(self, camera_info_msg):
        """Callback to store camera intrinsic parameters."""
        self.camera_info = camera_info_msg

    def depth_callback(self, data):
        """Callback to store the latest depth frame."""
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def rgb_callback(self, data):
        """Callback to process RGB frames and run YOLO inference."""
        # Convert RGB frame
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model.predict(current_frame, verbose=False)

        if len(results) > 0:
            detections = results[0].boxes
            for box in detections:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                class_name = self.model.names.get(cls_id, f'class_{cls_id}')

                if class_name in self.valid_names:

                    # Draw bounding box and label
                    cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        current_frame,
                        f"{class_name} {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    # Get depth value and compute 3D coordinates
                    if self.depth_image is not None and self.camera_info is not None:
                        
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        depth_value = self.depth_image[center_y, center_x]

                        if depth_value > 0:  # Ensure valid depth
                            x, y, z = self.pixel_to_3d(center_x, center_y, depth_value)
                            x = x /1000
                            y = y /1000
                            z = z /1000
                            self.checkBehavior(class_name, x, y, z)
                            
                    elif  self.depth_image is None:
                        self.logger("No depth image")
                    elif self.camera_info is  None:
                        self.logger("No camera info")
                    else:
                        self.logger("A different error")
            if self.behavior["dogIsInBathroom"]:
                self.logger("****Dog found in bathroom")
            msg = BehaviorList()

            behavior = Behaviors()
            behavior.name = "dogIsInBathroom"
            behavior.state = self.behavior["dogIsInBathroom"]

            msg.states = [behavior]

            # msg.states = states = [behavior]
            self.behaviorPublisher.publish(msg)
        
        # Display the result
        cv2.imshow('YOLO Detections with Depth', current_frame)
        cv2.waitKey(1)

    def pixel_to_3d(self, pixel_x, pixel_y, depth):
        """Convert pixel coordinates and depth to 3D coordinates."""
        cx = self.camera_info.k[2]  # Principal point X
        cy = self.camera_info.k[5]  # Principal point Y
        fx = self.camera_info.k[0]  # Focal length X
        fy = self.camera_info.k[4]  # Focal length Y

        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth

        return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

