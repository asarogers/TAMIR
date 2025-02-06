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
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np
from ultralytics import YOLO
from tamir_interface.msg import Behaviors, BehaviorList
import geometry_msgs
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Transform

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
        self.door_x = 0.2
        self.door_z = 2.7
        self.behavior= {
            "dogIsInBathroom" : False
        }
        self.target_detection = False
        self.target_centre = None
        self.counter = 0

        self.behaviorPublisher = self.create_publisher(BehaviorList, "behavior_msg", 10)
        self.detection_sub = self.create_subscription(AprilTagDetectionArray, 'detections',
                                                      self.detection_callback, 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) 
    
    def detection_callback(self, msg):
        """
        Handle AprilTag detection messages.

        Extracts information about the first detected tag (e.g., ID, center coordinates),
        updates the detection status, and logs the detection.

        :param msg: AprilTag detection array message containing detected tag details.
        :type msg: apriltag_msgs.msg.AprilTagDetectionArray

        """
        if len(msg.detections) > 0:
            detection = msg.detections[0]
            # self.logger(f'Tag Data: {detection}')
            self.target_detection = True
            self.target_centre = (
                int(detection.centre.x),
                int(detection.centre.y)
            )
        else:
            self.target_detection = False
            self.target_centre = None

    def checkBehavior(self,class_name, x, y, z):
        # self.logger(f"3D Coordinates of {class_name}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        # self.logger(f"3D Coords of {class_name}: x={x:.3f}")
        if z >= self.door_z:
        #     self.logger("passed by z")
        #     self.logger(f"3D Coords of {class_name}: x={x:.3f}")
            door_x_min = self.door_x - (self.door_width / 2)
            door_x_max = self.door_x + (self.door_width / 2)
            # self.logger(f"{class_name}:  {door_x_min} < {x:.3f} <= {door_x_max}")

            if door_x_min <= x <= door_x_max:
                self.logger("Dog is in bathroom")
                # self.logger(f"3D Coords of {class_name}: x={x:.3f}")
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

        # Do nothing until results show on the screen
        if len(results) > 0:
            detections = results[0].boxes

            # get all of the detection boxes
            for box in detections:

                # get coordinates and names of all the boxes 
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                class_name = self.model.names.get(cls_id, f'class_{cls_id}')

                # only continue if their is a depth immage and info
                if self.depth_image is not None and self.camera_info is not None:
                    if self.target_detection and self.target_centre is not None:

                        # draw a circle where the apriltag is detected
                        cv2.circle(current_frame, self.target_centre, 10, (0, 255, 0), -1)
                        self.logger(f"target center = {self.target_centre}")
                        target_depth = self.depth_image[self.target_centre]
                        # only continue if there is some depth
                        if target_depth > 0:
                            target_x, target_y, target_z = self.pixel_to_3d(self.target_centre[0], self.target_centre[1], target_depth)
                            target_x =target_x/1000
                            target_y = target_y /1000
                            target_z = target_z / 1000
                            self.logger(f"depth target center = {target_x}, {target_y}, {target_z}")

                            t = geometry_msgs.msg.TransformStamped()
                            t.header.stamp = self.get_clock().now().to_msg()
                            t.header.frame_id = "camera_link"
                            t.child_frame_id = f"right_door_frame"

                            t.transform.translation.x = target_x
                            t.transform.translation.y = target_y
                            t.transform.translation.z = target_z

                            t.transform.rotation.x = 0.0
                            t.transform.rotation.y = 0.0
                            t.transform.rotation.z = 0.0
                            t.transform.rotation.w = 1.0  # No rotation

                            self.tf_broadcaster.sendTransform(t)
                    
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

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    depth_value = self.depth_image[center_y, center_x]

                    if depth_value > 0:
                        x, y, z = self.pixel_to_3d(center_x, center_y, depth_value)
                        x = x /1000
                        y = y /1000
                        z = z /1000
                        self.checkBehavior(class_name, x, y, z)
                            
            #         elif  self.depth_image is None:
            #             self.logger("No depth image")
            #         elif self.camera_info is  None:
            #             self.logger("No camera info")
            #         else:
            #             self.logger("A different error")
            # if self.behavior["dogIsInBathroom"]:
            #     self.logger("****Dog found in bathroom")
            msg = BehaviorList()

            behavior = Behaviors()
            behavior.name = "dogIsInBathroom"
            behavior.state = self.behavior["dogIsInBathroom"]

            msg.states = [behavior]

            # msg.states = states = [behavior]
            # self.logger(f"states = {msg}")
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
    
    def capture_picture(self, current_frame, x1, x2, y1, y2):
        self.counter +=1
        filename = f"test_image{self.counter}.jpg"
        cv2.imwrite(filename, current_frame[y1:y2, x1:x2])
        print("Image saved!")


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

