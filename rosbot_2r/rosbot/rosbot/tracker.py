#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # Import ROS2 point cloud processing

class OptimizedTrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        # Parameters for processing and performance
        self.processing_scale = 0.5  # Scale factor to reduce resolution
        self.confidence_threshold = 0.55
        self.skip_frames = 2         # Process one frame every N frames
        self.frame_counter = 0

        self.bridge = CvBridge()
        self.valid_names = ["dog"]

        # QoS profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Latest data storage (with lock for thread safety)
        self.camera_info = None
        self.depth_image = None
        self.latest_image = None
        self.image_lock = threading.Lock()

        # Subscriptions
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.compressed_image_callback, qos)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.pointcloud_callback, qos)
        self.pointcloud = None  # Store latest point cloud data

        self.detection_sub = self.create_subscription(AprilTagDetectionArray, 'detections',
                                                      self.detection_callback, 10)

        # Flags and variables for inference
        self.model = None
        self.target_detection = False
        self.model_loaded = threading.Event()
        self.last_inference_time = time.time()
        self.fps = 0

        # Start the inference thread (daemon so it stops on shutdown)
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

        # Load model in a separate thread
        self.get_logger().info('Loading YOLO model...')
        self.model_thread = threading.Thread(target=self.load_model, daemon=True)
        self.model_thread.start()


    def pointcloud_callback(self, msg):
        """ Store the latest point cloud message """
        self.pointcloud = msg

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
            # self.get_logger().info(f'Tag Data: {detection}')
            self.target_detection = True
            self.target_centre = (
                int(detection.centre.x),
                int(detection.centre.y)
            )
        else:
            self.target_detection = False
            self.target_centre = None

    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Cache intrinsic parameters for 3D calculations
        if not hasattr(self, 'cam_params'):
            cx = msg.k[2]
            cy = msg.k[5]
            fx = msg.k[0]
            fy = msg.k[4]
            self.cam_params = (cx, cy, fx, fy)
            self.get_logger().info('Camera info received and parameters cached.')

    def load_model(self):
        try:
            self.model = YOLO('yolo11n.pt')
            self.model.to('cpu')
            self.model.conf = self.confidence_threshold
            self.model.iou = 0.45
            self.model.max_det = 20

            # Filter for valid classes
            class_indices = [idx for idx, name in self.model.names.items() if name in self.valid_names]
            if class_indices:
                self.model.classes = class_indices

            self.get_logger().info(f'Model loaded on {self.model.device}')
            self.model_loaded.set()
        except Exception as e:
            self.get_logger().error(f'Error loading model: {str(e)}')

    def depth_callback(self, data):
        try:
            # Convert ROS Image to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')

    def compressed_image_callback(self, msg):
        try:
            # Quickly store the latest color image
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_img
        except Exception as e:
            self.get_logger().error(f'Error converting compressed image: {str(e)}')



    def pixel_to_3d(self, px, py, depth_value):
        if not self.camera_info or not hasattr(self, 'cam_params'):
            return None, None, None
        cx, cy, fx, fy = self.cam_params
        depth_m = depth_value / 1000.0  # Assuming depth is in mm
        x = (px - cx) * depth_m / fx
        y = (py - cy) * depth_m / fy
        z = depth_m
        return x, y, z
    
    def cloud_to_3d(self, px, py):
        """ Extract XYZ coordinates from the point cloud at pixel (px, py). """
        if self.pointcloud is None:
            # self.get_logger().warn("Point cloud data not available yet.")
            return None, None, None
            
        try:
            # Check if the point cloud is the expected type
            if not isinstance(self.pointcloud, PointCloud2):
                self.get_logger().error(f"Incorrect point cloud type: {type(self.pointcloud).__name__}")
                return None, None, None
                
            # Get point cloud dimensions
            width = self.pointcloud.width
            height = self.pointcloud.height
            
            # Check bounds
            if px < 0 or py < 0 or px >= width or py >= height:
                self.get_logger().warn(f"Pixel coordinates ({px}, {py}) out of bounds")
                return None, None, None
                
            # Use the raw data directly to extract the point
            # This approach avoids the pc2.read_points function that's causing problems
            row_step = self.pointcloud.row_step
            point_step = self.pointcloud.point_step
            
            # Calculate the byte offset for the desired point
            offset = (py * row_step) + (px * point_step)
            
            # Extract the bytes for this point
            data = self.pointcloud.data
            
            # Make sure we're not going out of bounds in the data array
            if offset + 12 > len(data):  # 12 bytes for 3 float32 values (x,y,z)
                self.get_logger().warn("Point index out of data bounds")
                return None, None, None
                
            # Convert bytes to float32 (assuming x,y,z are consecutive float32 values)
            import struct
            x = struct.unpack('f', data[offset:offset+4])[0]
            y = struct.unpack('f', data[offset+4:offset+8])[0]
            z = struct.unpack('f', data[offset+8:offset+12])[0]
            
            # Check for NaN values
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                return None, None, None
                
            return x, y, z
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud data: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
        self.get_logger().warn(f"No valid point found at ({px}, {py})")
        return None, None, None
    

    def inference_loop(self):
        while rclpy.ok():
            # Wait until model is loaded
            if not self.model_loaded.is_set():
                time.sleep(0.01)
                continue

            # Get a copy of the latest image
            with self.image_lock:
                if self.latest_image is None:
                    img = None
                else:
                    # Copy to avoid processing a frame being updated concurrently
                    img = self.latest_image.copy()

            if img is None:
                time.sleep(0.005)
                continue

            # Skip frames as configured
            self.frame_counter += 1
            if self.frame_counter % self.skip_frames != 0:
                time.sleep(0.001)
                continue

            # Resize image for faster inference if needed
            if self.processing_scale != 1.0:
                h, w = img.shape[:2]
                proc_img = cv2.resize(img, (int(w * self.processing_scale), int(h * self.processing_scale)))
            else:
                proc_img = img

            # Run YOLO inference
            start_time = time.time()
            results = self.model.predict(proc_img, verbose=False)
            inference_duration = time.time() - start_time

            # Update FPS estimate
            current_time = time.time()
            dt = current_time - self.last_inference_time
            self.last_inference_time = current_time
            self.fps = 1.0 / dt if dt > 0 else 0

            # 
            if self.target_detection and self.target_centre:
                x, y = self.target_centre
                target_x, target_y, target_z = self.cloud_to_3d(x, y)

                if target_x is not None:
                    self.get_logger().info(f"AprilTag 3D Position: X={target_x:.2f}, Y={-target_y:.2f}, Z={target_z:.2f} meters")



            # Process detections if any
            if results and len(results) > 0:
                detections = results[0].boxes
                for box in detections:
                    # Get bounding box coordinates
                    xyxy = box.xyxy[0].cpu().numpy()
                    if self.processing_scale != 1.0:
                        xyxy = xyxy / self.processing_scale
                    x1, y1, x2, y2 = xyxy.astype(int)
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    class_name = self.model.names.get(cls_id, f'class_{cls_id}')
                    



                    if class_name in self.valid_names:
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(img, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        # If depth info is available, compute and display 3D position
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        if 0 <= center_x and 0 <= center_y:
                            x3d, y3d, z3d = self.cloud_to_3d(center_x, center_y)

                            if x3d is not None:
                                self.get_logger().info(f"X:{x3d:.2f} Y:{y3d:.2f} Z:{z3d:.2f}m")

                                    # cv2.putText(img, f"X:{x3d:.2f} Y:{y3d:.2f} Z:{z3d:.2f}m",
                                    #             (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Overlay performance metrics and depth status
            depth_status = "Depth: Available" if self.depth_image is not None else "Depth: Not Available"
            cv2.putText(img, f"FPS: {self.fps:.1f} | {depth_status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Show the result
            cv2.imshow('Optimized Tracker', img)
            key = cv2.waitKey(1) & 0xFF

            # Allow dynamic adjustments via keypresses
            if key in [ord('+'), ord('=')]:
                self.processing_scale = min(1.0, self.processing_scale + 0.1)
                self.get_logger().info(f'Processing scale increased to {self.processing_scale:.1f}')
            elif key == ord('-'):
                self.processing_scale = max(0.2, self.processing_scale - 0.1)
                self.get_logger().info(f'Processing scale decreased to {self.processing_scale:.1f}')
            elif key == ord('s'):
                self.skip_frames = max(1, (self.skip_frames % 5) + 1)
                self.get_logger().info(f'Processing every {self.skip_frames} frames')

            # Brief sleep to yield CPU
            time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
