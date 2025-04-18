"""
Publishes a static transformation from the `doorbase` frame to (`base`) for camera localization.

The node uses the `tf2_ros.StaticTransformBroadcaster` to broadcast the static transform.
"""
import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped, Quaternion
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math


class CameraLocalizer(Node):
    """ROS2 Node that Publishes the doorbase's Tag for the Camera to localize itself."""

    def __init__(self):
        """Initialize the CameraLocalizer Node."""
        super().__init__('camera_localizer')
        self.get_logger().info('CameraLocalizer Node Started')

        # Static Transform Broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Broadcast the transformation from doorbase -> BaseTag
        doorbase_tag = TransformStamped()
        doorbase_tag.header.stamp = self.get_clock().now().to_msg()
        doorbase_tag.header.frame_id = 'right_door_frame'  # TODO: Parameterize the frame ids
        doorbase_tag.child_frame_id = 'left_door_frame'
        doorbase_tag.transform.translation.x = 0.0
        doorbase_tag.transform.translation.y = 1.0
        doorbase_tag.transform.translation.z = 0.0
        euler_rotation = (-math.pi / 2.0, -math.pi / 2.0, 0.0)  # RPY
        quaternion = self.euler_to_quaternion(*euler_rotation)
        doorbase_tag.transform.rotation = quaternion
        self.static_broadcaster.sendTransform(doorbase_tag)
        self.get_logger().info('CameraLocalizer Published doorbase -> BaseTag')

        world_to_camera = TransformStamped()
        world_to_camera.header.stamp = self.get_clock().now().to_msg()
        world_to_camera.header.frame_id = 'world'
        world_to_camera.child_frame_id = 'camera_link'
        
        # Position the camera in the world frame (adjust these values as needed)
        world_to_camera.transform.translation.x = 2.9972
        world_to_camera.transform.translation.y = 0.0
        world_to_camera.transform.translation.z = 2.0  # Camera height from ground
        
        # Orient camera (assuming camera is looking forward along x-axis)
        camera_quaternion = self.euler_to_quaternion(0.0, 0.0, 0.0)
        world_to_camera.transform.rotation = camera_quaternion
        
        # Send the transform
        self.static_broadcaster.sendTransform(world_to_camera)
        
        # Log success

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles to a Quaternion.

        :param roll: The roll angle [rad]
        :type roll: float
        :param pitch: The pitch angle [rad]
        :type pitch: float
        :param yaw: The yaw angle [rad]
        :type yaw: float
        :return: The Quaternion representation of the Euler angles
        :rtype: Quaternion
        """
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        q.w = cy * cr * cp + sy * sr * sp
        q.x = cy * sr * cp - sy * cr * sp
        q.y = cy * cr * sp + sy * sr * cp
        q.z = sy * cr * cp - cy * sr * sp
        return q


def main(args=None):
    """Set up the node, spins it to handle callbacks, and gracefully shuts down."""
    rclpy.init(args=args)
    camera_localizer = CameraLocalizer()
    rclpy.spin(camera_localizer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
