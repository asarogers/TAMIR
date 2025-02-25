import rclpy
from rclpy.node import Node
import cv2

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Simple camera node started')

    def timer_callback(self):
        # Create a blank image
        img = cv2.imread('/dev/null', cv2.IMREAD_COLOR)
        if img is None:
            img = 255 * numpy.ones((400, 600, 3), numpy.uint8)
        
        # Display the image
        cv2.imshow('Simple Camera Window', img)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()