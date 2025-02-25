#!/usr/bin/env python3
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_cmd = Twist()
        
        # Define key mappings for arrow keys
        self.key_map = {
            '\x1b[A': (1.0, 0.0),    # Up arrow - Move forward
            '\x1b[B': (-1.0, 0.0),   # Down arrow - Move backward
            '\x1b[D': (0.0, 1.0),    # Left arrow - Twist left
            '\x1b[C': (0.0, -1.0),   # Right arrow - Twist right
        }
        
    def publish_cmd(self, linear_x, angular_z):
        self.move_cmd.linear.x = float(linear_x)
        self.move_cmd.angular.z = float(angular_z)
        self.publisher.publish(self.move_cmd)

def main():
    rclpy.init()
    teleop_node = KeyboardTeleop()
    
    # Prepare terminal settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    print("Use arrow keys to control the robot:")
    print("Up/Down arrows: forward/backward")
    print("Left/Right arrows: rotate left/right")
    print("Press Ctrl+C to exit")
    
    try:
        tty.setraw(fd)
        
        while rclpy.ok():
            # Read initial character
            char = sys.stdin.read(1)
            
            if char == '\x03':  # Exit on Ctrl+C
                break
                
            # If it's an escape sequence (arrow key)
            if char == '\x1b':
                # Read the next two characters of the sequence
                char += sys.stdin.read(2)
                
                if char in teleop_node.key_map:
                    linear, angular = teleop_node.key_map[char]
                    teleop_node.publish_cmd(linear, angular)
                else:
                    # Stop if key not recognized
                    teleop_node.publish_cmd(0.0, 0.0)
            else:
                # Stop for any other key
                teleop_node.publish_cmd(0.0, 0.0)
                
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()