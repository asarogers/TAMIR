from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from bleak import BleakScanner, BleakClient
import asyncio


class Bluetooth(Node):
    """ROS2 Node to manage Bluetooth scanning and connections."""

    def __init__(self):
        """Initialize the Bluetooth node."""
        super().__init__('bluetooth_node')
        self.print = self.get_logger().info
        self.client = None
        self.target_device = '26:91:71:54:00:09'

    async def pair_with_speaker(self):
        """Pair with the target Bluetooth device."""
        try:
            self.print('Scanning for Bluetooth devices...')
            devices = await BleakScanner.discover()

            if not devices:
                self.print('No devices found.')
                return False

            # Check if the target device is available
            found = False
            for device in devices:
                if device.address == self.target_device:
                    found = True
                    self.print(f"Found target device: {device.name} [{device.address}]")
                    break

            if not found:
                self.print(f"Target device with address {self.target_device} not found.")
                return False

            # Connect to the target device
            self.client = BleakClient(self.target_device)
            await self.client.connect()
            self.print(f'Connected to device: {self.target_device}')
            return True
        except Exception as e:
            self.print(f"Error pairing with device: {str(e)}")
            return False

    async def disconnect(self):
        """Disconnect from the Bluetooth device."""
        try:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                self.print('Disconnected from Bluetooth device')
        except Exception as e:
            self.print(f"Error during disconnection: {str(e)}")

    async def run(self):
        """Main asynchronous task for the node."""
        paired = await self.pair_with_speaker()

        if paired:

            # Keep the node running
            self.print("Node running. Use Ctrl+C to exit.")
            try:
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                    await asyncio.sleep(0.1)  # Allow other async tasks to run
            except KeyboardInterrupt:
                self.print("Node interrupted by user.")
            finally:
                await self.disconnect()
        else:
            self.print("Could not pair with the device. Exiting.")


def main(args=None):
    """Entry point for ROS 2."""
    rclpy.init(args=args)
    node = Bluetooth()

    try:
        # Run the asynchronous main task
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
