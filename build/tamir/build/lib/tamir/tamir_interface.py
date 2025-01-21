"""
Tamir Interface Node.

This module defines the `TamirInterface` class, which acts as a wrapper for various robotic components,
including the Bluetooth_Node.

Classes:
--------
TamirInterface:
    A ROS 2 node that initializes and manages the robotic interface, including Bluetooth operations.

Functions:
----------
main(args=None):
    Initializes the ROS 2 environment and runs the `TamirInterface` node.

Authors:
--------
Asa Rogers
Date: 2025-01-01
"""
# Copyright 2024 Asa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tamir.Bluetooth_Node import Bluetooth_Node
import asyncio
import subprocess
from launch_ros.substitutions import FindPackageShare
import os
from std_srvs.srv import Empty
from bleak import BleakScanner, BleakClient
import ultralytics

class TamirInterface(Node):
    """
    ROS 2 Node for initiating and managing robotic interface actions.

    This class integrates the `Bluetooth_Node` functionality with other system components, enabling
    streamlined operations for managing robotic tasks.

    Attributes:
    -----------
    bluetooth : Bluetooth_Node
        An instance of the `Bluetooth_Node` class responsible for Bluetooth scanning and connections.

    Methods:
    --------
    __init__():
        Initializes the TamirInterface node, setting up necessary components.
    """

    def __init__(self):
        """
        Initialize the Tamir Interface node.

        This constructor initializes the `TamirInterface` node and creates an instance of the `Bluetooth_Node` class
        for handling Bluetooth operations.
        """
        super().__init__('tamir_interface_node')
        self.get_logger().debug('Tamir interface Started!')
        self.print = self.get_logger().info
        self.bluetooth = Bluetooth_Node()
        client_cb_group = ReentrantCallbackGroup()


        self.correctiveSignal = self.create_service(Empty, 'correctiveSignal', self.play_audio, callback_group=client_cb_group)
        self.bluetoothScanner = self.create_service(Empty, 'scan_for_devices', self.bluetooth_scanner, callback_group=client_cb_group)



    def play_audio(self, request, response):
        """Play audio."""
        file_path = 'experiment.mp3'
        tamir = FindPackageShare('tamir').find('tamir')
        full_path = os.path.join(tamir, file_path)
        subprocess.run(['mpg321', full_path])

    def bluetooth_scanner(self, request, response):
        """Scan for available bluetooth devices"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.scan_for_devices())
            loop.close()

            return response

        except Exception as e:
            self.print(f"Error pairing with device: {str(e)}")
            return response

    async def scan_for_devices(self):
        self.print('scanning')
        devices = await BleakScanner.discover()

        if not devices:
            self.print('No devices found.')
            return False
        
        for device in devices:
            self.print(f"devices {device.name} [{device.address}]")
        self.print("done")
 
async def run(node):
    """Main asynchronous task for the node."""
    paired = await node.pair_with_speaker()

    if paired:
        # Keep the node running
        node.print("Node running. Use Ctrl+C to exit.")
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            node.print("Node interrupted by user.")
        finally:
            await node.disconnect()
    else:
        node.print("Could not pair with the device. Exiting.")

def main(args=None):
    """
    Initialize and run the Tamir Interface node.

    This function sets up the ROS 2 environment, initializes the `TamirInterface` node, and spins it
    to maintain an active state until interrupted.

    Parameters:
    -----------
    args : list, optional
        Command-line arguments passed to the ROS 2 node. Defaults to None.

    Example:
    --------
    To run the node:
    >>> python3 tamir_interface.py
    """
    rclpy.init(args=args)
    tamir = TamirInterface()


    try:
        # Run the Bluetooth_Node asynchronously
        # asyncio.run(run(tamir.bluetooth))
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(tamir)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            tamir.destroy_node()
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()