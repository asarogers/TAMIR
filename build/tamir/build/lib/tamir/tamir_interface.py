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
from rclpy.executors import MultiThreadedExecutor
from tamir_interface.msg import BehaviorList 


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
        self.target_device = "26:91:71:54:00:09"
        self.client = None
        self.soundPosition = 35

        self.correctiveSignalService = self.create_service(Empty, 'correctiveSignal', self.play_audio, callback_group=client_cb_group)
        self.bluetoothScanner = self.create_service(Empty, 'scan_for_devices', self.bluetooth_scanner, callback_group=client_cb_group)
        self.pairBluetooth = self.create_service(Empty, 'pair_bluetooth', self.connect_speaker, callback_group=client_cb_group)

        self.timer = self.create_timer(1, self.process_latest_message)

        self.correctiveSignal_client = self.create_client(Empty, "correctiveSignal")
        while not self.correctiveSignal_client.wait_for_service(timeout_sec=2.0):
            self.print("Waiting for corrective service...")


        self.subscription = self.create_subscription(
                    BehaviorList,
                    'behavior_msg',
                    self.listener_callback,
                    10)
        self.behavior = None
    
    def process_latest_message(self):
        """Process the latest message once every second."""
        if hasattr(self.behavior, "state") and self.behavior.state:
            self.print("dog is in bathroom")
            # self.print(f"behavior = {self.behavior}")
            self.begin_corrective_signal()


    def listener_callback(self, msg):
        self.behavior = msg.states[0]
    
    def begin_corrective_signal(self):
        try:
            req = Empty.Request()
            future = self.correctiveSignal_client.call_async(req)

        except Exception as e:
            self.get_logger().error(f"Error calling corrective signal service: {e}")


    def play_audio(self, request, response):
        """Play audio starting from 25 seconds and stop after 5 seconds."""
        file_path = 'high_pitch.mp3'
        tamir = FindPackageShare('tamir').find('tamir')
        full_path = os.path.join(tamir, file_path)
        
        # Play the audio starting at 25 seconds and play for 5 seconds
        subprocess.run(['ffplay', '-ss', f'{self.soundPosition}', '-t', '1', '-i', full_path, '-autoexit', '-nodisp'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.soundPosition += 1
        return response


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
        found = False

        if not devices:
            self.print('No devices found.')
            return False
        
        for device in devices:
            self.print(f"devices {device}")
        self.print("done")


    async def connect_speaker(self, request, response):
        """Pair with the target Bluetooth device."""
        try:
            # if await self.check_device_connected(self.target_device) is False:
            # print("No device connected")
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.pair_bluetooth_speaker())
            loop.close()
            print("Device connected")
            return response

        except Exception as e:
            self.print(f"Error pairing with device: {str(e)}")
            return response
        
    async def check_device_connected(self, device_address):
        try:
            async with BleakClient(device_address) as client:
                connected = await client.is_connected()
                print(f"Device {device_address} connected: {connected}")
                return True
            return False
        except Exception as e:

            print(f"Error checking device: {e}")
        
    async def pair_bluetooth_speaker(self):
        try:
            self.print('Scanning for Bluetooth devices...')
            devices = await BleakScanner.discover()

            if not devices:
                self.print('No devices found.')
                return False

            # Check if the target device is available
            found_device = next((device for device in devices if device.address == self.target_device), None)

            if not found_device:
                self.print(f"Target device with address {self.target_device} not found.")
                    
                return False

            self.print(f"Found target device: {found_device.name} [{found_device.address}]")

            # Attempt to connect persistently
            if not self.client:
                self.client = BleakClient(self.target_device)
            
            if not self.client.is_connected:
                await self.client.connect()
                self.print(f"Successfully connected to {found_device.name} [{found_device.address}]")
            else:
                self.print(f"Already connected to {found_device.name} [{found_device.address}]")

            return True

        except Exception as e:
            self.print(f"Error during pairing: {str(e)}")
            return False


    async def disconnect_client(self):
        """Safely disconnect the BleakClient."""
        try:
            if self.client and await self.client.is_connected():
                await self.client.disconnect()
                self.print('Disconnected from Bluetooth device.')
        except Exception as e:
            self.print(f"Error during disconnection: {str(e)}")
        finally:
            self.client = None  # Clean up the client reference

        

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

    # Create the executor
    executor = MultiThreadedExecutor()
    executor.add_node(tamir)

    try:
        executor.spin()
    except KeyboardInterrupt:
        tamir.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        tamir.disconnect_client()
        executor.shutdown()
        tamir.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()