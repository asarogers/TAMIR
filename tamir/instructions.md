1. Install packages and create virtual enviroment
./install_dependencies.sh 

2. Source changes
source ~/.bashrc

3. launch application
colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch tamir startup.launch.py


OR

1. Create a virtual enviroment
```
python3 -m venv myenv
```
2. Add this or simialr to your ~/.bashrc file
export PYTHONPATH="/home/asaace00/repo/winter/project/TAMIR/tamir/myenv/lib/python3.12/site-packages:$PYTHONPATH"

basically, you just need a reference to your virtual enviroment's python3.12-site-packages attached to your PYTHONPATH

3. Source your enviroment
source myenv/bin/activate

4. install packages
pip3 install -r requirements.txt

sudo apt install ros-jazzy-apriltag-ros

5. run this:
colcon clean workspace -y && colcon build && source install/setup.bash && ros2 launch tamir startup.launch.py


***get apriltags working on the raspberry pi:***
1) inside your workspace(the one your ros2 project is inside), create a new directory and clone the following repos
mkdir support
cd support
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/christianrauch/apriltag_ros.git
git clone https://github.com/christianrauch/apriltag_msgs.git

2) go back to the workspace directory
cd ..

build and source
colcon build && source install/setup.bash

3) launch the application

**to get the realsense2 camera to work on raspberry pi 5**
Before launching ROS, check if the camera streams outside ROS.
rs-enumerate-devices

If rs-enumerate-devices fails to detect the camera, restart the driver:
sudo systemctl restart udev
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo


If rs-enumerate-devices fails to detect the camera, restart the driver:
sudo apt update
sudo apt install ros-jazzy-realsense2-camera


Option 2: Test with realsense-viewer
If you have Intel's RealSense SDK installed, try:
realsense-viewer

The camera might be disconnecting due to USB power-saving settings. You can disable auto-suspend with:
echo 'options usbcore autosuspend=-1' | sudo tee /etc/modprobe.d/usb-autosuspend.conf
sudo update-initramfs -u
sudo reboot





