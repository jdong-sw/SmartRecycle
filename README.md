# Smart Recycling Robot
This is the comprehensive repository for the Smart Recycling project for WPI's RBE 595 ST: Deep Learning for Advanced Robot Perception.

The project uses ROS Noetic to launch a series of nodes to:
- Control the robotic manipulator
- Access the Intel RealSense D435 camera feed
- Perform object detection on incoming point cloud data

For the professor: you probably won't have ROS configured, or the D435 camera available, and definitely not my robotic manipulator connected, so you won't be able to actually run the ROS packages I've developed. You can, however, test my object detection model using a sample script. Refer to the Quick Test section.

## Quick Test
To only test the waste object detection model, start a python3 environment with PyTorch, Torchvision, and the typical numerical packages. You can also install the required packages using the requirements_basic.txt file by running the following command in your python virtual environment:

`pip install -r requirements_basic.txt`

Then open the "Inference for Testing.ipynb" notebook file. Run the first couple of cells to load the YOLOv5s model trained on my data. Then run any of the following cells to run the model on random included sample images, image of your choice, or live from your webcam.

## Installation

In order to setup your environment to run the full system, make sure you have ROS installed: (http://wiki.ros.org/ROS/Installation). Set up a catkin workspace and clone this repository into it. Finally, run `rosdep` to ensure you have the required packages.
- Create workspace folder "mkdir -p ros_ws/src" for example
- Source ROS: `source /opt/ros/<distro>/setup.bash`, replacing <distro> with your distribution of ROS
- Clone repository into /src folder: `gi clone git https://github.com/noodlephile/SmartRecycle.git
- Run rosdep `rosdep install --from-paths src --ignore-src -r -y`
- Install required python packages in your python virtual environment using: `pip install -r requirements.txt`

## Usage
There are a variety of launch files to launch different components of the system. To launch the full system including robot control, object detection, and visualization, just run:
```bash
roslaunch bumblebee bumblebee.launch
```
### Robot Control
To only run the Robot Control module to plan and execute trajectories with the robot, run:
```bash
roslaunch bb_control bb_control.launch bb_control:=true
```
If the robot is not connected, you can run a simulated robot with:
```bash
roslaunch bb_control bb_control.launch
```
If you wish to simulate the robot in Gazebo, you can run:
```bash
roslaunch bb_control bb_control.launch gazebo:=true
```

### Vision Module
To launch the full Vision Module, with PointCloud2 topics, YOLO object detection, 3D bounding box visualization in RViz, run:
```bash
roslaunch bb_perception bb_perception.launch
```
