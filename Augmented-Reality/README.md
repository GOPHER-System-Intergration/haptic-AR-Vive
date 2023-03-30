# AR Interface for Assisting Remote Robot (KINOVA Gen3) Manipulation

![image](https://github.com/tclin0207/Haptic-AR/blob/main/Augmented-Reality/AR.jpg)

## Preparation
1. Plugin the RealSense camera (Tested on D435)
2. Connect KINOVA Gen3 to the system (via ethernet)
3. Print the ArUco markers from [here](https://chev.me/arucogen/), be aware of the Dictionary to **Original ArUco** and marker ID and size from launch file (Tested on Original-50mm).

## Installation (Dependency)
1. ArUco markers installation: https://www.sauravag.com/2015/03/how-to-tracking-aruco-marker-using-a-usb-webcam-and-ro/\]
2. RealSense2 package: https://github.com/IntelRealSense/realsense-ros

## Install ArUco markers ROS package
1. Create the catkin workspace (aruco_ws in our case) and cd into the source folder
```bash
cd aruco_ws/src/
```
2. Clone and install the package
```bash
git clone https://github.com/pal-robotics/aruco_ros.git
cd ..
catkin_make install
```
3. Install the realsense2 package (source: https://github.com/IntelRealSense/realsense-ros)
```bash
sudo apt-get install ros-melodic-realsense2-camera
```
4. Setting environment variables
```bash
source install/setup.bash
```
5. Edit the launch file
```bash
gedit src/aruco_ros/aruco_ros/launch/single.launch

<remap from=”/camera_info” to=”/camera/color/camera_info” />
<remap from=”/image” to=”/camera/color/image_raw” />
<param name="camera_frame" value="/camera_link"/>
```

## Launch the ArUco markers tracking
Open five terminals
1. Launch the ROS
```bash
roscore
```
2. Launch the realsense
```bash
cd realsense_ws/
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch
```
3. Launch the ArUco markers tracking
```bash
cd aruco_ws/
source devel/setup.bash 
roslaunch aruco_ros single_realsense.launch
roslaunch aruco_ros double_realsense.launch (for multi-markers detection)
```
4. Visualization
```bash
rosrun image_view image_view image:=/aruco_single/result
rosrun image_view image_view image:=/aruco_simple/result (for multi-markers detection)
```
5. Check the realtime marker pose
```bash
rostopic echo /aruco_single/pose
rostopic echo /aruco_simple/pose (for multi-markers detection, marker ID: 51)
rostopic echo /aruco_simple/pose2 (for multi-markers detection, marker ID: 101)
```

## Launch the UI (AR interface)
Open another terminal
```bash
cd your_worklspace/
python ar_interface.py
```

![image](https://github.com/tclin0207/Haptic-AR/blob/main/Augmented-Reality/AR_Interface.png)

