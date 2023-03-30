# KINOVA Motion Mapping Teleoperation Interface (Windows/Unity + Ubuntu/ROS)

## Overview
Control input: HTC Vive controller

Visual device: RealSense

Communication: UDP

Windows/Unity: publish the data that received from HTC Vive controllers

Ubuntu/ROS: subscribe the data from Unity to control the KINOVA arm

## Installation (Dependencies)
1. ROS package for KINOVA: https://github.com/Kinovarobotics/ros_kortex
2. ROS package for RealSense: https://github.com/IntelRealSense/realsense-ros#installation-instructions

## Unity Setup
1. Install Steam/Steam VR
2. Create a new Unity project
3. In the main screen: Window -> Package Manager -> My Asset -> Install Steam VR Plug in
4. Under Project -> Under Assets -> SteamVR -> Prefabs -> Drag [CameraRig] under Hierachy
5. Window -> SteamVR Input -> add Grip, Menu, System, TriggerBtn, TrackpadBtn as boolean and TriggerAxis as Vector 1 and TrackpadAxis as Vector 2
6. Then click "Open binding UI" to setup the controller configuration and click "Save and generate"
7. Click "Camera" under Hierachy -> Add component -> create udp_server script (paste the code from udp_server.cs)
8. Then setup the corresponding sources for Target Camera, Left Controller, Right Controller, Cube and Haptic Action

## Preparation
1. Plugin the Vive system (headset and two controllers)
2. Connect the KINOVA via Ethernet (make sure to setup the Internet Protocol Version 4 (TCP/IPv4), 192.168.1.11 for IP address and 255.255.255.0 for subnet mask)
3. Open Steam
4. Open SteamVR
5. Open Unity and play it
6. Installed realsense package

## Launch the KINOVA robotic arm and UDP server
In the first terminal, launch the KINOVA robotic arm (prereques: KINOVA ROS package installed)
```bash
cd kinova_ws/
source devel/setup.bash
roslaunch kortex_driver kortex_driver.launch
```
In the second terminal, launch the socket connection (change the IP address accordingly)
```bash
cd kinova_ws/
source devel/setup.bash
cd src/ros_kortex/kortex_examples/src/full_arm/
python sockect_connection.py
```
## Stream realsense camera feed to Unity
In the third terminal, launch the realsense
```bash
roslaunch realsense2_camera rs_camera.launch 
```
In the fourth terminal, launch the streaming (change the IP address accordingly)
```bash
cd your_workspace/
python ui_cam_stream.py
```
## Lauch the Vive teleoperation interface
In the fifth terminal, launch the vive interface
```bash
cd kinova_ws/
source devel/setup.bash
cd src/ros_kortex/kortex_examples/src/full_arm/
python Vive_teleoperation.py
```
