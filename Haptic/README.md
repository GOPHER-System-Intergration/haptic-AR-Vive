# Haptic Feedback for Assisting Remote Robot (KINOVA Gen3) Manipulation

![image](https://github.com/tclin0207/Haptic-AR/blob/main/Haptic/Haptic.jpg)

## Preparation
1. Completed the setup for [Control Interface](https://github.com/GOPHER-System-Intergration/haptic-AR-Vive/tree/main/Control-Interface)
2. Replace the udp_server.cs for haptic

## Launch the KINOVA robotic arm and UDP server
In the first terminal, launch the KINOVA robotic arm
```bash
cd kinova_ws/
source devel/setup.bash
roslaunch kortex_driver kortex_driver.launch
```
In the second terminal, launch the socket connection with distance publisher (change the IP address accordingly)
```bash
cd kinova_ws/
source devel/setup.bash
cd src/ros_kortex/kortex_examples/src/full_arm/
python sockect_connection_haptic.py
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
