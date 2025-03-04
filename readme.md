# BlueROV2 Heavy ROS2 Setup Guide

## Installation of Linux/ROS2 on Laptop

1. Install **Ubuntu 22.04 LTS** (Recommended dual boot setup).
2. Install **ROS2 Humble**.
3. Install MAVROS and MAVLink:
   ```bash
   sudo apt-get install ros-humble-mavros ros-humble-mavlink ros-humble-mavros-extras ros-humble-mavros-msgs
   ```
4. Install Joy package:
   ```bash
   sudo apt-get install ros-humble-joy
   ```
5. Install Python dependencies:
   ```bash
   sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
   ```
6. Install GeographicLib datasets:
   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
   chmod u+x install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```
7. Install additional packages for 3D transformations:
   ```bash
   sudo apt-get install ros-humble-tf-transformations
   sudo pip3 install transform3d
   ```
8. Install video stream dependencies:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo apt-get remove modemmanager -y
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
   sudo apt install libfuse2 -y
   sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y
   ```
9. Install RViz2 (optional):
   ```bash
   sudo apt-get install ros-humble-rviz2
   ```
10. Create ROS2 workspace:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
11. Download and run QGroundControl:
    ```bash
    chmod u+x ./QGroundControl.AppImage
    ./QGroundControl.AppImage
    ```
12. Install joystick calibration tool:
    ```bash
    sudo apt-get install jstest-gtk
    ```
13. Install PingViewer for sonar:
   downlod [ping-viwer](https://github.com/bluerobotics/ping-viewer/releases/latest/download/pingviewer-Release.AppImage)
    ```bash
    chmod u+x ping-viewer-linux.AppImage
    ```

## Installation of ROS2 Packages

1. Copy `packs_marine_mech.tgz` into `ros2_ws/src`.
2. Extract the file:
   ```bash
   tar xzvf packs_marine_mech.tgz
   ```
3. Install Ping Sonar Python package:
   ```bash
   cd ~/ros2_ws/src/ping_sonar_ros-master/ping_sonar_ros/ping-python
   python3 setup.py install --user
   ```
4. Compile the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
5. Install the compiled packages:
   ```bash
   source install/setup.bash
   ```
   (You can add this command to `.bashrc` for automatic sourcing.)

## Network Configuration and Connection to BlueROV2

1. Connect the **yellow cable** of the robot to the bluebox and the **USB cable** of the bluebox to the laptop.
2. Create a **USB/Ethernet** network connection:
   - Name: `BR2`
   - Mode: `Manual`
   - Local IP (Laptop): `192.168.2.1`
   - Netmask: `255.255.255.0`
3. Power on the robot and wait for **boot completion (~1 min)**.
4. Access BlueOS interface via browser at `192.168.2.2`.

## Calibration

### IMU Calibration
1. Open **QGroundControl**.
2. Navigate to **Sensors** and calibrate:
   - Accelerometer (`Roll90` orientation)
   - Compass (`Roll90` orientation)
   - Pressure Sensor

### Gamepad Calibration
1. Connect gamepad to laptop.
2. Configure in **QGroundControl** under Vehicle Settings:
   - `RC Mode`: `3` (For Logitech F310 Gamepad)
   - Enable **Center Stick is Zero Throttle** and **Allow Negative Thrust**.
   - Adjust **Deadbands**.
3. If not using QGroundControl, use `jstest-gtk` for joystick calibration.

### Motor Configuration
- Run **Automatic Motor Direction Detection** in **QGroundControl**.
- Keep clear of thrusters when testing.

### Sonar Pinger Setup
1. Run PingViewer:
   ```bash
   ./pingviewer.AppImage
   ```
2. Adjust sonar parameters in `ping1_components.py`.
3. Recompile and restart sonar node:
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch ping_sonar_ros run_pinger.launch
   ```

## Running ROS2 Nodes

### Start MAVROS Node
```bash
ros2 launch autonomous_rov run_mavros.launch
```

### Start Listener and Joystick Nodes
```bash
ros2 launch autonomous_rov run_listener_MIR_joy.launch
```
(NOTE: If the gamepad is not detected, update `joy_dev` path in launch file.)

### Start Camera Node
```bash
ros2 launch autonomous_rov run_video.launch
```

### Start Sonar Pinger Node
```bash
ros2 launch ping_sonar_ros run_pinger.launch
```

### Check ROS2 Topics and Nodes
```bash
ros2 topic list
ros2 node list
ros2 topic echo /bluerov2/ping1d/data
```
(NOTE: IMU feedback runs at 25 Hz, sonar at 10 Hz.)

## Shutdown Procedure
Use **BlueOS** in the browser to shut down the robot properly.

## Troubleshooting
- Check BlueRobotics documentation: [BlueROV2 Software Setup](https://bluerobotics.com/learn/bluerov2-software-setup/)
- Default Raspberry Pi password: `raspberry` (`ssh pi@192.168.2.2`)
- If using Conda, ensure it does not conflict with ROS2 environment.
- If Chrome does not connect to `192.168.2.2`, try Firefox.

---

This guide provides step-by-step instructions for setting up **BlueROV2 Heavy** with **ROS2** on a laptop, including network configuration, calibration, and troubleshooting. For any issues, refer to BlueRobotics documentation or community forums.
