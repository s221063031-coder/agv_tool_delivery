# AGV Tool Delivery Project - Factory Simulation

This repository contains the ROS 2 workspace for an Automated Guided Vehicle (AGV) designed for autonomous tool delivery.

# 1. Building the Environment (The Workspace)
# Translates URDF/Python files into executables. 
# The --symlink-install allows Xacro changes to update without rebuilding.
```
cd ~/agv_ws
colcon build --symlink-install
source install/setup.bash

```
# 2. Launching the Simulation (Gazebo)
# Creates the physical world and robot body. 
# use_sim_time:=true syncs the robot brain to the Gazebo clock for LiDAR accuracy.
```
ros2 launch agv_factory_sim gazebo.launch.py use_sim_time:=true

```
# 3. Activating the "Brain" (SLAM Toolbox)
# Starts the mapping logic. 
# It links the Map to the Odom by listening to the /scan topic from your URDF.
```
rros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

```
# 4. Visualization Logic (RViz)
# Setup: Inside RViz, manually change the "Fixed Frame" to 'map'.
# Sets the factory floor as the (0,0,0) center of the universe.
```
rviz2

```
# 5. Checking the "Nervous System" (TF Tree)
# Generates a PDF proving Map, Odom, Base_Link, and Laser are connected.
# If this tree is broken, the robot is "blind."
```
ros2 run tf2_tools view_frames

```
# 6. Manual Control (Teleop)
# Use U-I-O / J-K-L / M-,-. keys to drive the AGV.
# This tests the Differential Drive plugin and motor controllers 
# independently of the autonomous navigation logic.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 📐 Robot & Environment Design

### 🤖 AGV Robot Design (URDF)
The physical structure, joint limits, and sensor placements (LiDAR) are defined in:
* **File:** `src/agv_factory_sim/urdf/tool_delivery_agv.urdf`
* **Features:** * Differential Drive plugin for motion control.
    * LiDAR sensor link for obstacle detection.
    * 3D visual and collision meshes.

### 🏭 Factory Environment (Gazebo World)
The custom factory layout with walls and delivery stations is defined in:
* **Folder:** `src/agv_factory_sim/worlds/`
* **Primary World:** `kilang_layout.world`
* **Features:** Realistic lighting, static obstacles, and localized coordinate systems.

### 🗺️ Navigation Map (SLAM)
The 2D occupancy grid used for the A* path planning logic is located in:
* **Folder:** `src/agv_factory_sim/maps/`
* **Files:** `.yaml` (metadata) and `.pgm` (occupancy grid image).
