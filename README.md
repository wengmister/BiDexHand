# Biomimetic Dexterous Hand
Author: [Zhengyang Kris Weng](https://wengmister.github.io/)

Open source release of biomimetic dexterous robotic hand, a 10-week M.S.Robotics 2025 Winter project. This guide will help you build your own hand and get your started on the setup.

[![Biomimetic Dexterous Hand Demonstration](https://img.youtube.com/vi/X8zVKlZNorc/0.jpg)](https://youtu.be/X8zVKlZNorc?si=lmVslFvECZyih0Kd)

# Overview

This is the open source release of the MSR Dexterous Hand V3, a robotic hand featuring 16 degrees of freedom. It utilizes a cable-and-pulley system, with 15 servos arranged in `N configuration` to drive its 15 joints with tendon.

Each finger provides three degrees of freedom: metacarpal (MCP) adduction/abduction, MCP flexion/extension, and proximal interphalangeal (PIP) flexion/extension. A custom four-bar linkage at the distal end of each phalanx converts the PIP motion into a coupled movement at the distal interphalangeal (DIP) joint. The thumb is designed with four degrees of freedom, including carpometacarpal (CMC) adduction/abduction and flexion/extension, as well as MCP adduction/abduction and flexion/extension.

The hand is controlled through provided `ROS2` packages found in `/src`. It provides several modes to interface with the robot - `motion shadowing` and `servo input` streaming through `ROS2` topic, or direct `servo input` control through CLI. See packages in `/src` for more details.

# Hardware Setup

STEP file for the cad asset can be found under `/cad_asset/_stp`, and individual STL file under `/cad_asset/_stl`.

See [BOM.md](/BOM.md) for more details.

Build instruction currently under construction, I'm planning on releasing it soon.

For `V3`, flash and deploy `/scripts/esp32_multi_servo_control (or _feather)` through `Arduino IDE`.

# Environment Setup

This project currently runs on `ROS2-JAZZY`. To build locally, run:

    git clone https://github.com/wengmister/BiDexHand.git
    cd BiDexHand
    rosdep install --from-paths src -y --ignore-src

Finally, 

    colcon build
    . install/setup.bash


# Quickstart
### Hand Control

For motion shadowing:

    ros2 launch hand_motion_shadowing shadowing.launch.xml usb:=/dev/ttyACM0

For direct servo control:

    ros2 launch hand_servo_control multi_servo_control.launch.xml usb:=/dev/ttyACM0

Change usb port based on your device setting.

For Franka `MoveIT!` config demo:

    ros2 launch combined_fer_moveit_config demo.launch.py

### Franka Integration

For deploying on real Franka Fer, copy and build the following packages to your robot `station`:
- hand_rviz
- combined_fer_moveit_config

On station, run:

    ros2 launch combined_fer_moveit_config real.launch.py use_rviz:=false robot_ip:=[YOUR_ROBOT_IP]

On your laptop, run:

    ros2 launch combined_fer_moveit_config moveit_rviz.launch.py robot_ip:=[YOUR_ROBOT_IP]


# Demo

### Mixed Reality Motion Shadowing      
<img src="images/vr_control_exp.gif" alt="MR" width="500px">

### Franka FER Integrated    
<img src="images/franka_integration.gif" alt="Franka" width="500px">

# License
MIT