# ros2_manipulators
A collection of ROS2-based projects for robotic manipulators, including hardware integration, control algorithms, and simulation environments. This repository provides tools and resources for developing and testing various robotic arm systems in ROS2, making it ideal for both real-world applications and simulation-based research.

# ROS2 Humble Installation:
1. [Click For Ros2 installation manual: Binary Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. [Click for Ros2 installation manual: Building from Source](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

# Setup ROS2 IDE (*VS Code*)

1. VS Code extensions:
- C++ extension pack (*Microsoft*)
- Python (*Microsoft*)
- ROS (*Microsoft*)
- XML (*Red Hat*)
- XML Tools (*Josh Johnson*)

# C++ Linting
- Add `/opt/ros/humble/include/**` to your **C++ properties** include path.

# Installing Open Source Ros2 Packages
[Click to see all the available ros2 open source Packages](https://index.ros.org/repos/)

To install ros2 packages you can use the following commands e.g. `sudo apt install ros-<ros-distro>-<package_name>`.

We're going to install the following packages:
```bash
apt install ros-humble-joint-state-publisher-gui
apt install ros-humble-xacro
apt install ros-humble-ros-gz*
apt install ros-humble-gazebo-ros
apt install ros-humble-*-ros2-control
apt install ros-humble-ros2-controllers
apt install ros-humble-gazebo-ros2-control
apt install ros-humble-ign-ros2-control
apt install ros-humble-moveit*
apt install ros-humble-urdf-tutorial
```

Additional packages useful for our development

```bash
apt install libserial-dev # library used to communicate with the arduino through the serial port
pip install pyserial # library used to communicate with the arduino through the serial port

# Interface the robot with alexa assistant
pip install flask
pip install flask-ask-sdk
pip install ask-sdk
```
