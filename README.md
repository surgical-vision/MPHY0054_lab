# MPHY0054_lab

Repository for lab sessions in MPHY0054 Robotic Systems Engineering.

The previous year's lab, coursework, robot-description, and simulation packages
have been removed. Updated packages will be added for the environment below.

## Ubuntu 22.04 and ROS 2 Humble Hawksbill Setup

The course targets **Ubuntu 22.04 LTS (Jammy Jellyfish)** and
**ROS 2 Humble Hawksbill**.

Install ROS 2 Humble using the
[official Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Choose the desktop installation, then source Humble in each terminal used for ROS:

```bash
source /opt/ros/humble/setup.bash
```

If you previously configured another ROS distribution in `~/.bashrc`, replace
its setup line with the Humble setup line above.

## Download from GitHub

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/surgical-vision/MPHY0054_lab.git
```

This checkout currently contains no ROS packages to build or run.

## Update to the Latest Version

```bash
cd ~/ros2_ws/src/MPHY0054_lab
git pull
```

## Development Tools and Common Dependencies

After installing ROS 2 Humble and configuring its apt repository:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions \
                 python3-rosdep \
                 python3-numpy \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-gazebo-ros-pkgs
```

Package-specific dependencies and launch instructions will accompany the updated
lab and coursework packages when they are added.
