# MPHY0054_lab
Collection of packages for lab sessions in COMP0127 Robotic Systems Engineering.

## Lab Manuals
The markdown (.md) versions of the .pdf lab manuals available on moodle are located in the [lab_manuals](https://github.com/surgical-vision/comp0127_lab/tree/master/lab_manuals) folder.

## Ubuntu 20.04 and ROS 1 Noetic Setup

A pdf describing setup instructions for Ubuntu 20.04 and ROS noetic can be found on moodle.

Note:
This course is designed using **ROS 1 noetic** (a.k.a. ROS noetic) in a **Ubuntu 20.04** environment.
Newer releases of Ubuntu (e.g. Ubuntu 22.04 or Ubuntu 23.04) are **NOT** compatible with ROS 1.
Attempting coursework using ROS 2 distributions will result in compatability issues.

Ubuntu 20.04 releases available for download at:
https://releases.ubuntu.com/focal/

If you do not have Ubuntu 20.04 as your default OS, it is recommended that you set up Ubuntu 20.04 in a virtual environment. Instructions for setting up Ubuntu 20.04 are available on moodle.

In a Ubuntu 20.04 environment, install ROS noetic by following the installation instructions available at:
http://wiki.ros.org/noetic/Installation/Ubuntu


## MPHY0054_lab Dependencies

Once ROS noetic is installed, run the following code in a Ubuntu command terminal:
```
sudo apt install ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-gazebo-ros-control ros-noetic-joint-trajectory-controller ros-noetic-velocity-controllers ros-noetic-ros-controllers ros-noetic-ros-control
```

References:
- youbot_description: 'https://github.com/youbot'
- manipulator_h_description: 'https://github.com/ROBOTIS-GIT/ROBOTIS-MANIPULATOR-H'
- open_manipulator_description: 'https://github.com/ROBOTIS-GIT/open_manipulator'


## Debugging

This section provides comprehensive information on debugging your workspace, addressing any issues that may have arisen during the lab, and reviewing reported errors and problems.

### Issue 1: "usr/bin/env: ‘python3\r’: No such file or directory"

**Problem:** You encounter the error message `/usr/bin/env: ‘python3\r’: No such file or directory`.

**Solution:** To resolve this issue, follow these steps:

1. Open your terminal.

2. Run the following command to install the `dos2unix` utility:

   ```bash
   sudo apt install dos2unix
   ```

More info on the issue can be found [here](https://askubuntu.com/questions/896860/usr-bin-env-python3-r-no-such-file-or-directory).
