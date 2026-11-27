# Lab 07: Joint trajectories and Cartesian checkpoints

Ubuntu 22.04 / ROS 2 Humble. This package contains its own YouBot model, meshes,
helpers and RViz settings. Copy this folder alone into your workspace's `src/`.
No coursework, other lab, or solution package is needed.

## Student edits

Edit only the regions bounded by `BEGIN/END STUDENT CODE`. Keep the provided
imports, assertions and returns intact. The `helpers` region is optional.

Complete the four original exercise regions in `src/lab07_task.py`:

- `load_targets`: copy the two supplied joint vectors into `self.joint_targets`, shape `(5,2)`, one waypoint per column.
- `checkpoints`: assign their FK transforms to `checkpoints`, shape `(4,4,2)`, using `self.model.forward_kinematics`.
- `publish_checkpoints`: call the provided `self.publish_checkpoints(checkpoints)`.
- `trajectory`: assign `traj` as a `JointTrajectory` with joint names `JOINT_NAMES`, both waypoints in order, and `time_from_start` equal to 12 s and 14 s.

Set the trajectory header timestamp to the current ROS time, or zero to start on
receipt. Protected code validates and publishes the completed message.
The tip used for this exercise is `arm_link_5`, matching the existing ROS 2 lab.
Two colored markers show the Cartesian checkpoints.

The supplied `preview` executable subscribes to the actual `joint_trajectory`
message and interpolates its waypoints into `joint_states`. The arm starts at
zero, reaches the first waypoint at 12 s, then the second at 14 s and holds.
This is a kinematic RViz preview, not a Gazebo simulation or a dynamics controller.
It requires no controller manager or other course packages.

In another sourced terminal, inspect the actual message:

```bash
ros2 topic echo /lab07_task_solution/joint_trajectory --once --qos-durability transient_local
```

Check names, positions and times, as well as the visible robot motion. The
trajectory publisher retains its message so a later subscriber can receive it.

## Build and launch

From your workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lab07_task_solution
source install/setup.bash
ros2 launch lab07_task_solution lab07.launch.py
```

With only this package in `src/`, plain `colcon build` also works. Selecting the
package avoids attempting to build unfinished coursework. Rebuild after edits,
source again and restart the launch. The installed Python entry point is `task`.

Unfinished student regions deliberately stop with an error. To view the model first:

```bash
ros2 launch lab07_task_solution lab07.launch.py start_task:=false
```

For operation without windows, add `gui:=false rviz:=false`. Stop with Ctrl+C.
All topics, including TF, are isolated under `/lab07_task_solution` by default.
The `namespace` launch argument can change that prefix.
