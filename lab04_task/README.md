# Lab 04: forward kinematics and TF2

This self-contained ROS 2 Humble package includes the OpenManipulator model,
meshes, RViz settings and a single-frame TF exercise. Copy this folder alone
into your workspace's `src/`. No solution or other course package is required.

The exercise follows the original Lab 04 handout: import the required ROS
types, name a custom frame, initialize a broadcaster and subscriber, construct
and broadcast a transform, and add the task node to the launch file. Only the
first DH frame is used. The DH transform and quaternion helper are provided.
The illustrative DH frame is not the URDF link frame with the same number.

## Student edits

Edit only the `BEGIN/END STUDENT CODE` regions:

| File | Region | Required result |
| --- | --- | --- |
| `src/lab04_task.py` | `ros_imports` | Import `JointState`, `TransformBroadcaster`, `TransformStamped` from their ROS modules. |
| Same file | `frame_names` | Set `name_link` to one custom child-frame name, such as `fkine_link_1`. |
| Same file | `broadcaster` | Create and retain `self.br`. |
| Same file | `subscription` | Create `self.subscription` for relative topic `joint_states` and callback `self.fkine_wrapper`. |
| Same file | `transform_message` | Assign a valid `TransformStamped` using `joint1` by name, the supplied DH model, parent `world`, child `name_link[0]`, and the input timestamp. |
| Same file | `broadcast_transform` | Publish the checked transform through your broadcaster. |
| `launch/lab04.launch.py` | `task_node` | Assign `student_node` to a launch `Node` for this package's `task` executable. Use the supplied `PACKAGE`, `namespace`, and `REMAP`. |

The `helpers` region is optional. Keep other imports and protected code intact.
No extra libraries are needed. The required ROS imports above are permitted.

## Build and run

If Humble is not sourced, first run `source /opt/ros/humble/setup.bash`.
From your workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch lab04_task lab04.launch.py
```

Rebuild and restart after source edits. Incomplete regions deliberately fail.
To explore the supplied robot before completing the task or launch region:

```bash
ros2 launch lab04_task lab04.launch.py start_task:=false
```

Default namespace: `/lab04_task`. Move `joint1` in the joint slider window;
your custom frame should follow the supplied DH model. With `joint1 = 0`, its
translation relative to `world` is `(0.012, 0, 0.075)` metres and its rotation
is +90 degrees around X. Other joints should not affect this single-frame task.

In another sourced terminal (replace the child name if you chose another):

```bash
ros2 topic echo /lab04_task/joint_states --once
ros2 run tf2_ros tf2_echo world fkine_link_1 --ros-args \
  -r /tf:=/lab04_task/tf -r /tf_static:=/lab04_task/tf_static
```

Use `gui:=false rviz:=false` for operation without display windows. The launch
then uses the non-GUI joint state publisher. Namespaced topics and TF prevent
a concurrently launched package from sharing this package's robot state.

A valid transform and a unit quaternion do not by themselves prove numerical
correctness. Compare several joint angles against the supplied DH transform.
