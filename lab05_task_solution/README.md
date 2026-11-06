# Lab 05: URDF and KDL

This self-contained ROS 2 Humble package includes the YouBot and OpenManipulator
models, meshes, a small URDF example, KDL examples, and the YouBot exercise.
Copy this folder alone into your workspace's `src/`. No solution or other
course package is required. NumPy and PyKDL are provided course dependencies.

URDF specifies links, joints, origins, axes and limits. The models under
`models/` are expanded from the course Xacro descriptions for standalone use.
PyKDL represents the resulting chain as segments and joints and evaluates FK.

## Student edits

The original Lab 05 handout asks for three missing parts in `src/lab05_task.py`:

| Region | Required result |
| --- | --- |
| `main` | Initialize rclpy with the supplied arguments, create `node` as a `YoubotKDLNode`, and spin it. Cleanup is provided. |
| `urdf_chain` | Assign a five-joint KDL chain from the YouBot base to its end-effector. Use the provided URDF parser and inspect the model for the link names. |
| `parent_frame` | Set the transform's parent to the frame in which that chain's FK is expressed. |

Complete only marked regions; keep imports, checks and returns intact.
`dh_comparison_parameters` is an optional extension with arbitrary initial
values. It is not the correct YouBot DH table and is not expected to match the
URDF model. Deriving a matching YouBot DH table remains a coursework exercise.

The completed OpenManipulator program `src/lab05_example.py` and the small
`models/my_robot.urdf` are provided worked examples in both handouts. They are
not copies of the YouBot task answer. The OpenManipulator example models four
revolute arm joints and a prismatic gripper, including the fixed mounting offset.

## Build and run

If Humble is not sourced, first run `source /opt/ros/humble/setup.bash`.
From your workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch lab05_task_solution lab05.launch.py
```

Rebuild and restart after source edits. Default mode is the YouBot task;
unfinished regions deliberately fail. The supplied examples run before you
complete the task:

```bash
ros2 launch lab05_task_solution lab05.launch.py mode:=robot
ros2 launch lab05_task_solution lab05.launch.py mode:=example
```

Run one mode at a time and stop it with Ctrl+C before changing modes.

| Mode | What to observe |
| --- | --- |
| `robot` | Small three-revolute-joint URDF; move the shoulder, elbow and wrist sliders. |
| `example` | OpenManipulator; `urdf_link` and `dh_link` should coincide, including when the gripper slides. |
| `task` | YouBot; your `urdf_link` should coincide with `arm_link_ee`. The arbitrary `dh_random_link` generally does not coincide. |

Default namespace: `/lab05_task_solution`. To compare the YouBot frames in another
sourced terminal:

```bash
ros2 run tf2_ros tf2_echo arm_link_ee urdf_link --ros-args \
  -r /tf:=/lab05_task_solution/tf -r /tf_static:=/lab05_task_solution/tf_static
```

For a correct completed task, this relative transform is identity at every
joint configuration: zero translation and quaternion `(0,0,0,1)` or its negative.
Allow small floating-point error. Check several slider configurations; launch
success alone does not prove the transform is correct.

For the provided OpenManipulator example, compare `urdf_link` with `dh_link`
in the same way. Use `start_task:=false` to display only the robot, or
`gui:=false rviz:=false` to use the non-GUI joint state publisher without windows.
Topics, including TF and robot_description, are isolated under the namespace.
