# Lab 06: Jacobian with KDL

Ubuntu 22.04 / ROS 2 Humble. This package contains its own YouBot model, meshes,
helpers and RViz settings. Copy this folder alone into your workspace's `src/`.
No coursework, other lab, or solution package is needed.

## Student edits

Edit only the regions bounded by `BEGIN/END STUDENT CODE`. Keep the provided
imports, assertions and returns intact. The `helpers` region is optional.

Complete four regions in `src/lab06_task.py`:

- `jacobian_subscription`: create `self.sub_jac` on relative topic `joint_states`, using `print_jacobian`.
- `jacobian_solver`: create `self.jac_calc` for `self.model.chain`.
- `print_jacobian`: use the five arm joints by name and assign `jacobian`.
- `get_jacobian`: assign the KDL geometric Jacobian as a finite real NumPy `(6,5)` array.

`joint_values`, `to_kdl`, and `to_numpy` are supplied helpers. The Jacobian has
linear-velocity rows first, angular-velocity rows second, expressed in `base_link`,
for the endpoint `arm_link_ee`. FK and TF broadcasting are supplied.
The console prints the Jacobian initially and whenever it changes. Move the joint
sliders to inspect it; gripper values must not be used as arm joint angles.
The `urdf_link` frame should coincide with `arm_link_ee`.

## Build and launch

From your workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lab06_task_solution
source install/setup.bash
ros2 launch lab06_task_solution lab06.launch.py
```

With only this package in `src/`, plain `colcon build` also works. Selecting the
package avoids attempting to build unfinished coursework. Rebuild after edits,
source again and restart the launch. The installed Python entry point is `task`.

Unfinished student regions deliberately stop with an error. To view the model first:

```bash
ros2 launch lab06_task_solution lab06.launch.py start_task:=false
```

For operation without windows, add `gui:=false rviz:=false`. Stop with Ctrl+C.
All topics, including TF, are isolated under `/lab06_task_solution` by default.
The `namespace` launch argument can change that prefix.
