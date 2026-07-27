# Coursework 2 Template

This is the student handout for coursework 2. The updated brief is in `MPHY0054_Robotic_Systems_Engineering_Coursework_2___Inverse_Kinematics__Path_Planning__Actuators__Mechanisms__and_Robot_Dynamics__ros2_.pdf`.

## LSA package names

The LSA packages use unique ROS package and Python module names ending in `_lsa`, so they can be built in the same colcon workspace as the standard coursework packages. Shared robot dependencies retain their original names and are not duplicated or modified.

Build the CW2 LSA packages and their required package dependencies with:

```bash
colcon build --packages-up-to cw2q2_lsa cw2q4_lsa cw2q7_lsa youbot_trail_rviz_cw2_lsa
source install/setup.bash
```

## Launch commands

Run each command from the root of the colcon workspace after building and
sourcing the workspace as shown above.

Question 2, including RViz and the trail visualiser:

```bash
ros2 launch cw2q2_lsa cw2q2.launch.py
```

RViz and the trail visualiser can be disabled when required:

```bash
ros2 launch cw2q2_lsa cw2q2.launch.py rviz:=false trail:=false
```

Question 4 validator:

```bash
ros2 launch cw2q4_lsa cw2q4_validate.launch.py
```

Question 7 simulation and trajectory node:

```bash
ros2 launch cw2q7_lsa cw2q7.launch.py
```

Question 7 can also be launched without RViz:

```bash
ros2 launch cw2q7_lsa cw2q7.launch.py rviz:=false
```

Standalone youBot trail visualiser:

```bash
ros2 launch youbot_trail_rviz_cw2_lsa trail.launch.py
```

- `cw2q2_lsa`: ROS 2 Foxy package for question 2. Edit `src/cw2q2_node.py`. Launch via `launch/cw2q2.launch.py`.
- `youbot_trail_rviz_cw2_lsa`: Optional RViz trail visualiser.
- `cw2q4_lsa`: Dynamics template; edit `src/cw2q4_lsa/iiwa14DynStudent.py`.
- `cw2q7_lsa`: Trajectory/acceleration template; edit `src/cw2q7.py` and use `launch/cw2q7.launch.py`.

Follow the PDF for task details. Keep the package structure intact.
