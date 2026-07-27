# Coursework 1 template

This stack contains the code template for achieving the first coursework. Students must fill in code templates and submit the whole stack as part of the coursework submission.

## LSA package names

The LSA packages use unique ROS package and Python module names ending in `_lsa`, so they can be built in the same colcon workspace as the standard coursework packages. Shared robot dependencies retain their original names and are not duplicated or modified.

Build the CW1 LSA packages and their required package dependencies with:

```bash
colcon build --packages-up-to cw1q4_lsa cw1q5_lsa cw1q9_lsa
source install/setup.bash
```

## Run and launch commands

Run each command from the root of the colcon workspace after building and
sourcing the workspace as shown above.

Question 4 services:

```bash
ros2 run cw1q4_lsa cw1q4_services
```

Question 5b:

```bash
ros2 launch cw1q5_lsa q5b_launch.py
```

Question 5d:

```bash
ros2 launch cw1q5_lsa q5d_launch.py
```

Question 9 student node:

```bash
ros2 run cw1q9_lsa youbot_student_node
```

## PLEASE NOTE YOU MAY NOT USE LIBRARIES THAT ARE NOT INCLUDED WITH THIS RELEASE

`cw1q4_lsa` is a package for question 4 in the first coursework. Students should write a code to create two "ROS service" to convert rotation representations as stated in the instruction. The services are defined in `cw1q4_interfaces_lsa`.

`cw1q4_interfaces_lsa` contains the definition of the services used in the `cw1q4_lsa` package. Students should change the parameters in the two definitions as appropriate. Please do not change the names of the services as this will invoke an error.

`cw1q5_lsa` contains the code templates for question 5b and question 5d. The work is very similar to the examples shown during the lab sessions. The only difference is it is based on a different model of the KUKA youbot manipulator. There are three things to keep in mind when working on these questions:

1. Even if your code in cw1q5b is perfect, the frames you define will not align perfectly with the rviz model because the DH parameters are based on the simplified version. This discrepancy will not affect your marks in any way.
2. cw1q5c and cw1q5d require you to read the xacro file. It is basically a robot model for simulation, defining where robot parts, frames, links ond joints are in the model. This question may take you some time to work it out, but it is expected as it is the hardest question of this coursework.
3. The joint positions in the hardware interface and the ones that result from forward kinematics are usually similar and most of the time you are not required to account for offsets. However, this is not the case with the youbot manipulator. Please take a look at the "origin, rpy" and "limit" in the xacro file noted in question 5c and work out how to change the joint inputs accordingly. Without any modification, your robot arm may end up moving in the opposite direction or have an offset in the joint position you have not accounted for.

`cw1q9_lsa` contains the code templates for question 9.
