# lab07_task assignment

Student-facing version of lab07. The stack (URDF parsing, RViz launch, controller spawners) is identical to `lab07_solution`, but two key parts are left for you to implement inside `lab07_task/lab07_task.py`.

## TODOs

1. **Checkpoint markers**  
   - `_run_once` already allocates a `(4, 4, N)` array named `checkpoints`.  
   - Use `self.forward_kinematics` on every column of `self.joint_targets` to fill this array.  
   - Store it in `self._checkpoint_data`, call `publish_checkpoints`, and enable `_animation_active`.
   - In `publish_checkpoints`, iterate over the transforms and publish `Marker` messages on `self.checkpoint_pub`.
   - `_republish_markers` should reuse your `publish_checkpoints` implementation to re-send cached markers.

2. **Trajectory publisher**  
   - Implement `publish_trajectory`, constructing a `JointTrajectory` message that contains all waypoints.  
   - Each waypoint should have the correct `time_from_start` so the trajectory controller executes a smooth motion.  
   - Publish the message via `self.traj_pub`.

Once both parts are complete, `ros2 launch lab07_task lab07_task.launch.py` should show checkpoints and a moving robot in RViz.
