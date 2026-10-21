# Check your Lab 03 work

This folder contains everything needed for this exercise, including its service
definition and self-check tool. No solution package is required.

Place this folder in your workspace's `src/` directory. If Humble is not already
sourced in your terminal, first run `source /opt/ros/humble/setup.bash`.
After filling the marked regions, run from your workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch lab03_task lab03task.launch.py
```

This starts your server and client. Inspect the requests and responses in that
terminal. To run the separate self-check, keep the launch running and use a
second terminal from the same workspace root:

```bash
source install/setup.bash
ros2 run lab03_task check
```

The tool checks 27 known server inputs: identity, positive/negative quarter
turns, half turns, mixed-sign axes, near-zero/near-half-turn rotations and
reproducible random rotations. It checks finite, unit-length quaternions and
whether each output represents the requested rotation. Both `q` and `-q` are
accepted. The maximum permitted matrix-entry error is `1e-5`.

It then starts your installed client in a temporary namespace, checks three of
its requests, and forwards them to your server. Requests must contain nine
finite row-major entries forming an orthogonal matrix with determinant +1.
Each server reply is checked before being returned to your client. Only the
client process started by the checker is stopped when the check finishes.

Each case prints `PASS` or `FAIL`, followed by a final `RESULT`. A failure shows
the input or the relevant error. An unavailable service usually means the
server is not running, its name is wrong, or an unfinished region raised
`NotImplementedError`. Rebuild after changing `server.py` or `client.py`, then
restart the server and rerun the check.

If the client is still unfinished, stop the normal launch and start only the
server instead:

```bash
ros2 launch lab03_task lab03task.launch.py start_client:=false
```

Then, in the second sourced terminal, check only the server:

```bash
ros2 run lab03_task check --server-only
```

For a custom server namespace, use `--service /your_namespace/rotmat2quat`.
The client must use the relative service name `rotmat2quat` so ROS namespace
remapping works. Increase `--client-timeout 30` if your client sends requests
more slowly than the supplied three-second loop.

Passing these finite samples is useful evidence, not a proof for every input.
The tool checks requests and service replies, but does not grade your client's
printed explanation or verify that its random samples cover every rotation.
Also inspect your client's output in the normal launch terminal.
