# rover_ws, an example with known faults

A small ROS 2 jazzy workspace with faults planted on purpose, so `rosio` has something true to
say about it. Build it, then extract:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
rosio extract . --config rosio.yaml
rosio drift
```

What is wrong in it, for checking what rosio reports:

- `config/rover.yaml` sets `stop_distnce` on `safety_stop`, which declares `stop_distance`.
- `config/rover.yaml` sets `max_range: 8.0` and the launch file passes `max_range: 10.0` after
  the file, so the yaml line never reaches the node.
- `config/rover.yaml` has a block for `odometry`, a node nothing starts.
- `scan_filter` looks up `laser_frame`, a frame the robot model does not have (`laser_link`).
- `scan_filter` publishes `/scan_filtered` best effort, `safety_stop` subscribes to it reliable,
  so no message ever arrives.
