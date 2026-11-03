# Reviewing and changing ROS 2 code

Read this before you hunt for bugs in a ROS 2 repository, review a change to one, or change
robot code yourself.

## How to work

1. Start from the system. `rosio drift --min-severity warn` lists configuration defects that
   are already checked, each with the file and line to edit. Read them before any source.
2. Find candidates cheaply. Search for the patterns below (`rg -n`, `grep -rn` or a search
   tool), then read only the function around each hit. Reading whole packages costs tokens
   and finds less.
3. A candidate is a bug only when you can name the input that breaks it and the wrong result
   it gives. Read every caller, the message definition and the REP or API docs first. Check
   `git log -S '<line>'` for a reason the line is written that way.
4. Fix at the root with the smallest diff, then search for the same mistake in sibling code.
5. Do not report style, missing features, unused code, or a pointer that "could be null"
   without a path that makes it null.

## Where real bugs are, most common first

**Copy and paste.** Two near identical blocks where the second still uses the first's name:
`x` for `y`, `min` for `max`, `left` for `right`, `roll` for `pitch`, `linear` for
`angular`, `msg->a` checked and `msg->b` used. Read paired blocks side by side.

**Units and angles.**
- Degrees and radians: `rg -n 'M_PI|180\.0|deg|rad' --type cpp --type py`. Check every
  conversion is applied exactly once.
- Wrap: a difference of two yaws needs `angles::shortest_angular_distance` or a normalize to
  [-pi, pi]; a plain subtraction jumps by 2 pi.
- `atan2(y, x)`: the order is y first.
- Rates and periods: `1 / rate` with an integer rate is 0; `std::chrono::milliseconds(1.0 /
  hz)` truncates. Seconds, milliseconds and nanoseconds mixed in one expression.

**C++ numerics.**
- `abs(x)` on a double without `std::` can resolve to the int overload and truncate:
  `rg -n '[^:f]abs\(' --type cpp`.
- `v.size()-1` on an empty container wraps to a huge unsigned value; a `size_t` loop counting
  down with `i >= 0` never ends.
- Division by a value that can be zero: `dt` on the first message, a norm, a count.
- Integer division where a fraction was meant; members left uninitialized by a constructor.

**Frames and transforms** (REP 103, REP 105).
- Body frames: x forward, y left, z up. Optical frames (`*_optical_frame`): z forward,
  x right, y down. A camera mounted without the optical rotation sees the world sideways.
- One parent per frame. `map -> odom -> base_link`; odometry publishes `odom -> base_link`
  only, localization publishes `map -> odom`.
- Quaternion order: ROS messages and tf2 are (x, y, z, w); `Eigen::Quaterniond(w, x, y, z)`
  takes w first. `setRPY(roll, pitch, yaw)`. Normalize after arithmetic.
- A transform looked up at `TimePointZero` (latest) for data stamped earlier mixes two
  moments; a moving robot then smears its sensors.
- A hard coded `frame_id = "..."` where a parameter or the incoming header names the frame.
- `rosio drift` checks frames the code stamps against the robot model and duplicate
  broadcasters: `frame.not_in_urdf`, `frame.broadcast_twice`.

**Time.**
- `rclcpp::Clock()` and `std::chrono::system_clock` ignore `use_sim_time`; `node->now()`
  and `get_clock()` follow it. In python, `time.time()` against `self.get_clock().now()`.
- Subtracting two `rclcpp::Time` of different clock types throws.
- A header stamp left at zero, or set to `now()` when the input message carried its own
  stamp.
- `dt` zero or negative on the first message, after a pause, or when a simulation resets.
- rclpy `Time.nanoseconds` is an int; `Duration(seconds=...)` and `Duration(nanoseconds=...)`
  are different arguments.

**Parameters.** `rosio param <node> <name>` gives the value that reaches the node.
- Read without a declaration: `get_parameter` throws `ParameterNotDeclaredException`
  (`param.read_undeclared`).
- A yaml key nothing declares does nothing (`param.set_undeclared`).
- A double declared and an integer written (`speed: 1` against `1.0`) is rejected at start.
- A parameter callback that accepts a value but never copies it into the member the code
  uses, or a value read once in the constructor while the docs call it dynamic.

**QoS.** A reliable subscription never hears a best effort publisher (sensor data presets).
`/map` and `robot_description` are transient local, and a volatile subscriber that starts late
never gets them. `rosio drift` reports `qos.incompatible`.

**Executors and threads.**
- A blocking call inside a callback (`spin_until_future_complete`, a synchronous service
  call, `future.get()`, a sleep) on a single threaded executor deadlocks: the reply needs the
  same thread.
- Callbacks in a reentrant group, or several groups on a multi threaded executor, that share
  members without a mutex.
- A lambda capturing a message or local by reference that runs after the scope ends; a
  `std::thread` that the destructor neither joins nor stops.

**Lifecycle nodes.** A `LifecyclePublisher` publishes nothing before `on_activate`; timers made
in `on_configure` keep firing while inactive; `on_cleanup` that does not release what
`on_configure` made; a failure that returns `SUCCESS`.

**Sensor messages.**
- `LaserScan`: ranges can be NaN or inf and must be checked against `range_min` and
  `range_max`; the index of angle `a` is `(a-angle_min)/angle_increment`, bound it.
- `PointCloud2` with `is_dense` false holds NaN points; `Image` has an `encoding` and a `step`
  that is not always `width * channels`.
- Covariance arrays are row major: a 6x6 diagonal is 0, 7, 14, 21, 28, 35, a 3x3 is 0, 4, 8.
  `Imu` with `orientation_covariance[0] = -1` has no orientation.

**Errors that hide.** `catch (tf2::TransformException &)` that logs and then uses the
transform anyway; a service callback that returns without filling the response; a return
value (`canTransform`, `waitForService`) ignored.

**Launch and packaging.** Checked by `rosio drift`: remaps that match nothing, arguments an
include never reads, `use_sim_time` mixed in one profile, files a launch reads that the build
does not install. By hand: a `find_package` or python import whose package `package.xml` does
not depend on, which builds on the developer's machine and fails on the build farm.

## Reporting

One bug per entry: `file:line`, the input and the wrong result, the evidence (the line, the
doc or REP it breaks), and the smallest diff. Say how you verified it. Mark anything you could
not trace end to end as a suspicion, not a bug.
