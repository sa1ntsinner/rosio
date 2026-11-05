# Writing and fixing ROS 2 code

Read this before you write, change or debug code in a ROS 2 repository. It is short on
purpose: every line is a mistake that shipped in a real robot stack.

## Before you type

1. Read the whole node you are changing: constructor or `on_configure`, every callback, the
   parameter callback, `on_activate`, `on_deactivate`, `on_cleanup`, the destructor. A fix
   that is right for one callback and wrong for another is the most common bad patch.
2. Find every caller of the function you touch (`rg -n 'name\('`). Fix the shared function
   once rather than each caller.
3. Ask rosio what really runs: `rosio param <node> <name>` for the value that reaches the node
   and the line that sets it, `rosio query <topic>` for who publishes and subscribes with
   which QoS, `rosio drift --min-severity warn` for what the launch files, yaml, models and
   code already disagree about. Edit the line rosio cites, not a copy of it.
4. Match the file: its naming, error handling, logging macros, units and container types. A
   fix written in another style gets rejected in review even when it is right.

## Debugging: look before you guess

* Reproduce first. Write the input or call sequence that breaks it, then the smallest test
  or launch that shows it. A fix you never saw fail is a guess.
* Live graph: `ros2 node info <node>`, `ros2 topic info -v <topic>` (QoS of every end),
  `ros2 topic hz`, `ros2 topic echo <topic> --field header.stamp`, `ros2 param dump <node>`,
  `ros2 lifecycle get <node>`, `ros2 service list -t`, `ros2 doctor --report`.
* Frames: `ros2 run tf2_ros tf2_echo <parent> <child>`, `ros2 run tf2_tools view_frames`.
  A transform that is late, missing or has two parents shows here first.
* `rosio probe` then `rosio drift` compares the running graph with the code and launch
  files: a node that never started, a topic nobody publishes, a QoS that never matches.
* Logs: `--ros-args --log-level <node>:=debug`. Crashes: `prefix='gdb -ex run --args'` on
  the launch `Node`, or build with `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"` and run
  the test. Data races: `-fsanitize=thread`. Python: `faulthandler`, `py-spy dump`.
* Bags: record the topics around the failure (`ros2 bag record`), replay with
  `--clock` and `use_sim_time:=true`, and check stamps with `rosio stamps <bag>`.
* Change one thing at a time and say what you expect before you run it.

## Habits that keep robots safe

* Stop on silence. A velocity or effort command needs a timeout after which the output is
  zero. A sensor message older than a stated age is not used. Compare the age of
  `msg.header.stamp` on the node clock with a parameter, and log once when data goes stale.
* Validate everything that comes from outside before use: array lengths that must match,
  indices from goals and services, NaN and inf, `range_min`/`range_max`, empty arrays,
  enum values. Reject with a clear error rather than clamp in silence.
* Hardware calls fail. Return `ERROR` or `FAILURE` from `read`, `write`, `on_configure` and
  `on_activate` when the device does; never `SUCCESS` after a failed open or a missing file.
* Real time loops (`update`, `read`, `write`, a control timer) do not allocate, lock a mutex
  a non real time thread holds, log every cycle, or block. Use `realtime_tools` buffers and
  `RCLCPP_*_THROTTLE`.
* Never block a callback: no `spin_until_future_complete`, synchronous service call or
  `future.get()` inside a callback of the same executor. Use an async call with a callback,
  or a separate callback group on a multi threaded executor.
* Time: the node clock (`now()`, `get_clock()`) for stamps and anything a bag replays;
  `steady_clock` for timeouts and watchdogs. Never `system_clock` or `time.time()` for either.
  Keep the input stamp on derived data; do not restamp with `now()`.
* Parameters: declare every one with a type and a default, validate in an
  `add_on_set_parameters_callback` and return `successful = false` with a reason for bad
  values. When a value changes, update the object built from it, not only the member.
* Lifecycle: `on_configure` after `on_cleanup` starts clean, so clear containers and reset
  pointers in `on_cleanup`. Create publishers once. A `LifecyclePublisher` publishes only
  after `on_activate`; timers stop in `on_deactivate`.
* QoS: sensors are `SensorDataQoS` (best effort). Latched data (`/map`, `robot_description`)
  is transient local on both ends. Match what the other end uses; `rosio query` shows it.
* Frames and units (REP 103, REP 105): SI units, x forward, y left, z up, optical frames z
  forward. Frame ids come from parameters or the input header, never a literal in the code.
  Quaternions in messages are x, y, z, w; `Eigen::Quaterniond(w, x, y, z)` takes w first.
  Normalise angle differences to [-pi, pi].
* Copy and paste is where `x` stays `x` in the `y` line. After duplicating a block, read it
  once for each renamed symbol.

## Undefined and unwanted behaviour

* C++: `v[i + 1]` on the last element, `v.size()-1` on an empty vector, a `size_t` loop
  counting down to `>= 0`, signed overflow, an uninitialised member read in a callback that
  can fire before configure, integer division where a ratio was meant (`1 / rate`), `abs`
  on a double without `std::`, comparing floats with `==`.
* Lifetimes: a lambda or timer capturing `this` or a local by reference that outlives it,
  `shared_from_this()` in a constructor, an iterator kept across `push_back` or `erase`,
  erasing from a container while looping over it with a range `for`.
* Threads: state touched by two callback groups or a driver thread needs a mutex or an
  atomic; read the whole struct under one lock, not field by field.
* Python: a mutable default argument, `except Exception: pass`, modifying a list while
  iterating it, a name assigned only inside one branch and used after it, integer stamps
  built from floats with rounding error (use `Time.from_msg`, `Duration`).
* Errors that hide: a caught `TransformException` followed by use of the transform anyway,
  a service callback that returns without filling the response, an ignored return value.

## Writing the change

* Fix the root cause in the smallest diff that makes every caller right. Do not refactor,
  rename or reformat code you were not asked to touch.
* A new parameter only when a user needs to tune it; a constant otherwise, named.
* Comments say why, never what. One line, next to the surprising code: the unit, the
  reference frame, the hardware quirk, the reason a check exists. No comment that restates
  the code, no commented out code, no change history in comments.
* Log once with context a user can act on (`node`, value, limit, what happens next), at the
  right level: `ERROR` for what stops the robot, `WARN` for degraded, `DEBUG` for the rest.
* Leave one test that fails without the fix, at the level the bug lives: a unit test for a
  function, a `launch_testing` test for a launch file or a node interaction.
* Build and run it: `colcon build --packages-select <pkg>` then
  `colcon test --packages-select <pkg> --event-handlers console_direct+`. Run `rosio extract`
  again after touching launch, yaml or models, and check that no new finding arrived.
