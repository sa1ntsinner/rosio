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

## The mistakes that ship most, most frequent first

Confirmed bugs in public ROS 2 repositories come down to these habits. Check each one against
the code you write.

1. Bounds: before `front()`, `back()`, `points[i + 1]`, a search result or an index from a
   message, make sure the container holds it. Handle empty and single element inputs first.
2. Identity: a value stays with its name. Map joint and topic names to indices explicitly,
   keep the map through filtering and reordering, and size storage from the same set that
   indexes it. A value in a valid slot that belongs to another joint crashes nothing.
3. Guards that stop: a failed check returns, continues or throws at the right scope. Logging
   an error and carrying on, or reporting success before the work is done, is the bug.
4. Configuration names: use the exact declared key and type everywhere, build scoped names
   with one separator (`name + "." + key`), declare before reading, convert launch strings
   to the type the API takes.
5. Outside input: parse completely, catch the parser's exceptions, check required fields and
   accepted values. Resolve a path and check it stays inside its directory before deleting.
6. Message metadata from the payload: `height`, `width`, `step`, `encoding`, field types and
   the header describe the bytes you send. Check sizes before decoding a binary message.
7. State per use: decide what survives a new goal, a reset, a seek or a cache clear, and reset
   the rest together. No per request state in function statics or stale cached pointers.
8. Predicates for every value: separate `None`, empty, zero and false. Write the truth table
   for a disabled setting (a zero timeout that means "off") before choosing `&&` or `||`.
9. Launch wiring: every argument reaches its consumer under the name it reads, every file a
   launch names is installed, defaults and namespaces are run once. `rosio drift` checks
   much of this.
10. Ownership: one owner per resource, captures that outlive nothing, threads stopped and
    joined before what they use is freed, no `delete` on memory a library owns.
11. Formulas: write units and invariants first, keep old values an update still needs, check
    a hand computed case and both endpoints of every loop.
12. Clocks: one clock per purpose (above), full seconds and nanoseconds, age is now minus
    stamp, and temporal state resets on a backward jump.
13. Lifecycle: every resource has the transition that makes it, the one that stops its users
    and the one that frees it. A failed transition rolls back what it made.
14. Diagnostics and tests tell the truth: print the value the check used, assert the change
    the code promised, not a value that was already true.
15. Initialise every member and every local on every branch before a callback or thread can
    see it.
16. A parameter update the callback accepts reaches the running code: the object built from
    it, the timer, the threshold, the transform.
17. Numbers in range: check finiteness and the domain before dividing, `sqrt`, `log`, clamping
    or narrowing; keep NaN out of filters and actuator commands.
18. Use what you computed: the updated value, the returned result, the option the caller
    passed, not the original.
19. Units and width: name the unit, convert once at the boundary, never cast epoch stamps to
    `float` or 32 bit before subtracting.
20. Frames: the output header names the frame the data is in after a transform succeeded;
    on failure drop or retry, never publish untransformed data under the target frame.
21. Do not edit a container you iterate, a borrowed message, or a shared mutable default.
22. Spatial math: state vector order, bases and multiplication order; rotate a covariance
    as `R * C * R^T`; test with a rotation that is not the identity.
23. Parallel arrays in a message have the lengths their names say; reject the message if not.
24. Signs and angles: limits on magnitude, sign kept apart, angle differences wrapped.
25. Every wait has a deadline and a shutdown check, and a timeout is an outcome.
26. Each goal or async request is tracked by its own handle from before dispatch to exactly one
    completion; rejection and cancellation are explicit.
27. Stop paths remove authority: on cancel, abort, error or stale input send the stop
    command, and start with motion disabled until every input is valid.

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

## On a real machine

What careful drivers and controllers do (UR, franka, ros2_controllers, nav2 collision
monitor, velodyne, socketcan) and average code does not:

* Expire commands in the control cycle itself, and replace an expired command with a stop
  target driven through the limiter (`diff_drive_controller` writes zero on timeout). Expire
  control input an estimator uses the same way.
* Arm the device's own watchdog when it has one; a check in ROS does nothing once the host
  hangs. Arbitration (`twist_mux`) or collision filtering is not a motor watchdog.
* Missing, stale or untransformable safety data, or an unset footprint, means stop, never
  free space.
* Latch faults. Require a cleared fault, zero commands and fresh state before activation or
  an E stop reset; a software heartbeat does not undo an external stop.
* Cancel, abort, deactivate, error and shutdown each send an explicit stop, while the
  publisher or interface still works, and only then release it.
* Track freshness per data stream on a monotonic clock. One message received is not a live
  stream; diagnostics report data age, not only frequency.
* Every device wait is bounded: nonblocking sockets with a finite `poll`, bus reads with a
  timeout told apart from other errors, state transitions with a deadline that returns
  failure. A nonpositive timeout must not mean wait forever.
* `read` and `write` return `ERROR` on a failed transfer; reconnect with a pause and a fresh
  handle; a malformed packet is dropped before its payload is copied.
* In `update`, `read` and `write`: `try_lock` handoffs, publication from a worker
  (`RealtimePublisher`), buffers sized before the loop, no slow device configuration. Run
  periodic cycles from a steady timeline and skip a missed period rather than bursting.
* On activation seed position commands from the measured position and velocity commands with
  zero; after a mode switch wait for fresh state; on deactivation clear buffered commands and
  limiter history.
* Reject non finite commands at the hardware boundary, check array sizes and joint names
  before indexing, and check drivetrain geometry (wheel radius, separation) is positive.
* Limit velocity, acceleration and jerk with the measured period, decelerate on reversal,
  scale coupled axes together, and check stopping distance before a joint limit.
* A failed start rolls back what it activated; cleanup reports the failure of what it
  delegated to.
* Stop and join worker threads before destroying what their callbacks touch; device callbacks
  must not reach publishers that are still being built.

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
