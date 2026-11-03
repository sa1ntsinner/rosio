# Hunting bugs in ROS 2 code

Read this before you look for bugs in a ROS 2 repository, review a change to one, or change
robot code yourself.

## How to work

1. Run `rosio drift --min-severity warn` first. rosio has already compared the launch files,
   yaml, robot models and sources, and each finding is a place where they disagree: a
   parameter no code declares, a remap that matches nothing, a file a launch default names
   that does not exist, a frame no model has. Reading the C++ alone does not show these. Open
   the cited line of each finding and report the ones that hold.
2. List every package that holds source, controllers, lifecycle nodes, planners and drivers
   first, and work through all of them. Most bugs sit in packages a short hunt never opens.
   Mark each package done as you go. With more than a handful of packages and a way to start
   subagents, hand each one a group of packages, these checks and the reporting rule below,
   and merge what they return after checking each claim against the code yourself.
3. In each package read every node end to end: the constructor or `on_configure`, each
   callback, the parameter callback, and `on_activate`, `on_deactivate` and `on_cleanup`.
   Then go through the checks below.
4. A candidate is a bug when you can name the input or the call sequence that breaks it and
   the wrong result. Read the callers and the message definition. Check `git log -S '<line>'`
   for a reason it is written that way.
5. After each bug, search sibling code for the same mistake: a pattern wrong once is often
   wrong twice.

## Where bugs cluster

Run these searches and read every hit; each is a site the checks below apply to.

```
rg -n 'add_on_set_parameters_callback|add_post_set_parameters_callback|dynamicParametersCallback'
rg -n 'on_configure|on_cleanup|on_deactivate' --type cpp --type py
rg -n 'handle_goal|handle_accepted|goal->|request->' --type cpp
rg -n 'rclcpp::Clock\(\)|system_clock::now|steady_clock::now|time\.time\(\)'
rg -n '\babs\(|atan2|deg2rad|M_PI|size\(\) ?-1|i >= 0'
rg -n 'catch \(.*TransformException|lookupTransform|lookup_transform'
```

## Checks, most productive first

**Lifecycle re-entry.** Walk configure, cleanup, configure again, and activate, deactivate,
activate again. A vector `on_configure` appends to without a `clear()`, a publisher or timer
made twice, state `on_cleanup` does not reset, a timer still firing after `on_deactivate`, a
`LifecyclePublisher` used before `on_activate`, a failure that returns `SUCCESS`.

**Input from outside.** A goal, service request or message field used as an index or size
without a bounds check (`goal_index`, two arrays that must have equal length), an empty array,
NaN or inf in a sensor value. `LaserScan` ranges outside `range_min` and `range_max`; a
`PointCloud2` with `is_dense` false; an `Image` whose `step` is not `width * channels`.

**Edges of a range.** The last element (`i + 1` past the end), a search that returns fewer
results than it asked for, `v.size()-1` on an empty container, a `size_t` counting down to
`i >= 0`. A zero that means "disabled" in one place and "expire at once" in another (timeouts).

**Parameters.** `rosio param <node> <name>` gives the value that reaches the node.
- Read without a declaration: `get_parameter` throws (`param.read_undeclared`).
- A dynamic parameter callback that updates the member but not the object built from it at
  configure time (odometry keeping the old wheel radius), or logs a rejection and accepts it.
- A name built by concatenation that misses its `.`; a double declared and an integer written.

**Copy and paste, signs.** A second block still using the first one's name: `x` for `y`,
`min` for `max`, `left` for `right`, `linear` for `angular`. A check that only works for one
sign of a value (clockwise but not counter clockwise rotation), `atan2(x, y)`.

**Units and angles.** Degrees against radians, applied twice or never. A yaw difference with no
wrap to [-pi, pi]. `1 / rate` with an integer rate. `abs()` on a double without `std::`.

**Frames** (REP 103, REP 105). Body frames x forward, y left, z up; optical frames z forward.
`map -> odom -> base_link`, one parent per frame. Message quaternions are (x, y, z, w) while
`Eigen::Quaterniond` takes w first. A transform looked up at the latest time for data stamped
earlier. A hard coded `frame_id` where a parameter or the input header names it.

**Time.** `rclcpp::Clock()`, `system_clock` or python `time.time()` where the node clock is
meant, which ignores `use_sim_time`. A stamp left at zero or set to `now()` over the input's
own stamp. `dt` zero or negative on the first message or after a simulation reset.

**QoS.** A reliable subscriber never hears a best effort publisher. `/map` and
`robot_description` are transient local; a volatile subscriber that starts late gets nothing.

**Threads.** A blocking call in a callback (`spin_until_future_complete`, a synchronous service
call, `future.get()`) on a single threaded executor deadlocks. Members shared by reentrant or
multi threaded callbacks without a lock. A lambda capturing a local by reference that outlives
it.

**Errors that hide.** A caught `TransformException` followed by use of the transform anyway; a
service callback that returns without filling the response; an ignored return value.

**Covariance.** Row major: a 6x6 diagonal is 0, 7, 14, 21, 28, 35, a 3x3 is 0, 4, 8.

## Reporting

One bug per entry: `file:line`, the input or sequence and the wrong result, the evidence, and
the smallest diff. Leave out style, missing features and anything you could not trace end to
end.
