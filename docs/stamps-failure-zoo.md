# The timestamp failure zoo

A catalog of the timestamp failures that actually show up in robot systems, written for the engineer who arrived here after pasting an error message into a search box.

Each entry has the same four parts.

- **Symptom.** What you see first. Usually not a timestamp complaint at all, but a transform error, a localisation dropout or an odometry solver that refuses to initialise. Where a public report documents the exact text, it is quoted verbatim so the search that brought you here matches.
- **Cause.** The mechanism underneath, in one paragraph.
- **What rosio reports.** The check name, the severity, and the wording `rosio stamps` prints. Run `rosio stamps /path/to/bag` on the recording, or `rosio probe --stamps --duration 10` against the live system, and compare.
- **Reference.** Public issues, dataset documentation and standards text. Every link here was fetched and read while writing this page. Where a claim could not be confirmed in the source, the entry says so instead of quoting.

Several entries end with a `rosio-bagmutate` line. That command injects the same fault into a clean recording of your own, or into the synthetic one `rosio-bagmutate --example clean.bag` writes, so you can confirm what the failure looks like in your pipeline before you go hunting for it in a real bag. Where no preset reproduces the failure honestly, the entry says that too.

A note on how the checks are built. `epoch_offset` compares the header stamp with the time the recorder wrote the message, classifies the difference against known constants, and separately tests whether the stamp is plausible in absolute terms. `inter_topic_skew` compares each topic against the median of the bag. `clock_drift` fits a slope through the offset over time. `monotonic`, `frozen_stamps`, `zero_stamps` and `stamp_gaps` look at one topic's stamp sequence on its own. Whatever the class, the reported severity is `error` when the recording cannot be trusted for fusion and `warning` when it merits a look.

## TAI against UTC, the 37 second offset from PTP hardware stamping

**Symptom.** Nothing publishes an obvious error. Instead the transform machinery starves. Nav2 reports:

> Message Filter dropping message: frame 'laser_frame' at time 1640913301.637 for reason 'the timestamp on the message is earlier than all the data in the transform cache'

The same message shows up from `tf2_ros::MessageFilter` in geometry2 issue 788, where the filter blocks until timeout rather than recovering. The costmap stops updating, the robot stops, and the lidar looks fine on every other measure.

In the more extreme form the offset is not 37 seconds at all. In ouster-ros issue 366 the reporter measured a fixed offset of `74986812` seconds, about 2.38 years, with the sensor in PTP mode, and posted the delay statistics:

> average delay: 74986812.741...min: 74986812.494s max: 74986813.430s

**Cause.** PTP distributes time on the TAI timescale. TAI is ahead of UTC by a whole number of seconds, currently 37, and that number changes when a leap second is inserted. A sensor that hardware stamps from its PTP clock and hands the value straight into a `std_msgs/Header` therefore publishes UTC plus 37 whenever the grandmaster serves TAI and the host runs UTC. Everything downstream that was stamped from the host clock sits 37 seconds in the past relative to the sensor, and tf never has a transform old enough. The much larger offsets come from the same family: a sensor whose PTP clock is counting from the PTP epoch rather than the Unix epoch, or a grandmaster that came up without a valid time source at all.

The driver side of this is a configuration parameter. ouster-ros introduced `ptp_utc_tai_offset` with a default of `-37.0`, and issue 231 argues for changing that default because upgrading users had to "change the aforementioned parameter from `-37.0` to `0.0` for our synchronization to work properly again". Whichever value is right for a given rig, it is a manual constant that nothing validates, so it is a permanent source of this failure.

**What rosio reports.** `epoch_offset`, severity `error`, class `tai utc offset`:

```
ERROR epoch_offset /os_cloud_node/points: /os_cloud_node/points offset -37.000 s matches the TAI to UTC offset (37 s), check the PTP timescale or a ptp_utc_tai_offset driver setting, stamps ahead of the receiver
```

The class matches on magnitude, so either sign lands on `tai utc offset`, and the sign is spelled out in the message. The 74986812 second variant is far outside any leap offset and comes out as class `unsynced epoch`, also an `error`:

```
ERROR epoch_offset /os_cloud_node/points: /os_cloud_node/points stamps are 74986812 s off the receiving clock, source clock was never set, stamps ahead of the receiver
```

If only the lidar carries the offset while the rest of the bag is clean, the verdict line names the fix directly: set the PTP timescale to UTC or correct a `ptp_utc_tai_offset` style setting on that driver. If every topic carries it, the verdict points at the recorder host clock instead of the sensors.

Live, `rosio probe --stamps` reaches the same classification from its sampling window.

**Reference.**

- ouster-lidar/ouster-ros issue 366, the 74986812 second offset: https://github.com/ouster-lidar/ouster-ros/issues/366
- ouster-lidar/ouster-ros issue 231, the `ptp_utc_tai_offset` default of `-37.0`: https://github.com/ouster-lidar/ouster-ros/issues/231
- ros-navigation/navigation2 issue 2768, the transform cache message: https://github.com/ros-navigation/navigation2/issues/2768
- ros2/geometry2 issue 788, the same message from the message filter: https://github.com/ros2/geometry2/issues/788
- IERS Bulletin C, "from 2017 January 1, 0h UTC, until further notice : UTC-TAI = -37 s": https://datacenter.iers.org/data/latestVersion/bulletinC.txt

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology tai
rosio-bagmutate clean.bag out --pathology ouster_ptp
```

## GPS against UTC, the 18 second leap offset

**Symptom.** The same downstream shape as the TAI case, a smaller number. Transform lookups fail on the GNSS derived topics only, fused odometry rejects the GNSS updates as too old, and a fix that is geometrically correct lands 18 seconds away from the pose it should correct. On a moving platform that is tens of metres of apparent position error with no corresponding jump in the raw fix.

**Cause.** GPS time is continuous and carries no leap seconds. It was aligned to UTC at its epoch in January 1980, when TAI minus UTC was 19 seconds, so GPS minus UTC equals the current leap second count minus 19. ESA's Navipedia states the relation directly: "At that epoch, the difference TAI-UTC was 19 seconds, thence GPS-UTC=n - 19s." With UTC minus TAI at minus 37 seconds since 2017, that leaves GPS ahead of UTC by 18 seconds today. A receiver or a driver that reports the GNSS time of week without applying the leap second correction publishes stamps 18 seconds in the future.

The value is not a constant of nature, it steps whenever a leap second is inserted, which is why code that hardcodes it silently rots. IERS Bulletin C also announces that no leap second will be introduced at the end of December 2026, so 18 is correct for now.

**What rosio reports.** `epoch_offset`, severity `error`, class `gps utc leap offset`:

```
ERROR epoch_offset /gnss/fix: /gnss/fix offset -18.000 s matches the GPS to UTC leap offset (18 s), the source likely serves GPS time as UTC, stamps ahead of the receiver
```

Because the classifier works on magnitude, anything from 15 to 25 seconds lands in this class, which covers both the current 18 and the older leap counts that appear in archived recordings.

When only the GNSS topics carry it, `inter_topic_skew` fires alongside on those topics, and the verdict names them as the offset sources while the rest of the system is clean.

**Reference.**

- ESA Navipedia, time references in GNSS, for the GPS to TAI and GPS to UTC relation: https://gssc.esa.int/navipedia/index.php/Time_References_in_GNSS
- IERS Bulletin C, for the current leap second count: https://datacenter.iers.org/data/latestVersion/bulletinC.txt
- UrbanNav dataset, for a real recording where the bag clock and the GNSS clock are not tied together. The documentation states, for the pilot sequences, "The rosbag timestamp might have a gap with the GPS time as no hardware time sync was applied in the pilot data." The readme does not state a leap second figure, so the 18 second value above is taken from the standards references and not from the dataset: https://github.com/IPNL-POLYU/UrbanNavDataset

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology gps_leap
rosio-bagmutate clean.bag out --pathology gps_leap --magnitude -18
```

## The source clock was never set, stamps in 1970 or near zero

**Symptom.** Playback looks broken in a way that is hard to describe. RViz shows nothing for the affected topic. Transform lookups fail immediately rather than after a tolerance window. Anything that subtracts two stamps gets an absurd number. In realsense-ros issue 1756 the reporter measured the offset between the recorder and the sensor directly:

> serialization time 1615285596.116480667 but sensor time 1615284828.469705105 differing by -767.647 s

with the IMU also non monotonic and, in the same recording, "IMU data (frame_id: "camera_imu_optical_frame") has a large gap, largest is 84.1733 s".

The pure form is a stamp of exactly zero, or a stamp a few seconds after the Unix epoch. In the nuway autonomous shuttle recording in our corpus, `/bond` behaves that way on a live system: its stamps sit in 1970 while every other topic on the same bag sits within milliseconds of the recorder clock.

**Cause.** Two mechanisms, same signature. Either the publisher never fills the header at all, so the stamp stays at its zero initialised value, or the source device has its own clock that came up unset and counts from its own boot rather than from the epoch. Embedded sensors, microcontroller bridges and any node that constructs a message without calling the clock all land here. The realsense case is the second kind, a sensor clock running on its own base and never disciplined to the host.

**What rosio reports.** Two checks, both `error`, depending on which mechanism it is.

A hard zero:

```
ERROR zero_stamps /bond: /bond never sets the stamp, all 1200 messages are zero
```

or, if only some messages are affected, "`/bond` has 43 of 1200 messages with a zero stamp".

A clock counting from its own base, which is what the nuway `/bond` topic actually does, goes through the absolute plausibility gate in `epoch_offset` before any offset classification happens:

```
ERROR epoch_offset /bond: /bond stamps sit in year 1970, before the plausibility floor of 2005, source clock was never set or a post processing shift moved them
```

On a multi topic bag the same topic normally also trips `inter_topic_skew` with an enormous delta, because its offset from the recorder clock is the whole age of the Unix epoch.

**Reference.**

- IntelRealSense/realsense-ros issue 1756, "D435i IMU Timming Problem", with the sensor time offset and the 84 second gap quoted above: https://github.com/IntelRealSense/realsense-ros/issues/1756
- xrkong/nuway_rosbag, the shuttle recordings where `/bond` shows the 1970 stamps described above, observed on 2026-08-11: https://huggingface.co/datasets/xrkong/nuway_rosbag

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology zero --topic /scan
```

## A post processing shift applied to both clocks

**Symptom.** Nothing at all, at first. The bag replays cleanly, every topic agrees with every other topic, and the latency between the header stamp and the record time is zero on every message. The failure appears only when the recording meets something that carries real world time: a GNSS log, a calibration file with an absolute epoch, a second recording made on a different machine, or a solver that assumes the stamps are Unix time.

**Cause.** The dataset was normalised offline. The Hilti Trimble SLAM Challenge 2026 documents its convention plainly:

> A constant offset of `10000 s` was applied to each recording to prevent negative IMU timestamps.

That is a reasonable thing for a dataset publisher to do, and it is invisible to every relative check, because the shift was applied to the record timestamps as well as to the header stamps. We measured it: image stamps start at exactly 10000.000000 s and the IMU at 9999.253204 s, while the median of receive minus stamp is 0.000000 s on all three topics. Any tool that only compares the two clocks against each other declares the bag perfect.

**What rosio reports.** This case is why `epoch_offset` tests absolute plausibility before it tests the offset. The first nonzero stamp of each topic is compared against a floor of 2005-01-01 UTC, and a stamp below it is an `error` regardless of how well the two clocks agree:

```
ERROR epoch_offset /alphasense/imu: /alphasense/imu stamps sit in year 1970, before the plausibility floor of 2005, source clock was never set or a post processing shift moved them
```

The message covers both readings deliberately, because from the bag alone the two are indistinguishable: an unset source clock and a deliberate offline shift produce the same bytes. The evidence field carries the first stamp and a sample of the following ones so a reader can tell which one it is looking at.

**Reference.**

- Hilti Trimble SLAM Challenge 2026, timestamp convention: https://huggingface.co/datasets/Hilti-Research/hilti-trimble-slam-challenge-2026
- Challenge repository: https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026

**Reproduce it.** No preset reproduces this one honestly. `rosio-bagmutate` shifts header stamps and copies the record timestamps untouched, which is the opposite of what happened here, so `rosio-bagmutate clean.bag out --pathology offset --magnitude -1600000000` produces a bag that the offset gate catches for the wrong reason. Building a true positive control means rewriting the record timestamps as well, which the mutation tool deliberately does not do.

## A driver serving the wrong scale because of one configuration flag

**Symptom.** Stamps land in the future by a round, suspiciously human number of hours. From septentrio_gnss_driver issue 142:

> the driver is publishing timestamps that are ~2.5 Hours into the future

with the concrete pair `sec: 1731667491` published while UTC was `1731659021`. The reporter had set `use_gnss_time: true` expecting the receiver's own time and got a value that tracked neither the receiver nor the host.

**Cause.** GNSS drivers usually expose a switch between the host clock and the receiver clock, and the two paths have different bugs. The receiver path can serve a time of week without the week number, a local time with a zone applied, a time that is only valid once the receiver has a fix, or GPS time labelled as UTC. The switch is a boolean in a YAML file, it is set once during bringup, and nothing downstream ever validates the result. Offsets that are near multiples of an hour are the tell: they come from a timezone or a coarse initialisation, not from a leap second and not from network latency.

**What rosio reports.** An offset of this size fits no known constant, and the stamps run ahead of the recorder, so it comes out as `epoch_offset`, severity `error`, class `stamps from the future`:

```
ERROR epoch_offset /gnss/navsatfix: /gnss/navsatfix stamps run 9000.000 s ahead of the receiving clock
```

If the GNSS topics are the only ones affected, which is the usual shape, `inter_topic_skew` fires on them as well and the verdict states that the clock or driver of that source is at fault while the other topics are clean. That is the sentence that sends you to the driver configuration rather than to the host.

**Reference.**

- septentrio-gnss/septentrio_gnss_driver issue 142, the 2.5 hour offset and the `use_gnss_time` semantics: https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/142

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology septentrio
```

## Stamps that go backwards or stop moving under load

**Symptom.** Three different faces of the same underlying condition.

Backwards steps inside one topic. realsense-ros issue 1756 reports that the IMU timestamps "are not strictly increasing" and that the stream "jumps backwards in time", which happened on a Raspberry Pi recorder and not on a desktop with the same camera. Any filter that integrates over a negative time step produces garbage, and most solvers either diverge or refuse the message.

Stamps that stop advancing while messages keep arriving. A driver under load reuses the last stamp it computed. Downstream everything looks alive on the topic monitor, the message rate is nominal, and the state estimate freezes.

Rate that is not the rate you configured. zed-ros2-wrapper issue 445, "IMU timestamps don't have constant rate", documents exactly this on a Jetson Orin AGX: the reporter extracted IMU stamps to CSV, plotted the deltas, and found spread instead of a line. To be precise about what that issue does and does not say, it documents irregular intervals, not backwards steps. The backwards steps in this section come from the realsense report.

Backwards stamps also arrive from the recorder rather than the sensor. rosbag2 issue 857, "Reading can return out-of-order timestamps across file boundary in split bags", explains the mechanism: individual files use their index to return messages in timestamp order, but "a file is played in full before a next file is loaded", so a message written out of order near a split can replay after messages that are newer than it. The sensor was fine, the recording is what reordered.

**Cause.** A driver that computes its stamp on the CPU rather than in hardware inherits every scheduling delay on the host. Under load the arrival order of the buffers stops matching the acquisition order, or the driver skips the clock read and reuses the previous value, or two threads write the header from different clock sources. Split bag replay adds an ordering hazard on top that has nothing to do with the sensor at all.

**What rosio reports.** Three checks, and which one fires tells you which face you have.

```
ERROR monotonic /camera/imu: /camera/imu stamps go backwards, largest jump 1.500000000 s
ERROR frozen_stamps /camera/imu: /camera/imu stamp frozen at 1731659021.412000000 s for 25 messages over 1.250 s
INFO stamp_rate /camera/imu: /camera/imu rate 199.83 Hz, p95 jitter 0.0041 s
```

`monotonic` and `frozen_stamps` are errors. `stamp_rate` is informational by design: jitter is a number you compare against the sensor's specification and against the same topic on a healthy day, not something the linter can call good or bad on its own. It is printed for every topic with enough messages, which makes it the fastest way to confirm or refute the zed issue 445 shape on your own hardware.

The verdict adds the reading. For `monotonic` it states that the source clock stepped backwards or the messages were reordered, which is the distinction between the realsense case and the rosbag2 case.

**Reference.**

- IntelRealSense/realsense-ros issue 1756, non monotonic IMU stamps: https://github.com/IntelRealSense/realsense-ros/issues/1756
- stereolabs/zed-ros2-wrapper issue 445, irregular IMU stamp intervals: https://github.com/stereolabs/zed-ros2-wrapper/issues/445
- ros2/rosbag2 issue 857, out of order replay across a split bag boundary: https://github.com/ros2/rosbag2/issues/857

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology backward --at 0.5
rosio-bagmutate clean.bag out --pathology frozen --length 25
```

## Recorder dropouts, every topic at once

**Symptom.** A hole in the recording. Not one sensor missing, all of them, for roughly the same window. The Hilti SLAM Challenge 2021 issue titled "Large sections of dropped messages" describes it from `rqt_bag`:

> there are many gaps of ~1 second length where almost no data is recorded. From our experience, this usually happens when we try to more data than the disk can handle so messages get dropped.

Downstream this is the failure that looks like a sensor fault and is not one. Odometry loses tracking at the gap, the map tears, and the topic that gets blamed is whichever one the estimator noticed first.

**Cause.** The recorder, not the sensors. Write bandwidth exceeded the disk, or a compression thread stalled, or the host swapped. Every topic sharing the recorder shares the gap, which is exactly what separates this from a single sensor dropping out.

**What rosio reports.** `stamp_gaps`, severity `warning`, one finding per affected topic:

```
WARNING stamp_gaps /os_cloud_node/points: /os_cloud_node/points has stamp gaps, largest 1.032 s against a median 0.100 s
```

A gap has to exceed both five times the topic's median period and half a second, so bursty streams that are supposed to be bursty, RTCM correction traffic for example, are not reported as broken by the fact of being bursty alone.

The part that matters here is the verdict, which correlates the gaps in time rather than reporting them one by one:

```
/imu/data, /os_cloud_node/points and /camera/image_raw all gap at the same moment, up to 1.032 s, so the recorder or the host stalled around t=1731659087.412 rather than each sensor failing on its own.
```

Three or more topics gapping inside a two second window is what triggers that sentence. It is the difference between opening the recorder configuration and opening a sensor driver.

**Reference.**

- Hilti-Research/hilti-slam-challenge-2021 issue 13, the report quoted above: https://github.com/Hilti-Research/hilti-slam-challenge-2021/issues/13
- Honest caveat. The dropouts do not reproduce in the currently published bag. On 2026-08-11 we scanned `LAB_Survey_2.bag` from the challenge download, 5000 messages per topic, and found no `stamp_gaps` findings at all, with per topic p95 jitter at or below 1 ms. Either the published file was replaced after the report or the gaps were an artefact of the reporter's tooling. The sequence is kept in our corpus as a clean control rather than as a defect case, and the synthetic dropout entry carries the positive control instead.

**Reproduce it.**

```
rosio-bagmutate clean.bag out --pathology dropout --magnitude 2.0
```

## Multi sensor rigs whose clocks drift apart

**Symptom.** Calibration that was correct at the start of a run is wrong at the end. Points smear along the direction of travel, more at minute twenty than at minute one. Visual inertial initialisation succeeds on short segments and fails on long ones. Nothing in the recording is ever obviously wrong at any single instant, which is what makes this the most expensive entry in the zoo.

The Newer College Dataset documents it against its own hardware:

> After the experiments we noticed that the two sensor's timestamps slowly drift relative to one another.

with a measured figure of "Relative drift is about 58.78 ms/h", attributed to the RealSense camera, probably from the point its driver starts. The lidar side of that rig was synchronised: the Ouster is disciplined to the recording computer over PTP. The dataset ships a csv of time offsets per experiment so users can correct the camera stamps, which is a fair admission that the recording cannot be used at full precision without it.

**Cause.** Two independent oscillators, each free running, each with its own frequency error. A crystal that is 20 parts per million off produces 72 ms of divergence per hour on its own. Nothing in the data model notices, because both streams are internally consistent and monotonic, and any check that looks at a single instant sees agreement. Only the slope over a long window reveals it. This is also why the failure survives every review that consists of playing back the first thirty seconds of a bag.

**What rosio reports.** `clock_drift`, which fits a line through the offset between each topic's stamps and the recorder clock:

```
WARNING clock_drift /camera/imu: /camera/imu clock drifts at 0.150 ms/s against the receiver, its source clock is unsynchronized
```

Drift is reported as a `warning` above 1e-4 s per s and an `error` above 1e-3 s per s. Calibrate your expectations against the Newer College number before you go looking: 58.78 ms/h is 1.633e-5 s per s, well under the warning floor, so a rig drifting at that rate is not reported by this check at all. It still costs 0.01 s of misalignment after ten minutes and 0.06 s after an hour, which is enough to matter for a spinning lidar. The floor is set where it is because anything lower is not separable from ordinary offset noise on a short recording. To avoid calling a single step a drift, the check requires at least 30 messages over at least 5 seconds and requires each half of the series to reproduce at least half the overall slope with the same sign. A dropout produces a step in one half and nothing in the other, so it does not qualify. Both slopes are loose bringup heuristics rather than an error budget, so a silent `clock_drift` is not a statement that the rig is aligned, only that it is not visibly falling apart. With a budget of your own, the constants to move are `DRIFT_WARN_S_PER_S` and `DRIFT_ERROR_S_PER_S` in `rosio/stamps/clock.py`.

The verdict converts the slope into the number that decides whether you care:

```
/camera/imu drifts against the receiving clock at 1.500e-04 s per s, which is 0.090 s of error after ten minutes, discipline that source clock instead of resampling its stamps.
```

Alongside it, `inter_topic_skew` catches the static part of the same problem, a source whose median offset simply disagrees with the rest of the bag by more than 0.1 seconds. Skew tells you the rig was never aligned. Drift tells you it will not stay aligned.

**Reference.**

- Newer College Dataset, stereo camera calibration page, drift measurement and the per experiment offset csv: https://ori-drs.github.io/newer-college-dataset/stereo-cam/calibration-stereo/
- Newer College Dataset home: https://ori-drs.github.io/newer-college-dataset/

**Reproduce it.** The `drift` preset rescales stamps against elapsed time instead of shifting them, so its magnitude is a slope in seconds per second, applied from `--at` onward. A single topic offset produces the skew half of this entry without the drift half:

```
rosio-bagmutate clean.bag out --pathology drift --at 0 --magnitude 0.02
rosio-bagmutate clean.bag out --pathology offset --topic /camera/imu --magnitude 0.3
```

The corpus carries both as `synthetic-drift` and `synthetic-skew`. A slope has to be present across the whole recording to be reported, so a drift injected halfway through a bag is not detected.

## Corpus entries that exercise these cases

The checks above are validated against a manifest of public recordings in `corpus/stamps/manifest.yaml`, in four tiers: `control` recordings that must stay quiet, `buggy` recordings with a publicly documented defect, `noise` unlabelled data used to hunt reader crashes, and `synthetic` positive controls built offline with `rosio-bagmutate`. The data is never committed. `python corpus/stamps/fetch.py --list` shows the manifest and `python corpus/stamps/run.py` checks whatever is on disk against the expected findings.

- `synthetic-tai-offset`, the TAI against UTC entry. A clean 12 second scan bag with every stamp moved 37 seconds off the record clock. Expects `epoch_offset` at `error` on `/scan`, forbids `stamp_gaps` and `monotonic`.
- `synthetic-dropout`, the recorder dropout entry. A 2 second window of scans removed from the middle of the same clean bag, the shape Hilti 2021 issue 13 describes. Expects `stamp_gaps` at `warning` on `/scan`, forbids `epoch_offset` and `monotonic`.
- `hilti-trimble-2026-sample`, the post processing shift entry. The 10000 second shift, applied to both clocks and therefore invisible to the offset gate, caught by absolute plausibility. Expects `epoch_offset` at `error`. This entry also drove the sqlite3 fallback reader, since the bag is written by Jazzy with empty type description hashes and `rosbags` refuses to open it.
- `urbannav-tokyo`, the GNSS entry. No hardware time synchronisation between the sensors. Expects `inter_topic_skew` at `warning` and `epoch_offset` at `warning`.
- `hilti-2021-lab-survey-2`, the recorder dropout entry as a negative result. Held as a `control` rather than a defect case, because the reported 1 second gaps did not reproduce in the published bag. Forbids any `error`.
- `hf-nuway`, the unset source clock entry. Autonomous shuttle recordings where `/bond` carries 1970 stamps on a live system while the rest of the bag tracks the recorder. Held in the `noise` tier with no expectations, since it is unlabelled data.
- `hf-tesla3`, `noise` tier, the reader robustness case. It is the entry that forced per connection type registration, after one unparseable `ublox_ubx_msgs` definition took a whole bag down. Its only findings are `stamp_gaps` on the RTCM correction streams, which is normal for correction traffic.
- `euroc-mh01` and `foxglove-nuscenes-scene0061`, `control` tier, the false positive baseline. Hardware synchronised recordings that must produce no `error` finding. Without these, tightening any threshold above is unfalsifiable.
- `euroc-v2-03`, `buggy` tier, documented missing camera frames. Expects `stamp_gaps` at `warning` on the camera topics.
