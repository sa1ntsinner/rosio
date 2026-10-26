# Demo

A running system with one broken sensor, in containers, no robot needed. Two nodes publish `sensor_msgs/msg/Imu` at 50 Hz on the same DDS domain. One of them stamps correctly, the other stamps 37 s ahead, which is what a device serving TAI instead of UTC looks like. The doctor captures the live topics for ten seconds and says which one is wrong and why.

## Three commands

```
docker compose build
docker compose up -d
docker compose run --rm doctor
```

The first command builds one image holding stamplint, the stampdoctor node and the demo publishers. The second starts the two publishers in the background. The third is the demo moment.

## What you should see

```
capture: 10.0 s, 2 topics, 887 messages

ERROR epoch_offset /imu/skewed: /imu/skewed offset 36.999 s matches the TAI to UTC offset (37 s), check the PTP timescale or a ptp_utc_tai_offset driver setting, stamps ahead of the receiver
INFO stamp_rate /imu/clean: /imu/clean rate 50.00 Hz, p95 jitter 0.0001 s
INFO stamp_rate /imu/skewed: /imu/skewed rate 50.00 Hz, p95 jitter 0.0001 s

verdict:
  /imu/skewed carries a tai utc offset of about 36.999 s while the rest of the system is clean, set the PTP timescale to UTC or correct a ptp_utc_tai_offset style setting on that driver.

time sources:
  WARN none: no time sources observed, the clock is unmonitored
```

The exit code is `1`, because there is at least one error finding. Message counts and the last offset digit move a little from run to run. The time sources warning is correct behavior: the container has no PTP daemon, chrony or timesyncd to ask, and an unmonitored clock is reported rather than assumed healthy. On a real host that block names what disciplines the clock.

## Switching the fault

The faulty publisher takes its pathology from the environment, so the shell that starts it chooses what the doctor has to find. `OFFSET_S=18 docker compose up -d` shifts the stamps by 18 s instead and the report comes back as the GPS to UTC leap offset rather than TAI. `MODE=drift docker compose up -d` stamps correctly at first and then slides away at `DRIFT_S_PER_S`, default 0.005, which the doctor reports as `clock_drift` at about 5 ms/s together with the error it accumulates in ten minutes. `MODE=zero docker compose up -d` publishes a header that is never filled, and the report stacks up `zero_stamps`, `frozen_stamps` and an unsynced epoch on the same topic. Re run `docker compose run --rm doctor` after any of these; the clean publisher stays clean throughout, which is what lets the verdict blame one source instead of the host clock.

## Teardown

```
docker compose down
```

The image stays. Remove it with `docker rmi stampdoctor-demo:latest`.

## Notes

The containers run on `ROS_DOMAIN_ID` 42 so the demo never joins a domain you are already using, and they share the host IPC namespace so that the default DDS shared memory transport can carry messages between them. On Docker Desktop, where `network_mode: host` does not reach the host, replace it in `compose.yaml` with a shared bridge network and the publishers and the doctor still find each other over UDP.
