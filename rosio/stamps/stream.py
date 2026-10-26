from statistics import median

from .model import Finding

FROZEN_MIN_RUN = 10
FROZEN_MIN_SPAN_S = 1.0
JITTER_MIN_MSGS = 20
GAP_DT_FACTOR = 5.0
GAP_MIN_S = 0.5
GAP_COUNT_DT_FACTOR = 3.0
JITTER_PERCENTILE = 0.95


def largest_backward_step(stamps):
    """Biggest backward jump in a stamp sequence as (seconds, index), (0.0, -1) if none."""
    worst = 0.0
    worst_index = -1
    for i in range(1, len(stamps)):
        drop = stamps[i - 1] - stamps[i]
        if drop > worst:
            worst = drop
            worst_index = i
    return worst, worst_index


def longest_frozen_run(stamps, receive_times):
    """Longest run of one repeated stamp that qualifies as frozen, or None.

    Returns (run length, receive span, stamp, start index).
    """
    if len(stamps) < FROZEN_MIN_RUN:
        return None
    best = None
    start = 0
    for i in range(1, len(stamps) + 1):
        if i < len(stamps) and stamps[i] == stamps[start]:
            continue
        run = i - start
        span = receive_times[i - 1] - receive_times[start]
        if run >= FROZEN_MIN_RUN and span >= FROZEN_MIN_SPAN_S and (best is None or run > best[0]):
            best = (run, span, stamps[start], start)
        start = i
    return best


def check_monotonic(series):
    findings = []
    for topic, entry in sorted(series.items()):
        stamps = entry.stamps
        worst, worst_index = largest_backward_step(stamps)
        if worst_index < 0:
            continue
        findings.append(
            Finding(
                check="monotonic",
                topic=topic,
                severity="error",
                message=f"{topic} stamps go backwards, largest jump {worst:.9f} s",
                evidence={
                    "largest_backward_jump": worst,
                    "index": worst_index,
                    "previous_stamp": stamps[worst_index - 1],
                    "stamp": stamps[worst_index],
                },
            )
        )
    return findings


def check_frozen_stamps(series):
    findings = []
    for topic, entry in sorted(series.items()):
        best = longest_frozen_run(entry.stamps, entry.receive_times)
        if best is None:
            continue
        run, span, stamp, index = best
        findings.append(
            Finding(
                check="frozen_stamps",
                topic=topic,
                severity="error",
                message=f"{topic} stamp frozen at {stamp:.9f} s for {run} messages "
                f"over {span:.3f} s",
                evidence={
                    "stamp": stamp,
                    "run_length": run,
                    "receive_span_s": span,
                    "start_index": index,
                },
            )
        )
    return findings


def check_zero_stamps(series):
    findings = []
    for topic, entry in sorted(series.items()):
        stamps = entry.stamps
        zeros = [i for i, stamp in enumerate(stamps) if stamp == 0.0]
        if not zeros:
            continue
        all_zero = len(zeros) == len(stamps)
        first_receive = entry.receive_times[zeros[0]]
        if all_zero:
            message = f"{topic} never sets the stamp, all {len(stamps)} messages are zero"
        else:
            message = f"{topic} has {len(zeros)} of {len(stamps)} messages with a zero stamp"
        findings.append(
            Finding(
                check="zero_stamps",
                topic=topic,
                severity="error",
                message=message,
                evidence={
                    "count": len(zeros),
                    "total": len(stamps),
                    "all_zero": all_zero,
                    "first_receive_time": first_receive,
                },
            )
        )
    return findings


def _percentile(values, fraction):
    ordered = sorted(values)
    index = round(fraction * (len(ordered) - 1))
    return ordered[index]


def _deltas(entry):
    stamps = entry.stamps
    if len(stamps) < JITTER_MIN_MSGS:
        return None, None
    deltas = [stamps[i] - stamps[i - 1] for i in range(1, len(stamps))]
    dt = median(deltas)
    if dt <= 0:
        return None, None
    return deltas, dt


def check_stamp_gaps(series):
    findings = []
    for topic, entry in sorted(series.items()):
        deltas, dt = _deltas(entry)
        if deltas is None:
            continue
        max_gap = max(deltas)
        if max_gap <= max(GAP_DT_FACTOR * dt, GAP_MIN_S):
            continue
        worst_index = deltas.index(max_gap) + 1
        gap_count = sum(1 for delta in deltas if delta > GAP_COUNT_DT_FACTOR * dt)
        findings.append(
            Finding(
                check="stamp_gaps",
                topic=topic,
                severity="warning",
                message=f"{topic} has stamp gaps, largest {max_gap:.3f} s "
                f"against a median {dt:.3f} s",
                evidence={
                    "median_dt_s": dt,
                    "max_gap_s": max_gap,
                    "gap_count": gap_count,
                    "worst_gap_receive_time": entry.receive_times[worst_index],
                },
            )
        )
    return findings


def check_stamp_rate(series):
    findings = []
    for topic, entry in sorted(series.items()):
        deltas, dt = _deltas(entry)
        if deltas is None:
            continue
        jitter = _percentile([abs(delta - dt) for delta in deltas], JITTER_PERCENTILE)
        rate = 1.0 / dt
        findings.append(
            Finding(
                check="stamp_rate",
                topic=topic,
                severity="info",
                message=f"{topic} rate {rate:.2f} Hz, p95 jitter {jitter:.4f} s",
                evidence={
                    "median_dt_s": dt,
                    "rate_hz": rate,
                    "p95_jitter_s": jitter,
                    "messages": len(entry.stamps),
                },
            )
        )
    return findings


CHECKS = {
    "monotonic": check_monotonic,
    "frozen_stamps": check_frozen_stamps,
    "zero_stamps": check_zero_stamps,
    "stamp_gaps": check_stamp_gaps,
    "stamp_rate": check_stamp_rate,
}
