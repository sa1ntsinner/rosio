import time
from statistics import median

from .model import Finding

MAX_PLAUSIBLE_LATENCY_S = 0.5
GPS_UTC_MIN_S = 15.0
GPS_UTC_MAX_S = 25.0
TAI_UTC_MIN_S = 30.0
TAI_UTC_MAX_S = 45.0
UNSYNCED_OFFSET_S = 30 * 86400.0
MIN_PLAUSIBLE_STAMP_S = 1104537600.0  # 2005-01-01 UTC
SKEW_WARNING_S = 0.1
SKEW_ERROR_S = 0.5
MIN_TOPICS_FOR_SKEW = 3
SAMPLE_STAMPS = 3
DRIFT_WARN_S_PER_S = 0.0001
DRIFT_ERROR_S_PER_S = 0.001
DRIFT_MIN_MESSAGES = 30
DRIFT_MIN_SPAN_S = 5.0
# A drift is present in every part of the series, a dropout step is present in none of them.
# Each half must reproduce at least this fraction of the overall slope, sign included.
DRIFT_SEGMENT_SLOPE_FRACTION = 0.5


def _median_offset(entry):
    pairs = list(zip(entry.receive_times, entry.stamps, strict=True))
    if not pairs:
        return None
    return median(received - stamp for received, stamp in pairs)


def _direction(offset):
    return "stamps behind the receiver" if offset > 0 else "stamps ahead of the receiver"


def _first_nonzero(stamps):
    for stamp in stamps:
        if stamp != 0.0:
            return stamp
    return None


def _implausible_epoch(topic, offset, stamp, stamps):
    year = time.gmtime(stamp).tm_year
    return Finding(
        check="epoch_offset",
        topic=topic,
        severity="error",
        message=f"{topic} stamps sit in year {year}, before the plausibility floor of 2005, "
        "source clock was never set or a post processing shift moved them",
        evidence={
            "median_offset_s": offset,
            "class": "unsynced epoch",
            "first_stamp_s": stamp,
            "sample_stamps": stamps[:SAMPLE_STAMPS],
        },
    )


def classify_offset(topic, offset, stamps):
    """Grade one median offset as (class name, severity, message)."""
    size = abs(offset)
    if size > UNSYNCED_OFFSET_S or stamps[0] < MIN_PLAUSIBLE_STAMP_S:
        return (
            "unsynced epoch",
            "error",
            f"{topic} stamps are {size:.0f} s off the receiving clock, "
            f"source clock was never set, {_direction(offset)}",
        )
    if GPS_UTC_MIN_S <= size <= GPS_UTC_MAX_S:
        return (
            "gps utc leap offset",
            "error",
            f"{topic} offset {size:.3f} s matches the GPS to UTC leap offset (18 s), "
            f"the source likely serves GPS time as UTC, {_direction(offset)}",
        )
    if TAI_UTC_MIN_S <= size <= TAI_UTC_MAX_S:
        return (
            "tai utc offset",
            "error",
            f"{topic} offset {size:.3f} s matches the TAI to UTC offset (37 s), "
            f"check the PTP timescale or a ptp_utc_tai_offset driver setting, {_direction(offset)}",
        )
    if offset < -MAX_PLAUSIBLE_LATENCY_S:
        return (
            "stamps from the future",
            "error",
            f"{topic} stamps run {size:.3f} s ahead of the receiving clock",
        )
    return (
        "unclassified offset",
        "warning",
        f"{topic} offset {offset:.3f} s is beyond the plausible latency of "
        f"{MAX_PLAUSIBLE_LATENCY_S:.1f} s",
    )


def check_epoch_offset(series):
    findings = []
    for topic, entry in sorted(series.items()):
        offset = _median_offset(entry)
        if offset is None:
            continue
        first = _first_nonzero(entry.stamps)
        if first is not None and first < MIN_PLAUSIBLE_STAMP_S:
            findings.append(_implausible_epoch(topic, offset, first, entry.stamps))
            continue
        if abs(offset) <= MAX_PLAUSIBLE_LATENCY_S:
            continue
        name, severity, message = classify_offset(topic, offset, entry.stamps)
        findings.append(
            Finding(
                check="epoch_offset",
                topic=topic,
                severity=severity,
                message=message,
                evidence={
                    "median_offset_s": offset,
                    "class": name,
                    "sample_stamps": entry.stamps[:SAMPLE_STAMPS],
                },
            )
        )
    return findings


def check_inter_topic_skew(series):
    offsets = {}
    for topic, entry in series.items():
        offset = _median_offset(entry)
        if offset is not None:
            offsets[topic] = offset
    if len(offsets) < MIN_TOPICS_FOR_SKEW:
        return []
    reference = median(offsets.values())
    findings = []
    for topic, offset in sorted(offsets.items()):
        delta = offset - reference
        if abs(delta) > SKEW_ERROR_S:
            severity = "error"
        elif abs(delta) > SKEW_WARNING_S:
            severity = "warning"
        else:
            continue
        findings.append(
            Finding(
                check="inter_topic_skew",
                topic=topic,
                severity=severity,
                message=f"{topic} offset {offset:.3f} s deviates {delta:.3f} s "
                f"from the bag reference {reference:.3f} s",
                evidence={
                    "topic_offset_s": offset,
                    "reference_offset_s": reference,
                    "delta_s": delta,
                },
            )
        )
    return findings


def _fit(times, offsets):
    count = len(times)
    mean_t = sum(times) / count
    mean_y = sum(offsets) / count
    spread = sum((t - mean_t) ** 2 for t in times)
    if spread == 0.0:
        return None, None
    slope = sum((t - mean_t) * (y - mean_y) for t, y in zip(times, offsets, strict=True)) / spread
    residual = (
        sum(abs(y - mean_y - slope * (t - mean_t)) for t, y in zip(times, offsets, strict=True))
        / count
    )
    return slope, residual


def _segment_ratio(times, offsets, slope):
    half = len(times) // 2
    ratios = []
    for lo, hi in ((0, half), (half, len(times))):
        part, _ = _fit(times[lo:hi], offsets[lo:hi])
        if part is None:
            return 0.0
        ratios.append(part / slope)
    return min(ratios)


def _drift_severity(slope):
    if abs(slope) > DRIFT_ERROR_S_PER_S:
        return "error"
    if abs(slope) > DRIFT_WARN_S_PER_S:
        return "warning"
    return None


def check_clock_drift(series):
    findings = []
    for topic, entry in sorted(series.items()):
        times = entry.receive_times
        if len(times) < DRIFT_MIN_MESSAGES:
            continue
        span = max(times) - min(times)
        if span < DRIFT_MIN_SPAN_S:
            continue
        offsets = [received - stamp for received, stamp in zip(times, entry.stamps, strict=True)]
        slope, residual = _fit(times, offsets)
        if slope is None:
            continue
        severity = _drift_severity(slope)
        if severity is None:
            continue
        ratio = _segment_ratio(times, offsets, slope)
        if ratio < DRIFT_SEGMENT_SLOPE_FRACTION:
            continue
        findings.append(
            Finding(
                check="clock_drift",
                topic=topic,
                severity=severity,
                message=f"{topic} clock drifts at {slope * 1000.0:.3f} ms/s against the receiver, "
                "its source clock is unsynchronized",
                evidence={
                    "slope_s_per_s": slope,
                    "span_s": span,
                    "messages": len(times),
                    "mean_abs_residual_s": residual,
                    "segment_slope_ratio": ratio,
                },
            )
        )
    return findings


_classify = classify_offset

CHECKS = {
    "epoch_offset": check_epoch_offset,
    "inter_topic_skew": check_inter_topic_skew,
    "clock_drift": check_clock_drift,
}
