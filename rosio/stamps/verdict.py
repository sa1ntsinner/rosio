from dataclasses import dataclass, field
from statistics import median

SEVERITY_RANK = {"error": 0, "warning": 1, "info": 2}
NOT_ANALYZED = "no topics with headers were found, nothing was analyzed"
MIN_TOPICS_FOR_GLOBAL = 3
MIN_TOPICS_FOR_STALL = 3
STALL_WINDOW_S = 2.0
DRIFT_HORIZON_S = 600.0

CAUSES = {
    "stamp_gaps": "the other topics kept flowing, so look at that source alone",
    "frozen_stamps": "the driver stopped refreshing the header stamp",
    "zero_stamps": "the publisher leaves the header stamp unset",
    "monotonic": "the source clock stepped backwards or the messages were reordered",
    "inter_topic_skew": "that one source runs offset from the rest of the system",
}


@dataclass
class Verdict:
    hypotheses: list = field(default_factory=list)
    level: str = "ok"


def _magnitude(seconds):
    return f"{seconds:.0f} s" if abs(seconds) >= 1000.0 else f"{seconds:.3f} s"


def _by_check(findings, name):
    return [f for f in findings if f.check == name]


def _topics(series, findings):
    return set(series) or {f.topic for f in findings}


def _worst_severity(findings):
    if not findings:
        return "ok"
    return min((f.severity for f in findings), key=lambda s: SEVERITY_RANK.get(s, 3))


def _listed(topics):
    ordered = sorted(topics)
    if len(ordered) == 1:
        return ordered[0]
    return "{} and {}".format(", ".join(ordered[:-1]), ordered[-1])


def _global_timescale(epoch, topics):
    if len(topics) < MIN_TOPICS_FOR_GLOBAL or {f.topic for f in epoch} < topics:
        return None
    classes = {f.evidence.get("class") for f in epoch}
    if len(classes) != 1:
        return None
    name = classes.pop()
    size = median(abs(f.evidence.get("median_offset_s", 0.0)) for f in epoch)
    return (
        f"All {len(topics)} topics carry the same {name} of about {_magnitude(size)}, "
        f"so the whole system shares one clock problem, "
        "look at the recorder host clock or a global timescale setting rather than the sensors."
    )


def _single_source(epoch):
    hypotheses = []
    groups = {}
    for finding in epoch:
        groups.setdefault(finding.evidence.get("class"), []).append(finding)
    for name in sorted(groups, key=str):
        found = groups[name]
        size = median(abs(f.evidence.get("median_offset_s", 0.0)) for f in found)
        if name == "tai utc offset":
            pointer = (
                "set the PTP timescale to UTC or correct a "
                "ptp_utc_tai_offset style setting on that driver"
            )
        else:
            pointer = "the clock or driver of that source is at fault, the other topics are clean"
        hypotheses.append(
            "{} {} a {} of about {} while the rest of the system is clean, {}.".format(
                _listed(f.topic for f in found),
                "carries" if len(found) == 1 else "carry",
                name,
                _magnitude(size),
                pointer,
            )
        )
    return hypotheses


def _drift(findings):
    hypotheses = []
    for finding in _by_check(findings, "clock_drift"):
        slope = finding.evidence.get("slope_s_per_s", 0.0)
        hypotheses.append(
            f"{finding.topic} drifts against the receiving clock at {slope:.3e} s per s, "
            f"which is {abs(slope) * DRIFT_HORIZON_S:.3f} s of error "
            "after ten minutes, discipline that source clock instead of resampling its stamps."
        )
    return hypotheses


def _stall_cluster(gaps):
    timed = sorted(
        (f for f in gaps if f.evidence.get("worst_gap_receive_time") is not None),
        key=lambda f: f.evidence["worst_gap_receive_time"],
    )
    best = []
    for start in range(len(timed)):
        window = [
            f
            for f in timed[start:]
            if f.evidence["worst_gap_receive_time"]
            - timed[start].evidence["worst_gap_receive_time"]
            <= STALL_WINDOW_S
        ]
        if len(window) > len(best):
            best = window
    return best if len(best) >= MIN_TOPICS_FOR_STALL else []


def _gaps(findings):
    gaps = _by_check(findings, "stamp_gaps")
    cluster = _stall_cluster(gaps)
    hypotheses = []
    if cluster:
        moment = median(f.evidence["worst_gap_receive_time"] for f in cluster)
        worst = max(f.evidence.get("max_gap_s", 0.0) for f in cluster)
        hypotheses.append(
            f"{_listed(f.topic for f in cluster)} all gap at the same moment, up to {worst:.3f} s, "
            "so the recorder or the host stalled "
            f"around t={moment:.3f} rather than each sensor failing on its own."
        )
    for finding in gaps:
        if finding not in cluster:
            hypotheses.append(_condensed(finding))
    return hypotheses


def _condensed(finding):
    cause = CAUSES.get(finding.check)
    return f"{finding.message}, {cause}." if cause else f"{finding.message}."


def diagnose(series, findings):
    findings = list(findings)
    if not series and not findings:
        return Verdict(hypotheses=[NOT_ANALYZED], level="unknown")
    topics = _topics(series, findings)
    level = _worst_severity(findings)

    epoch = _by_check(findings, "epoch_offset")
    hypotheses = []
    shared = _global_timescale(epoch, topics)
    if shared:
        hypotheses.append(shared)
    elif epoch:
        hypotheses.extend(_single_source(epoch))
    hypotheses.extend(_drift(findings))
    hypotheses.extend(_gaps(findings))
    for check in ("frozen_stamps", "zero_stamps", "monotonic", "inter_topic_skew"):
        hypotheses.extend(_condensed(f) for f in _by_check(findings, check))

    if not hypotheses:
        count = len(topics)
        hypotheses.append(
            "No timestamp problems found across {} topic{}.".format(
                count, "" if count == 1 else "s"
            )
        )
    return Verdict(hypotheses=hypotheses, level=level)
