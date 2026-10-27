"""`rosio stamps`: the header stamp checks over one recording, with no ROS installation."""

from __future__ import annotations

import argparse
import statistics
import sys
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path

from rosio import __version__, model

from .checks import CHECKS
from .model import TopicSeries
from .read import read_topic_series_report
from .verdict import diagnose

PROG = "rosio stamps"
SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}


def add_arguments(parser: argparse.ArgumentParser, *, json_flag: bool = True) -> None:
    """The flags of the verb, shared by `rosio stamps` and `python -m rosio.stamps`. Under
    `rosio` the `--json` flag is a global one, so the verb does not add it twice."""
    parser.add_argument("bag", nargs="?", metavar="BAG", help="path to a ROS 1 or ROS 2 bag")
    parser.add_argument(
        "--topics", action="append", metavar="TOPIC", help="only read this topic, repeatable"
    )
    parser.add_argument("--limit", type=int, metavar="N", help="stop after N messages per topic")
    if json_flag:
        parser.add_argument("--json", action="store_true", help="print findings as JSON")
    parser.add_argument(
        "--json-verdict", action="store_true", help="print findings and verdict as one JSON object"
    )
    parser.add_argument("--stats", action="store_true", help="print per topic statistics")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog=PROG, description="timestamp checks for a robot bag")
    add_arguments(parser)
    parser.add_argument("--version", action="version", version=__version__)
    args = parser.parse_args(argv)
    if args.bag is None:
        parser.print_usage()
        return 2
    return run(args)


def run(args: argparse.Namespace) -> int:
    """Read, check, print. Exit 0 clean or warnings only, 1 on an error, 2 when nothing could
    be analyzed."""
    if args.limit is not None and args.limit < 1:
        print(f"{PROG}: --limit needs a positive number", file=sys.stderr)
        return 2

    if not Path(args.bag).exists():
        print(f"{PROG}: no recording at {args.bag}, name a bag directory or file", file=sys.stderr)
        return 2
    if _holds_no_recording(Path(args.bag)):
        print(
            f"{PROG}: {args.bag} holds no recording: a ROS 2 bag directory holds metadata.yaml "
            "or a .db3 or .mcap file, name one of those or a ROS 1 .bag file",
            file=sys.stderr,
        )
        return 2
    try:
        report = read_topic_series_report(args.bag, topics=args.topics, limit=args.limit)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"{PROG}: {exc}", file=sys.stderr)
        return 2

    series = report.series
    for notice in report.notices:
        print(f"NOTICE {notice}", file=sys.stderr)

    findings = []
    for check in CHECKS.values():
        findings.extend(check(series))
    findings.sort(key=lambda f: (SEVERITY_ORDER.get(f.severity, 3), f.check, f.topic))

    verdict = diagnose(series, findings)

    if args.stats:
        print_stats(series)
    if args.json:
        print(model.dumps([asdict(f) for f in findings]), end="")
    elif args.json_verdict:
        payload = {
            "findings": [asdict(f) for f in findings],
            "verdict": {"level": verdict.level, "hypotheses": verdict.hypotheses},
        }
        print(model.dumps(payload), end="")
    else:
        for finding in findings:
            print(f"{finding.severity.upper()} {finding.check} {finding.topic}: {finding.message}")
        print()
        print("verdict:")
        for hypothesis in verdict.hypotheses:
            print(f"  {hypothesis}")

    if verdict.level == "unknown":
        return 2
    return 1 if any(f.severity == "error" for f in findings) else 0


def _holds_no_recording(path: Path) -> bool:
    """A directory with nothing either reader opens. The reader's own refusal names a
    metadata file the person never asked about."""
    return path.is_dir() and not any(
        any(path.glob(pattern)) for pattern in ("metadata.yaml", "*.db3", "*.mcap")
    )


def print_stats(series: dict[str, TopicSeries]) -> None:
    for topic, entry in sorted(series.items()):
        count = len(entry.stamps)
        duration = entry.receive_times[-1] - entry.receive_times[0] if count > 1 else 0.0
        gaps = [b - a for a, b in zip(entry.stamps, entry.stamps[1:], strict=False)]
        gap = statistics.median(gaps) if gaps else 0.0
        rate = 1.0 / gap if gap > 0 else 0.0
        offsets = [r - s for r, s in zip(entry.receive_times, entry.stamps, strict=False)]
        offset = statistics.median(offsets) if offsets else 0.0
        print(
            f"{topic} msgs={count} duration={duration:.3f}s rate={rate:.2f}Hz offset={offset:.6f}s"
        )


if __name__ == "__main__":
    sys.exit(main())
