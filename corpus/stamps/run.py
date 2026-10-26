"""Run the stamp checks over the fetched corpus and check the manifest expectations."""

import argparse
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

from fetch import DEFAULT_DEST, HERE, MANIFEST, TIERS, load_manifest, select

from rosio.stamps.checks import CHECKS
from rosio.stamps.read import read_topic_series
from rosio.stamps.verdict import NOT_ANALYZED

BAG_SUFFIXES = (".bag", ".mcap", ".db3")
DEFAULT_LIMIT = 5000
NO_SAMPLES = "no samples were analyzed"
REPO = HERE.parents[1]


def positive_int(text):
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text} is not a number") from None
    if value < 1:
        raise argparse.ArgumentTypeError(f"needs a positive number, got {value}")
    return value


def generate(entry, dest):
    """Build a synthetic entry in place from the commands in its manifest record."""
    work = dest.parent / (".work-" + entry["name"])
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    fields = {"python": sys.executable, "dest": str(dest), "work": str(work)}
    try:
        for command in entry["generate"]:
            argv = [part.format(**fields) for part in shlex.split(command)]
            subprocess.run(argv, cwd=str(REPO), check=True, stdout=subprocess.DEVNULL)
    finally:
        shutil.rmtree(work, ignore_errors=True)


def find_data(root):
    """Locate the readable bag inside an entry directory."""
    if not root.exists():
        return None
    if root.is_file():
        return root
    if (root / "metadata.yaml").exists():
        return root
    for suffix in BAG_SUFFIXES:
        paths = sorted(root.rglob("*" + suffix))
        if not paths:
            continue
        first = paths[0]
        return first.parent if (first.parent / "metadata.yaml").exists() else first
    return None


def analyse(path, limit):
    notices = []
    series = read_topic_series(path, limit=limit, notices=notices)
    findings = []
    for check in CHECKS.values():
        findings.extend(check(series))
    return series, findings, notices


def matches(rule, finding):
    if rule.get("check") and rule["check"] != finding.check:
        return False
    if rule.get("severity") and rule["severity"] != finding.severity:
        return False
    topic = rule.get("topic_contains")
    return not topic or topic in finding.topic


def describe(rule):
    parts = [str(rule[key]) for key in ("check", "severity", "topic_contains") if rule.get(key)]
    return " ".join(parts)


def evaluate(entry, findings):
    failures = []
    for rule in entry.get("expect") or []:
        if not any(matches(rule, f) for f in findings):
            failures.append(f"expected {describe(rule)} but nothing matched")
    for rule in entry.get("forbid") or []:
        hits = [f for f in findings if matches(rule, f)]
        if hits:
            failures.append(
                f"forbade {describe(rule)} but {len(hits)} finding(s) matched, "
                f"first on {hits[0].topic}"
            )
    return failures


def report(entry, path, series, findings, notices, failures):
    print("=== {} [{}] {}".format(entry["name"], entry.get("tier", ""), path))
    print(f"  {len(series)} topic(s), {len(findings)} finding(s)")
    for notice in notices:
        print(f"  NOTICE {notice}")
    for finding in findings:
        print(f"  {finding.severity.upper()} {finding.check} {finding.topic}: {finding.message}")
    for failure in failures:
        print(f"  ! {failure}")
    print("  {}".format("FAIL" if failures else "PASS"))


def run_entry(entry, root, limit):
    dest = root / entry["name"]
    if entry.get("generate") and not dest.exists():
        try:
            generate(entry, dest)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(
                "=== {} [{}] generation failed, {}".format(
                    entry["name"], entry.get("tier", ""), exc
                )
            )
            print("  FAIL")
            return False
    path = find_data(dest)
    if path is None:
        print("=== {} [{}] no data under {}".format(entry["name"], entry.get("tier", ""), dest))
        print("  SKIP")
        return None
    try:
        series, findings, notices = analyse(path, limit)
    except (OSError, RuntimeError, ValueError) as exc:
        print("=== {} [{}] {}".format(entry["name"], entry.get("tier", ""), path))
        print(f"  ! read failed, {exc}")
        print("  FAIL")
        return False
    failures = evaluate(entry, findings)
    if not series:
        failures.insert(0, NOT_ANALYZED)
    elif not any(s.stamps for s in series.values()):
        failures.insert(0, NO_SAMPLES)
    report(entry, path, series, findings, notices, failures)
    return not failures


def main(argv=None):
    parser = argparse.ArgumentParser(description="run the stamp checks over the fetched corpus")
    parser.add_argument("--manifest", default=str(MANIFEST))
    parser.add_argument("--dest", default=str(DEFAULT_DEST))
    parser.add_argument("--entry", metavar="NAME", help="run a single entry")
    parser.add_argument("--tier", choices=TIERS)
    parser.add_argument(
        "--limit", type=positive_int, default=DEFAULT_LIMIT, help="messages per topic"
    )
    args = parser.parse_args(argv)

    names = [args.entry] if args.entry else None
    entries = select(load_manifest(args.manifest), names, args.tier)
    root = Path(args.dest).expanduser()

    results = [run_entry(entry, root, args.limit) for entry in entries]
    passed = results.count(True)
    failed = results.count(False)
    print(f"{passed} passed, {failed} failed, {results.count(None)} skipped")
    if failed:
        return 1
    if not passed:
        print("nothing was validated")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
