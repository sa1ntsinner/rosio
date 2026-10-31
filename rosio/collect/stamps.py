"""Header stamp checks over the recordings in configured bag roots."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rosio import extra_line, ids, model
from rosio.collect import CollectContext, register
from rosio.collect.bag import BAG_PKG, METADATA_NAME, STORAGE_SUFFIXES, BagCollector
from rosio.errors import CollectorError
from rosio.stamps.checks import run_checks, stamped_series
from rosio.stamps.clock import median_offset
from rosio.stamps.model import TopicSeries
from rosio.stamps.read import read_topic_series_report
from rosio.stamps.verdict import diagnose

NAME = "stamps"
LAYER = model.Layer.RECORDED.value
STAMPS_EXTRA = "rosbags>=0.11.4,<0.12"


def require_reader() -> None:
    try:
        from rosbags import typesys
    except ImportError as exc:
        raise CollectorError(
            f"header stamp checks need the stamps extra ({STAMPS_EXTRA}), "
            f"install it with: {extra_line('stamps')}",
            LAYER,
            NAME,
        ) from exc
    del typesys


def rounded(value: Any, key: str = "") -> Any:
    """One precision for stored stamp measurements, seconds at nine places, rates at six."""
    if isinstance(value, float):
        return round(value, 6 if key == "rate_hz" else 9)
    if isinstance(value, dict):
        return {name: rounded(item, name) for name, item in sorted(value.items())}
    if isinstance(value, list):
        return [rounded(item, key) for item in value]
    return value


def analyze(series: dict[str, TopicSeries]) -> tuple[list[Any], dict[str, Any]]:
    """Use one check order and verdict shape for recorded and live samples."""
    results = run_checks(series)
    verdict = diagnose(series, results)
    return results, {"level": verdict.level, "hypotheses": sorted(verdict.hypotheses)}


def check_rows(results: list[Any], topic: str) -> list[dict[str, Any]]:
    return [
        {
            "check": row.check,
            "severity": row.severity,
            "message": row.message,
            "evidence": row.evidence,
        }
        for row in sorted(results, key=lambda row: (row.check, row.severity))
        if row.topic == topic and row.check != "stamp_rate"
    ]


def _problem(path: str, message: str, exc_type: str) -> dict[str, str]:
    bag_id = ids.build_bag(Path(path).name)
    message = message.replace(path, bag_id)
    message = re.sub(r"(?<![\w:])(?:[A-Za-z]:[\\/]|/)[^\s'\"<>]+", bag_id, message)
    return {"file": bag_id, "phase": NAME, "exc_type": exc_type, "message": message}


def _evidence(ctx: CollectContext, path: str) -> list[model.Source]:
    directory = Path(path)
    evidence = []
    for item in sorted(directory.iterdir()):
        if not item.is_file() or (
            item.suffix not in STORAGE_SUFFIXES and item.name != METADATA_NAME
        ):
            continue
        file_id = ids.build_file(BAG_PKG, f"{directory.name}/{item.name}")
        ctx.builder.node(file_id)
        evidence.append(
            model.Source(
                file=file_id,
                locator="rosbag2_bagfile_information" if item.name == METADATA_NAME else "",
                root=model.Root.BAG.value,
            )
        )
    return evidence


@dataclass
class StampsCollector:
    """Recorded stamp measurements, independent of the metadata collector's output."""

    name: str = NAME
    layer: str = LAYER
    paths: tuple[str, ...] = ()

    def inputs_present(self, ctx: CollectContext) -> bool | str:
        return BagCollector(paths=self.paths).inputs_present(ctx)

    def collect(self, ctx: CollectContext) -> None:
        paths, problems = BagCollector(paths=self.paths).recordings(ctx)
        if not paths and not problems:
            return
        require_reader()
        run = ctx.run or ctx.builder.add_run(
            LAYER,
            input_digest=ids.digest(NAME, *ctx.ws.root_hashes().values()),
        )
        names = [Path(path).name for path in paths]
        for path in paths:
            if names.count(Path(path).name) > 1:
                problems.append(
                    _problem(path, "recording basename is not unique", "CollectorError")
                )
                continue
            try:
                self._emit(ctx, run, path, problems)
            except Exception as exc:
                problem = _problem(path, str(exc), type(exc).__name__)
                problems.append(problem)
                self._refuse(ctx, path, problem["message"])
        ctx.builder.atlas.entities[run].attrs["exceptions"].extend(
            sorted(problems, key=lambda row: (row["file"], row["phase"], row["message"]))
        )

    def _refuse(self, ctx: CollectContext, path: str, detail: str) -> None:
        evidence = _evidence(ctx, path)
        if not evidence:
            return
        source = next((row for row in evidence if not row.locator), evidence[0])
        ctx.builder.node(
            source.file,
            attrs={
                "loadable": False,
                "refused_because": "stamps_unreadable",
                "refused_detail": detail,
            },
        )
        ctx.builder.unresolved(
            model.Unresolved(
                want=model.Want.OTHER.value,
                owner=source.file,
                expr=f"header stamps in {Path(path).name}",
                reason="stamps_unreadable",
                role="header_stamps",
                evidence=evidence,
            )
        )

    def _emit(
        self, ctx: CollectContext, run: str, path: str, problems: list[dict[str, Any]]
    ) -> None:
        report = read_topic_series_report(path)
        problems.extend(_problem(path, str(notice), "ReaderNotice") for notice in report.notices)
        results, verdict = analyze(report.series)
        evidence = _evidence(ctx, path)
        name = Path(path).name
        ctx.builder.fact(
            ids.build_bag(name),
            LAYER,
            run=run,
            values={"stamp_verdict": verdict},
            evidence=evidence,
        )
        stamped = stamped_series(report.series)
        for topic, series in sorted(report.series.items()):
            checks = check_rows(results, topic)
            volatile = {}
            for result in sorted(results, key=lambda row: (row.check, row.severity)):
                if result.topic != topic:
                    continue
                if result.check == "stamp_rate":
                    volatile = {
                        key: result.evidence[key]
                        for key in ("rate_hz", "p95_jitter_s", "median_dt_s")
                    }
            values: dict[str, Any] = {"stamps": {"messages": len(series.stamps), "checks": checks}}
            offset = median_offset(series) if topic in stamped else None
            if offset is not None:
                values["stamps"]["offset_s"] = offset
            if volatile:
                values["volatile"] = {"stamps": volatile}
            ctx.builder.fact(
                ids.build_bagtopic(name, topic),
                LAYER,
                run=run,
                values=rounded(values),
                evidence=evidence,
            )


register(NAME, StampsCollector)
