from . import clock, stream

MODULES = (stream, clock)


def _merge(modules):
    merged = {}
    for module in modules:
        for name, check in module.CHECKS.items():
            if name in merged:
                raise RuntimeError(f"duplicate check name {name} in {module.__name__}")
            merged[name] = check
    return merged


CHECKS = _merge(MODULES)


def stamped_series(series):
    return {topic: entry for topic, entry in series.items() if any(entry.stamps)}


def run_checks(series):
    stamped = stamped_series(series)
    return [
        result
        for name, check in sorted(CHECKS.items())
        for result in check(series if name == "zero_stamps" else stamped)
    ]


__all__ = ["CHECKS", "MODULES", "clock", "run_checks", "stream"]
