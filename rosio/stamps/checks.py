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

__all__ = ["CHECKS", "MODULES", "clock", "stream"]
