"""Exception hierarchy and the process exit codes each class maps to."""

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_USAGE = 2
EXIT_COLLECTOR = 3
EXIT_CONTRACT = 4
EXIT_DRIFT = 5


class RosioError(Exception):
    """Base class for every error rosio raises on purpose."""

    exit_code = EXIT_ERROR


class UsageError(RosioError):
    """Bad command line, missing input path, unknown selector."""

    exit_code = EXIT_USAGE


class CollectorError(RosioError):
    """A layer collector could not produce a usable result.

    Carries the layer and collector name so partial runs stay attributable.
    """

    exit_code = EXIT_COLLECTOR

    def __init__(self, message: str, layer: str = "", collector: str = "") -> None:
        super().__init__(message)
        self.layer = layer
        self.collector = collector

    def __str__(self) -> str:
        where = ".".join(part for part in (self.layer, self.collector) if part)
        base = super().__str__()
        return f"{where}: {base}" if where else base


class ContractError(RosioError):
    """An atlas or on disk artifact does not satisfy the join contract."""

    exit_code = EXIT_CONTRACT

    def __init__(self, message: str, pointer: str = "") -> None:
        super().__init__(message)
        self.pointer = pointer

    def __str__(self) -> str:
        base = super().__str__()
        return f"{self.pointer}: {base}" if self.pointer else base


class DriftGateError(RosioError):
    """Drift findings crossed the configured gate severity."""

    exit_code = EXIT_DRIFT


def exit_code_for(exc: BaseException) -> int:
    """Exit code for an exception, EXIT_ERROR for anything unexpected."""
    if isinstance(exc, RosioError):
        return exc.exit_code
    if isinstance(exc, KeyboardInterrupt):
        return 130
    if isinstance(exc, FileNotFoundError):
        return EXIT_USAGE
    return EXIT_ERROR


__all__ = [
    "EXIT_COLLECTOR",
    "EXIT_CONTRACT",
    "EXIT_DRIFT",
    "EXIT_ERROR",
    "EXIT_OK",
    "EXIT_USAGE",
    "CollectorError",
    "ContractError",
    "DriftGateError",
    "RosioError",
    "UsageError",
    "exit_code_for",
]
