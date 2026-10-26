"""The header stamp checks: reading stamps out of a bag, the checks over them and the verdict.

The names re-exported here are the supported cross package surface. Everything
else in the package is internal and may be renamed in a patch release.
"""

from rosio import __version__

from .clock import MAX_PLAUSIBLE_LATENCY_S, classify_offset
from .fallback import UnsupportedEncapsulation, cdr_stamp_parts
from .read import probe_stamp_parser, stamp_from_cdr, starts_with_header
from .stream import (
    FROZEN_MIN_RUN,
    FROZEN_MIN_SPAN_S,
    largest_backward_step,
    longest_frozen_run,
)

__all__ = [
    "FROZEN_MIN_RUN",
    "FROZEN_MIN_SPAN_S",
    "MAX_PLAUSIBLE_LATENCY_S",
    "UnsupportedEncapsulation",
    "__version__",
    "cdr_stamp_parts",
    "classify_offset",
    "largest_backward_step",
    "longest_frozen_run",
    "probe_stamp_parser",
    "stamp_from_cdr",
    "starts_with_header",
]
