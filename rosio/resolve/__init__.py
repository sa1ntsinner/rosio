"""Resolution helpers shared by the declared layer collectors.

Nothing here touches the atlas. These modules turn source text and executed launch
descriptions into the plain records defined in `rosio.resolve.records`, and the
collectors turn those records into entities, facts and edges.
"""

from __future__ import annotations

import ast
import warnings


def parse_python(source: str, filename: str = "<unknown>") -> ast.Module:
    """`ast.parse` without the warnings Python prints about the source, such as an invalid
    escape sequence in a third party script. rosio reads that file, it did not write it.
    Python 3.12 calls that a SyntaxWarning and 3.10 a DeprecationWarning, so both go."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return ast.parse(source, filename=filename)


__all__ = ["parse_python", "records"]
