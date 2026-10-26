"""Resolution helpers shared by the declared layer collectors.

Nothing here touches the atlas. These modules turn source text and executed launch
descriptions into the plain records defined in `rosio.resolve.records`, and the
collectors turn those records into entities, facts and edges.
"""

from __future__ import annotations

__all__ = ["records"]
