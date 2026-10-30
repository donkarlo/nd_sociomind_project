"""Uniform-areal-density centroid of a closed 2-D polygon.

This is the center of mass of the *enclosed area*, not an average of
LiDAR return points or the centroid of a perimeter with linear density.
"""
from __future__ import annotations

import math
from collections.abc import Sequence


def polygon_area_centroid(
    vertices: Sequence[tuple[float, float]],
) -> tuple[float, float] | None:
    """Return the polygon's surface-mass centroid, or None if degenerate.

    The vertices describe one closed polygon in cyclic order; no repeated
    closing vertex is needed. Both clockwise and counterclockwise input work.
    Shifting the origin to the first vertex keeps the shoelace sums stable.
    """
    if len(vertices) < 3:
        return None
    origin_x, origin_y = vertices[0]
    if not (math.isfinite(origin_x) and math.isfinite(origin_y)):
        return None

    twice_signed_area = 0.0
    first_moment_x = 0.0
    first_moment_y = 0.0
    for index, (x, y) in enumerate(vertices):
        next_x, next_y = vertices[(index + 1) % len(vertices)]
        if not all(math.isfinite(v) for v in (x, y, next_x, next_y)):
            return None
        x -= origin_x
        y -= origin_y
        next_x -= origin_x
        next_y -= origin_y
        cross = x * next_y - next_x * y
        twice_signed_area += cross
        first_moment_x += (x + next_x) * cross
        first_moment_y += (y + next_y) * cross

    if abs(twice_signed_area) <= 1e-9:
        return None

    return (
        origin_x + first_moment_x / (3.0 * twice_signed_area),
        origin_y + first_moment_y / (3.0 * twice_signed_area),
    )
