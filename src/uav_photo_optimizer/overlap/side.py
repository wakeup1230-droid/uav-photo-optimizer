"""Cross-track overlap of one photo pair (nadir SIDE_OVERLAP / oblique CROSS_TRACK_*)."""

from __future__ import annotations

from shapely.geometry.base import BaseGeometry

from .engine import axis_overlap, unit
from .models import AxisOverlap


def cross_track_overlap(a: BaseGeometry, b: BaseGeometry, travel_heading: float) -> AxisOverlap:
    """Project both footprints on the cross-track axis of a strip with ``travel_heading``."""
    _, v = unit(travel_heading)
    return axis_overlap(a, b, v, travel_heading + 90.0, "CROSS")
