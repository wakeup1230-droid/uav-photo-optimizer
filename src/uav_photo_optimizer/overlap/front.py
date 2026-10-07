"""Along-track overlap of one photo pair (nadir FRONT_OVERLAP / oblique ALONG_TRACK_*)."""

from __future__ import annotations

from shapely.geometry.base import BaseGeometry

from .engine import axis_overlap, unit
from .models import AxisOverlap


def along_track_overlap(a: BaseGeometry, b: BaseGeometry, travel_heading: float) -> AxisOverlap:
    """Project both footprints on the strip's along-track axis (grid bearing ``travel_heading``)."""
    u, _ = unit(travel_heading)
    return axis_overlap(a, b, u, travel_heading, "ALONG")
