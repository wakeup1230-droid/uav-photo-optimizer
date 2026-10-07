"""
Flight / Flight Strip models (Phase 4).

* Flight      — one relatively continuous flight mission (time / sequence / space continuity)
* FlightStrip — a segment of one flight with near-constant travel heading and a near-linear
                track (one "flight line"); built from the camera GPS trajectory, capture order
                and FlightYaw — never from GimbalYaw (obliques look in other directions)

Headings are grid bearings in the projection CRS (clockwise from grid north):
    axis_heading   ∈ [0, 180)   the undirected line (90 and 270 → 90)
    travel_heading ∈ [0, 360)   the direction actually flown
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field


class CaptureType(str, Enum):
    NADIR = "NADIR"
    OBLIQUE = "OBLIQUE"
    OTHER = "OTHER"


class SegmentType(str, Enum):
    STRIP = "STRIP"                  # straight flight line, parallel to the flight's main axis
    TURN = "TURN"                    # heading changing between lines
    TRANSIT = "TRANSIT"              # straight, but not a mapping line (other axis / connector)
    UNCLASSIFIED = "UNCLASSIFIED"    # heading undefined (hover) or too few photos


class ViewDirection(str, Enum):
    """Oblique look direction relative to the strip travel heading (8 sectors)."""

    FORWARD = "FORWARD"
    FORWARD_RIGHT = "FORWARD_RIGHT"
    RIGHT = "RIGHT"
    BACKWARD_RIGHT = "BACKWARD_RIGHT"
    BACKWARD = "BACKWARD"
    BACKWARD_LEFT = "BACKWARD_LEFT"
    LEFT = "LEFT"
    FORWARD_LEFT = "FORWARD_LEFT"
    NOT_APPLICABLE = "NOT_APPLICABLE"   # nadir / no strip
    OTHER = "OTHER"


class Flight(BaseModel):
    model_config = ConfigDict(extra="forbid")

    flight_id: str
    photo_ids: list[str] = Field(default_factory=list)   # capture order
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    folders: list[str] = Field(default_factory=list)
    split_reason: str = ""          # why this flight starts a new one


class FlightStrip(BaseModel):
    """A straight flight line within one flight."""

    model_config = ConfigDict(extra="forbid")

    strip_id: str
    flight_id: str
    photo_ids: list[str] = Field(default_factory=list)   # capture order
    heading: Optional[float] = None                       # = travel_heading (backwards compat.)
    axis_heading: Optional[float] = None                  # [0, 180)
    travel_heading: Optional[float] = None                # [0, 360)
    length_m: float = 0.0
    start_xy: Optional[tuple[float, float]] = None
    end_xy: Optional[tuple[float, float]] = None
    centre_xy: Optional[tuple[float, float]] = None
    cross_track_rms_m: float = 0.0
    n_nadir: int = 0
    n_oblique: int = 0
    merged_from: int = 1


class PhotoTrack(BaseModel):
    """Per-photo flight analysis."""

    model_config = ConfigDict(extra="forbid")

    photo_id: str
    flight_id: str
    sequence_index: int                    # 0-based position in the flight's capture order
    xy: tuple[float, float]
    segment_type: SegmentType
    segment_index: int
    strip_id: Optional[str] = None
    capture_type: CaptureType
    track_heading: Optional[float] = None  # local trajectory heading (grid, [0, 360))
    heading_source: str = ""               # "TRAJECTORY" / "FLIGHT_YAW" / ""
    travel_heading: Optional[float] = None # strip travel heading (when in a strip)
    axis_heading: Optional[float] = None
    gimbal_pitch: Optional[float] = None
    gimbal_yaw_grid: Optional[float] = None
    relative_view_yaw: Optional[float] = None   # (-180, 180], gimbal yaw − strip travel heading
    view_direction: ViewDirection = ViewDirection.NOT_APPLICABLE


class FlightAnalysis(BaseModel):
    model_config = ConfigDict(extra="forbid")

    crs: str
    flights: list[Flight] = Field(default_factory=list)
    strips: list[FlightStrip] = Field(default_factory=list)
    photos: dict[str, PhotoTrack] = Field(default_factory=dict)
    warnings: list[str] = Field(default_factory=list)

    def counts(self) -> dict[str, int]:
        from collections import Counter
        seg = Counter(p.segment_type.value for p in self.photos.values())
        cap = Counter(p.capture_type.value for p in self.photos.values())
        return {"flights": len(self.flights), "strips": len(self.strips),
                **{f"photos_{k}": v for k, v in seg.items()},
                **{f"capture_{k}": v for k, v in cap.items()}}
