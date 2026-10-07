"""
Candidate selection against the buffered AOI.

GPS_POINT is the algorithm validated by Regression Baseline 001 (local data):
photo GPS (WGS84) → buffer CRS → ``shapely.covers`` (points on the boundary count as inside).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field

import geopandas as gpd
import shapely

from ..metadata.models import PhotoMetadata
from .buffer import AOIBuffer
from .crs import PHOTO_CRS


def select_by_gps_point(photos: list[PhotoMetadata], aoi: AOIBuffer
                        ) -> tuple[list[PhotoMetadata], list[PhotoMetadata]]:
    """Return (inside, outside). All ``photos`` must have GPS."""
    if not photos:
        return [], []
    points = gpd.GeoSeries(
        gpd.points_from_xy([p.longitude for p in photos], [p.latitude for p in photos]),
        crs=PHOTO_CRS,
    ).to_crs(aoi.crs)
    covered = shapely.covers(aoi.geometry, points.values)
    inside = [p for p, ok in zip(photos, covered) if ok]
    outside = [p for p, ok in zip(photos, covered) if not ok]
    return inside, outside


@dataclass
class FootprintSelection:
    inside: list = field(default_factory=list)        # [(PhotoMetadata, PhotoFootprint)]
    outside: list = field(default_factory=list)       # [(PhotoMetadata, PhotoFootprint)]
    unavailable: list = field(default_factory=list)   # [(PhotoMetadata, reason)]
    unresolved_inside: list = field(default_factory=list)   # [(PhotoMetadata, reason)]
    warning_counts: Counter = field(default_factory=Counter)


def select_by_footprint(photos: list[PhotoMetadata], aoi: AOIBuffer, projector,
                        context, lens_mode=None) -> FootprintSelection:
    """
    FOOTPRINT mode: Estimated Ground Footprint ``intersects`` the buffered AOI.
    ``context.target_crs`` must be the buffer CRS. Heights follow Height Strategy v2
    (flight analysis + neighbour interpolation). A HEIGHT_UNRESOLVED photo has no footprint;
    it is kept as a candidate conservatively when its GPS point is covered by the buffer.
    """
    from ..camera.models import LensMode
    from ..flight.strip import analyse_flights
    from ..footprint.batch import project_all

    lens_mode = lens_mode or LensMode.AUTO
    out = FootprintSelection()
    analysis = analyse_flights(photos, context.target_crs)
    batch = project_all(photos, analysis, projector, context, lens_mode)
    unresolved = set(batch.unresolved)
    gps_in = {m.photo_id for m in select_by_gps_point(
        [m for m in photos if m.photo_id in unresolved], aoi)[0]}
    for meta in photos:
        fp = batch.footprints.get(meta.photo_id)
        if fp is None:
            reason = batch.unavailable.get(meta.photo_id, "footprint unavailable")
            if meta.photo_id in gps_in:
                out.unresolved_inside.append((meta, reason + "; candidate by GPS point"))
            else:
                out.unavailable.append((meta, reason))
            continue
        out.warning_counts.update(w.value for w in fp.warnings)
        (out.inside if shapely.intersects(aoi.geometry, fp.geometry) else out.outside
         ).append((meta, fp))
    return out
