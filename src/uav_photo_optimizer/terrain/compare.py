"""Planar vs terrain footprint comparison (Part 25)."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import shapely
from shapely.geometry.base import BaseGeometry

OBLIQUE_DEG = 5.0


def _pct(v: list[float], q) -> Optional[float]:
    return round(float(np.percentile(v, q)), 4) if v else None


def summary(values: list[float]) -> dict:
    return {"n": len(values), "median": _pct(values, 50), "p05": _pct(values, 5),
            "p95": _pct(values, 95), "min": round(min(values), 4) if values else None,
            "max": round(max(values), 4) if values else None}


def compare_footprints(planar: dict, terrain: dict, area: Optional[BaseGeometry] = None) -> dict:
    """Per capture type: area ratio, centroid shift, Hausdorff distance; candidate counts."""
    out: dict = {"photos_planar": len(planar), "photos_terrain": len(terrain),
                 "terrain_missing": sorted(set(planar) - set(terrain)).__len__()}
    groups: dict[str, dict[str, list[float]]] = {}
    for pid in sorted(set(planar) & set(terrain)):
        a, b = planar[pid], terrain[pid]
        off = a.provenance.get("pitch_deg")
        kind = "NADIR" if off is not None and 90 + off <= OBLIQUE_DEG else "OBLIQUE"
        g = groups.setdefault(kind, {"area_ratio": [], "centroid_shift_m": [],
                                     "hausdorff_m": [], "rel_hausdorff": []})
        ca, cb = a.geometry.centroid, b.geometry.centroid
        hd = a.geometry.hausdorff_distance(b.geometry)
        g["area_ratio"].append(b.geometry.area / a.geometry.area)
        g["centroid_shift_m"].append(math.hypot(ca.x - cb.x, ca.y - cb.y))
        g["hausdorff_m"].append(hd)
        g["rel_hausdorff"].append(hd / math.sqrt(a.geometry.area))
    out["by_capture"] = {k: {m: summary(v) for m, v in g.items()} for k, g in sorted(groups.items())}
    if area is not None:
        shapely.prepare(area)
        cp = {p for p, f in planar.items() if shapely.intersects(area, f.geometry)}
        ct = {p for p, f in terrain.items() if shapely.intersects(area, f.geometry)}
        out["candidates"] = {"planar": len(cp), "terrain": len(ct), "both": len(cp & ct),
                             "planar_only": len(cp - ct), "terrain_only": len(ct - cp)}
    return out
