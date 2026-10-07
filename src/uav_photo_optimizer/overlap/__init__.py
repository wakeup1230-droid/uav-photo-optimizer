"""Overlap engine (Phase 5): directional + shared-coverage overlap. Calculation only."""

from .engine import OverlapConfig, OverlapEngine, overlap_statistics
from .models import (OverlapKind, OverlapMethod, OverlapReport, OverlapResult,
                     PhotoAdjacencyGraph, StripPair, TerrainMode)

__all__ = ["OverlapConfig", "OverlapEngine", "overlap_statistics", "OverlapKind",
           "OverlapMethod", "OverlapReport", "OverlapResult", "PhotoAdjacencyGraph",
           "StripPair", "TerrainMode"]
