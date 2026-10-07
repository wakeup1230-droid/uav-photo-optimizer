"""
Terrain-aware footprints (Phase 7, Experimental): local GeoTIFF height fields only.

Optional dependencies: ``pip install 'uav-photo-optimizer[terrain]'`` (rasterio, weitsicht).
Importing this package needs neither; providers import them lazily.
"""

from .base import TerrainProvider
from .models import (TerrainCheck, TerrainConfig, TerrainDiagnostics, TerrainEngine,
                     TerrainIntersectionResult, TerrainIssue, TerrainSource, TerrainStatus,
                     TerrainSurfaceKind, VerticalAlignmentMode, VerticalDatumKind)


def make_provider(config: TerrainConfig) -> TerrainProvider:
    if config.engine is TerrainEngine.WEITSICHT:
        from .weitsicht_provider import WeitsichtRasterTerrainProvider
        return WeitsichtRasterTerrainProvider(config.source, preload=config.preload)
    from .reference import ReferenceRasterRaySolver
    return ReferenceRasterRaySolver(config.source, preload=config.preload,
                                    step_fraction=config.step_fraction)


__all__ = ["TerrainProvider", "TerrainCheck", "TerrainConfig", "TerrainDiagnostics",
           "TerrainEngine", "TerrainIntersectionResult", "TerrainIssue", "TerrainSource",
           "TerrainStatus", "TerrainSurfaceKind", "VerticalAlignmentMode", "VerticalDatumKind",
           "make_provider"]
