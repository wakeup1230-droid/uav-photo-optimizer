"""
MeshTerrainProvider — reserved slot (Part 30), NOT implemented.

A future implementation would wrap ``trimesh`` (MIT; optional ``embreex`` backend) for true
3-D surfaces (overhangs, bridges, building facades) loaded from a local mesh (e.g. a
ContextCapture / Metashape export). It must satisfy the same ``TerrainProvider`` contract
(first hit, projected metric CRS, explicit vertical datum). See
internal research notes.
"""

from __future__ import annotations

from .base import TerrainProvider


class MeshTerrainProvider(TerrainProvider):
    name = "MeshTerrainProvider"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError("MeshTerrainProvider is reserved (Planned); use a GeoTIFF "
                                  "height field with ReferenceRasterRaySolver")

    def sample_height(self, x, y):  # pragma: no cover
        raise NotImplementedError

    def intersect_rays(self, origins, directions, max_range=None):  # pragma: no cover
        raise NotImplementedError

    def get_bounds(self):  # pragma: no cover
        raise NotImplementedError
