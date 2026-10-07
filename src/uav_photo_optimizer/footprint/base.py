"""
FootprintProjector interface.

The Core depends only on this interface. Concrete projectors:

* ``PlanarProjector``           pure NumPy reference implementation (no extra dependency)
* ``CameraTransformProjector``  adapter around the optional ``cameratransform`` package

Terrain footprints (Phase 7, Experimental): ``terrain/footprint.py`` (TERRAIN_RASTER).

Subclasses implement only ``_ground_points``: ideal pinhole pixel coordinates → ground-plane
(x, y) relative to the camera nadir point. Everything else is shared here so all projectors
are traceable in the same way:

* height selection (Height Strategy v2, LRF quality gate; AUTO never falls back silently —
  an unvalidated height raises ``HeightUnresolvedError``; neighbour interpolation lives in
  ``footprint/batch.py`` and is passed in as an explicit ``height``)
* lens distortion (recorded pixels are undistorted before ray casting)
* pose convention, warnings, confidence, polygon
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from pyproj import CRS, Transformer  # noqa: F401  (Transformer: type of transformer())
from shapely.geometry import Polygon

from ..camera.models import CameraIntrinsics
from ..core.exceptions import UAVPhotoOptimizerError
from ..metadata.altitude import HeightMethod, HeightQuantity, HeightStrategy, ProjectionHeight
from ..metadata.models import PhotoMetadata
from .height import (HEIGHT_CONFIDENCE, lrf_local_plane_height, lrf_ray_height,
                     relative_height, user_height)
from .lrf import LRFQualityResult, LRFQualityStatus, LRFThresholds, assess_lrf
from .models import FootprintMethod, FootprintWarning, PhotoFootprint
from .pose import DJIPose, GridPose, to_grid_pose, wgs84_to

OBLIQUE_THRESHOLD_DEG = 5.0
EDGE_SAMPLES = 16          # boundary samples per image edge when distortion is modelled

GEOMETRY_FACTORS = {       # heuristic, documented in docs/FOOTPRINT_METHOD.md
    FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR: 0.8,
    FootprintWarning.LENS_DISTORTION_IGNORED: 0.95,
    FootprintWarning.PRINCIPAL_POINT_ASSUMED: 0.95,
    FootprintWarning.OBLIQUE_VIEW: 0.85,
    FootprintWarning.RTK_NOT_FIXED: 0.95,
}


class FootprintUnavailableError(UAVPhotoOptimizerError):
    """This photo's footprint cannot be estimated (reason in message)."""


class HeightUnresolvedError(FootprintUnavailableError):
    """HEIGHT_UNRESOLVED: no validated height for this photo."""

    def __init__(self, message: str, lrf_quality: Optional[LRFQualityResult] = None):
        super().__init__(message)
        self.lrf_quality = lrf_quality


@dataclass(frozen=True)
class ProjectionContext:
    target_crs: str                                  # metre-based projected CRS
    height_strategy: HeightStrategy = HeightStrategy.AUTO
    user_ground_elevation: Optional[float] = None    # for USER_GROUND_ELEVATION
    max_range_factor: float = 20.0                   # reject rays farther than k × height
    lrf_thresholds: LRFThresholds = field(default_factory=LRFThresholds)

    def transformer(self) -> Transformer:
        return wgs84_to(self.target_crs)


@dataclass
class _Geometry:
    polygon: Polygon
    corners: list[tuple[float, float]]
    principal: Optional[tuple[float, float]]


def dji_gimbal_pose(meta: PhotoMetadata) -> DJIPose:
    if not meta.has_gimbal_pose:
        raise FootprintUnavailableError("gimbal pose incomplete")
    return DJIPose(meta.gimbal_yaw, meta.gimbal_pitch, meta.gimbal_roll,
                   sources={k: meta.sources.get(f"gimbal_{k}") for k in ("yaw", "pitch", "roll")})


def boundary_pixels(camera: CameraIntrinsics, edge_samples: Optional[int] = None
                    ) -> tuple[np.ndarray, list[int]]:
    """
    Recorded-image boundary TL→TR→BR→BL (+ edge samples if distorted); corner indices.
    ``edge_samples`` forces that many samples per edge (terrain footprints, Phase 7).
    """
    w, h = camera.width_px, camera.height_px
    corners = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
    if camera.distortion is None and edge_samples is None:
        return np.array(corners, dtype=float), [0, 1, 2, 3]
    n = edge_samples or EDGE_SAMPLES
    pts, idx = [], []
    for i in range(4):
        (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
        idx.append(len(pts))
        pts.extend((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n))
    return np.array(pts, dtype=float), idx


class FootprintProjector(ABC):
    name: str = "abstract"

    @abstractmethod
    def _ground_points(self, pose: GridPose, camera: CameraIntrinsics, height_m: float,
                       pixels: np.ndarray) -> np.ndarray:
        """(N, 2) ideal pinhole pixels → (N, 2) ground (dx, dy) from camera nadir; NaN if none."""

    # -- geometry --------------------------------------------------------------------------

    def _geometry(self, pose: GridPose, camera: CameraIntrinsics, height_m: float,
                  x0: float, y0: float, max_range_factor: float) -> _Geometry:
        boundary, corner_idx = boundary_pixels(camera)
        ideal = camera.distortion.undistort_pixels(boundary) if camera.distortion else boundary
        pixels = np.vstack([ideal, [[camera.cx_px, camera.cy_px]]])
        ground = self._ground_points(pose, camera, height_m, pixels)
        ring, centre = ground[:-1], ground[-1]
        if not np.isfinite(ring).all():
            raise FootprintUnavailableError("image boundary ray above horizon (HORIZON_IN_VIEW)")
        if np.hypot(ring[:, 0], ring[:, 1]).max() > max_range_factor * height_m:
            raise FootprintUnavailableError("footprint exceeds max range (near-horizon view)")
        coords = [(x0 + dx, y0 + dy) for dx, dy in ring]
        polygon = Polygon(coords)
        if not polygon.is_valid or polygon.area <= 0:
            raise FootprintUnavailableError("degenerate footprint polygon")
        principal = (x0 + centre[0], y0 + centre[1]) if np.isfinite(centre).all() else None
        return _Geometry(polygon, [coords[i] for i in corner_idx], principal)

    # -- main ------------------------------------------------------------------------------

    def project(self, meta: PhotoMetadata, camera: CameraIntrinsics,
                context: ProjectionContext,
                height: Optional[ProjectionHeight] = None,
                lrf_quality: Optional[LRFQualityResult] = None) -> PhotoFootprint:
        """
        Estimate the footprint. ``height`` (e.g. neighbour-interpolated, see
        ``footprint/batch.py``) overrides the strategy. AUTO without ``height`` accepts only a
        VALID laser height; otherwise ``HeightUnresolvedError`` (never RelativeAltitude).
        """
        if not meta.has_gps:
            raise FootprintUnavailableError("no GPS position")
        pose = to_grid_pose(dji_gimbal_pose(meta), meta.longitude, meta.latitude,
                            context.target_crs)
        to_grid = context.transformer()
        x0, y0 = to_grid.transform(meta.longitude, meta.latitude)
        warnings: list[FootprintWarning] = [FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR]
        strategy = context.height_strategy
        geom: Optional[_Geometry] = None
        lrf_q = lrf_quality

        if height is None:
            if strategy in (HeightStrategy.AUTO, HeightStrategy.LRF_RAY_VERTICAL):
                height, geom, lrf_q = self._try_lrf(meta, camera, pose, x0, y0, context,
                                                    to_grid)
                accepted = ({LRFQualityStatus.VALID} if strategy is HeightStrategy.AUTO
                            else {LRFQualityStatus.VALID, LRFQualityStatus.SUSPECT})
                if height is None or lrf_q.status not in accepted:
                    detail = f" ({'; '.join(lrf_q.reasons)})" if lrf_q.reasons else ""
                    raise HeightUnresolvedError(
                        f"HEIGHT_UNRESOLVED: LRF {lrf_q.status.value}{detail}", lrf_q)
                if lrf_q.status is LRFQualityStatus.SUSPECT:
                    warnings.append(FootprintWarning.LRF_SUSPECT)
                    height = height.model_copy(
                        update={"confidence": HEIGHT_CONFIDENCE["LRF_SUSPECT"]})
            elif strategy is HeightStrategy.LRF_LOCAL_PLANE:
                height = lrf_local_plane_height(meta)
                if height is None:
                    raise FootprintUnavailableError("AbsoluteAltitude / LRFTargetAbsAlt missing")
                warnings.append(FootprintWarning.ALTITUDE_DIFFERENCE_HEIGHT)
            elif strategy is HeightStrategy.TAKEOFF_RELATIVE:
                height = relative_height(meta)
                if height is None:
                    raise FootprintUnavailableError("RelativeAltitude missing")
            elif strategy is HeightStrategy.USER_GROUND_ELEVATION:
                height = user_height(meta, context.user_ground_elevation)
                if height is None:
                    raise FootprintUnavailableError(
                        "user ground elevation or AbsoluteAltitude missing")
                warnings.append(FootprintWarning.GROUND_PLANE_USER)

        if height.height_m <= 0:
            raise FootprintUnavailableError(
                f"non-positive height above ground plane ({height.height_m:.2f} m)")
        if height.method is HeightMethod.TAKEOFF_RELATIVE:
            warnings += [FootprintWarning.ALTITUDE_TAKEOFF_RELATIVE,
                         FootprintWarning.LOW_CONFIDENCE_HEIGHT]
        if height.method is HeightMethod.NEIGHBOR_LRF_INTERPOLATED:
            warnings.append(FootprintWarning.NEIGHBOR_HEIGHT_INTERPOLATED)
        if height.method is HeightMethod.NEIGHBOR_LRF_NEAREST:
            warnings += [FootprintWarning.NEIGHBOR_HEIGHT_NEAREST,
                         FootprintWarning.LOW_CONFIDENCE_HEIGHT]
        if geom is None:
            geom = self._geometry(pose, camera, height.height_m, x0, y0,
                                  context.max_range_factor)

        if camera.distortion is None and meta.dewarp_flag != 1:
            warnings.append(FootprintWarning.LENS_DISTORTION_IGNORED)
        if camera.principal_point_source == "IMAGE_CENTER":
            warnings.append(FootprintWarning.PRINCIPAL_POINT_ASSUMED)
        if pose.off_nadir_deg > OBLIQUE_THRESHOLD_DEG:
            warnings.append(FootprintWarning.OBLIQUE_VIEW)
        if meta.rtk_flag is not None and meta.rtk_flag != 50:
            warnings.append(FootprintWarning.RTK_NOT_FIXED)   # absolute position only

        confidence = height.confidence
        for w in warnings:
            confidence *= GEOMETRY_FACTORS.get(w, 1.0)
        lrf_like = height.quantity in (HeightQuantity.CAMERA_TO_GROUND_VERTICAL_HEIGHT,
                                       HeightQuantity.CAMERA_TO_LRF_TARGET_VERTICAL_HEIGHT)
        dist_model = camera.distortion.model.value if camera.distortion else "PINHOLE"

        return PhotoFootprint(
            photo_id=meta.photo_id,
            geometry=geom.polygon,
            crs=CRS(context.target_crs).to_string(),
            method=(FootprintMethod.PLANAR_LRF_ESTIMATED if lrf_like
                    else FootprintMethod.PLANAR_ESTIMATED),
            confidence=round(confidence, 3),
            warnings=warnings,
            camera_xy=(x0, y0),
            principal_ground_xy=geom.principal,
            height_m=height.height_m,
            height_strategy=height.method.value,
            height_source=height.quantity.value,
            height_confidence=height.confidence,
            lrf_quality=lrf_q,
            corner_xy=geom.corners,
            lens_model=dist_model,
            lens_mode=camera.lens_mode.value,
            calibration_source=(camera.distortion.source if camera.distortion
                                else camera.focal_source),
            distortion_model=dist_model,
            projector=self.name,
            provenance={
                "height_strategy_requested": strategy.value,
                "height_method": height.method.value,
                "height_quantity": height.quantity.value,
                "height_neighbours": height.neighbours,
                "altitude_sources": [s.value for s in height.sources],
                "vertical_reference": height.vertical_reference.value,
                "ground_elevation": height.ground_elevation,
                "yaw_source": meta.sources.get("gimbal_yaw"),
                "pitch_source": meta.sources.get("gimbal_pitch"),
                "roll_source": meta.sources.get("gimbal_roll"),
                "focal_source": camera.focal_source,
                "principal_point_source": camera.principal_point_source,
                "lens_mode": camera.lens_mode.value,
                "focal_px": camera.focal_px,
                "yaw_true_deg": meta.gimbal_yaw,
                "grid_offset_deg": pose.grid_offset_deg,
                "yaw_grid_deg": pose.yaw_grid_deg,
                "pitch_deg": pose.pitch_deg,
                "roll_deg": pose.roll_deg,
            },
        )

    def _try_lrf(self, meta, camera, pose, x0, y0, context, to_grid
                 ) -> tuple[Optional[ProjectionHeight], Optional[_Geometry], LRFQualityResult]:
        """LRF ray height + geometry + full quality result (incl. the footprint check)."""
        q = assess_lrf(meta, None, None, context.lrf_thresholds)
        if q.status in (LRFQualityStatus.MISSING, LRFQualityStatus.INVALID):
            return None, None, q
        height = lrf_ray_height(meta, q)
        if height is None or height.height_m <= 0:
            q.status = LRFQualityStatus.INVALID
            q.reasons.append("laser height not computable")
            return None, None, q
        try:
            geom = self._geometry(pose, camera, height.height_m, x0, y0,
                                  context.max_range_factor)
        except FootprintUnavailableError as exc:
            q.status = LRFQualityStatus.INVALID
            q.reasons.append(str(exc))
            return None, None, q
        target_xy = to_grid.transform(meta.lrf_target_lon, meta.lrf_target_lat)
        q = assess_lrf(meta, geom.polygon, target_xy, context.lrf_thresholds)
        return height, geom, q
