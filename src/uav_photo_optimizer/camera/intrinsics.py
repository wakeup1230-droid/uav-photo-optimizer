"""
Resolve the pinhole intrinsics of one photo: metadata first, verified profile second.

Focal length (pixels), in priority order:
  1. XMP-drone-dji:CalibratedFocalLength (px)                         METADATA_CALIBRATED
  2. ExifIFD:FocalLength (mm) × width_px / profile.sensor_width_mm   METADATA_FOCAL_MM_PROFILE_SENSOR
  3. profile.horizontal_fov_deg                                       PROFILE_FOV
  else → None (unknown; the photo cannot be projected)

FocalLengthIn35mmFormat is deliberately NOT used: it is a 35 mm-equivalent (diagonal-based)
value, not the physical focal length.

Principal point: XMP CalibratedOpticalCenterX/Y when inside the image (origin top-left,
as observed on M4E: 2640/1978 = exact centre of 5280x3956), else image centre.

Lens mode (``LensMode``), when the image is not already undistorted (DewarpFlag != 1):
  AUTO          DewarpDataK6 (K6_RATIONAL) → DewarpData (BROWN5) → PINHOLE
                (the footprint then carries LENS_DISTORTION_IGNORED)
  K6_RATIONAL   DewarpDataK6 only;  BROWN5  DewarpData only  (both → PINHOLE when absent)
  PINHOLE       CalibratedFocalLength + optical centre, no distortion
"""

from __future__ import annotations

import math
from typing import Optional

from ..metadata.models import PhotoMetadata
from .lens import parse_dewarp
from .models import CameraIntrinsics, IntrinsicsSource, LensMode
from .profiles import DEFAULT_REGISTRY, CameraProfileRegistry


def resolve_intrinsics(meta: PhotoMetadata,
                       registry: CameraProfileRegistry = DEFAULT_REGISTRY,
                       lens_mode: LensMode = LensMode.AUTO) -> Optional[CameraIntrinsics]:
    profile = registry.get(meta.make, meta.model, meta.image_source)
    width = meta.image_width or (profile.native_width_px if profile else None)
    height = meta.image_height or (profile.native_height_px if profile else None)
    if not width or not height:
        return None
    size_note = "" if meta.image_width else " (size from profile)"
    profile_name = f"{profile.make} {profile.model} {profile.camera_name}" if profile else None

    if lens_mode is not LensMode.PINHOLE and meta.dewarp_flag != 1:
        cxm, cym = meta.calibrated_optical_center_x, meta.calibrated_optical_center_y
        inside = cxm is not None and cym is not None and 0 < cxm < width and 0 < cym < height
        order = {LensMode.K6_RATIONAL: ["dewarp_data_k6"],
                 LensMode.BROWN5: ["dewarp_data"]}.get(lens_mode,
                                                       ["dewarp_data_k6", "dewarp_data"])
        lens = None
        for fld in order:
            lens = parse_dewarp(getattr(meta, fld), width, height,
                                source=meta.sources.get(fld, fld),
                                center_x=cxm if inside else None,
                                center_y=cym if inside else None)
            if lens is not None:
                break
        if lens is not None:
            return CameraIntrinsics(
                width_px=width, height_px=height, focal_px=lens.fx, focal_y_px=lens.fy,
                cx_px=lens.cx, cy_px=lens.cy,
                focal_source=f"{IntrinsicsSource.METADATA_DEWARP.value} <- {lens.source}"
                             + size_note,
                principal_point_source=f"METADATA_DEWARP <- {lens.source} (centre offset)",
                profile=profile_name, distortion=lens,
                lens_mode=(LensMode.K6_RATIONAL if lens.model.value == "K6_RATIONAL"
                           else LensMode.BROWN5))

    focal_px: Optional[float] = None
    focal_source = ""
    if meta.calibrated_focal_length and meta.calibrated_focal_length > 0:
        focal_px = meta.calibrated_focal_length
        focal_source = (f"{IntrinsicsSource.METADATA_CALIBRATED.value}"
                        f" <- {meta.sources.get('calibrated_focal_length')}")
    elif meta.focal_length_mm and profile and profile.sensor_width_mm:
        focal_px = meta.focal_length_mm * width / profile.sensor_width_mm
        focal_source = (f"{IntrinsicsSource.METADATA_FOCAL_MM_PROFILE_SENSOR.value}"
                        f" <- {meta.sources.get('focal_length_mm')} + profile sensor width")
    elif profile and profile.horizontal_fov_deg:
        focal_px = width / 2 / math.tan(math.radians(profile.horizontal_fov_deg) / 2)
        focal_source = f"{IntrinsicsSource.PROFILE_FOV.value} <- profile {profile.camera_name}"
    if focal_px is None:
        return None

    cx, cy = meta.calibrated_optical_center_x, meta.calibrated_optical_center_y
    if cx is not None and cy is not None and 0 < cx < width and 0 < cy < height:
        pp_source = (f"METADATA <- {meta.sources.get('calibrated_optical_center_x')}, "
                     f"{meta.sources.get('calibrated_optical_center_y')}")
    else:
        cx, cy = width / 2, height / 2
        pp_source = IntrinsicsSource.IMAGE_CENTER.value

    return CameraIntrinsics(width_px=width, height_px=height, focal_px=focal_px,
                            cx_px=cx, cy_px=cy, focal_source=focal_source + size_note,
                            principal_point_source=pp_source, profile=profile_name,
                            lens_mode=LensMode.PINHOLE)
