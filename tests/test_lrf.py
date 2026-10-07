"""LRF gate v2 + LRF_RAY_VERTICAL height (synthetic, analytically consistent geometry)."""

import math
from pathlib import Path

import pytest
from pyproj import Geod

from uav_photo_optimizer.camera.models import CameraIntrinsics
from uav_photo_optimizer.footprint.base import (FootprintUnavailableError, HeightUnresolvedError,
                                                ProjectionContext)
from uav_photo_optimizer.footprint.lrf import LRFQualityStatus, LRFThresholds, assess_lrf
from uav_photo_optimizer.footprint.models import FootprintMethod, FootprintWarning
from uav_photo_optimizer.footprint.planar import PlanarProjector
from uav_photo_optimizer.metadata.altitude import HeightStrategy
from uav_photo_optimizer.metadata.models import PhotoMetadata

GEOD = Geod(ellps="WGS84")
LON, LAT = 121.0, 24.0
CAM = CameraIntrinsics(width_px=4000, height_px=3000, focal_px=2000, cx_px=2000, cy_px=1500,
                       focal_source="test", principal_point_source="test")
AUTO = ProjectionContext(target_crs="EPSG:3826")


def meta(*, h=120.0, pitch=-90.0, yaw=0.0, roll=0.0, offset_m=None, offset_az=None,
         distance=None, abs_alt=300.0, target_alt=None, rel=450.0, rtk=50, lrf=True,
         status="Normal") -> PhotoMetadata:
    """Camera at height h above the laser ground point, laser along the optical axis."""
    depression = math.radians(-pitch)
    horiz = h / math.tan(depression) if abs(pitch) < 89.999 else 0.0
    if offset_m is None:
        offset_m, offset_az = horiz, yaw
    kw = {}
    if lrf:
        tlon, tlat, _ = GEOD.fwd(LON, LAT, offset_az, offset_m) if offset_m else (LON, LAT, 0)
        kw = dict(lrf_status=status, lrf_target_lat=tlat, lrf_target_lon=tlon,
                  lrf_target_abs_alt=abs_alt - h if target_alt is None else target_alt,
                  lrf_target_distance=math.hypot(h, horiz) if distance is None else distance)
    return PhotoMetadata(photo_id="p", filename="p", path=Path("p"), latitude=LAT, longitude=LON,
                         absolute_altitude=abs_alt, relative_altitude=rel, gimbal_yaw=yaw,
                         gimbal_pitch=pitch, gimbal_roll=roll, dewarp_flag=1, rtk_flag=rtk, **kw)


def test_valid_lrf_ray_height():
    fp = PlanarProjector().project(meta(), CAM, AUTO)
    q = fp.lrf_quality
    assert q.status is LRFQualityStatus.VALID
    assert fp.height_m == pytest.approx(120.0)
    assert fp.method is FootprintMethod.PLANAR_LRF_ESTIMATED
    assert fp.height_strategy == "LRF_RAY_VERTICAL"
    assert fp.height_source == "camera_to_ground_vertical_height"
    assert fp.height_confidence == 0.9
    assert q.laser_vertical_height_m == pytest.approx(120.0)
    assert q.ray_down_component == pytest.approx(1.0)
    assert q.beam_angle_deg == pytest.approx(0, abs=1e-6)
    assert FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR in fp.warnings
    assert FootprintWarning.ALTITUDE_TAKEOFF_RELATIVE not in fp.warnings


def test_oblique_ray_height_uses_pose_adapter():
    # pitch -60: laser 138.56 m along the axis, vertical component sin 60°
    fp = PlanarProjector().project(meta(pitch=-60, yaw=30), CAM, AUTO)
    assert fp.lrf_quality.status is LRFQualityStatus.VALID
    assert fp.height_m == pytest.approx(120.0, abs=1e-6)
    assert fp.lrf_quality.beam_angle_deg < 0.05
    # roll 180 (Euler-equivalent backward look) gives the same vertical component
    fp2 = PlanarProjector().project(meta(pitch=-60, yaw=30, roll=180), CAM, AUTO)
    assert fp2.height_m == pytest.approx(120.0, abs=1e-6)


def test_height_independent_of_absolute_altitude_bias():
    """Non-fixed RTK altitude bias changes only the QA metric, not the height or the status."""
    m = meta(target_alt=300 - 120 + 15, rtk=34)      # altitudes inconsistent by 15 m
    fp = PlanarProjector().project(m, CAM, AUTO)
    assert fp.lrf_quality.status is LRFQualityStatus.VALID
    assert fp.height_m == pytest.approx(120.0)
    assert fp.lrf_quality.height_difference_m == pytest.approx(-15.0)
    assert fp.lrf_quality.absolute_position_confidence == 0.8


def test_beam_misaligned_is_unresolved_in_auto():
    # nadir camera, laser target 50 m to the side → ~24.6° off axis (laser geometry)
    m = meta(offset_m=50, offset_az=90, distance=130.0)
    with pytest.raises(HeightUnresolvedError) as exc:
        PlanarProjector().project(m, CAM, AUTO)
    assert exc.value.lrf_quality.status is LRFQualityStatus.INVALID
    assert exc.value.lrf_quality.beam_angle_deg == pytest.approx(math.degrees(
        math.asin(50 / 130)), abs=0.05)


def test_suspect_beam_explicit_strategy_only():
    m = meta(offset_m=130 * math.sin(math.radians(7)), offset_az=90, distance=130.0)
    with pytest.raises(HeightUnresolvedError):                          # AUTO: VALID only
        PlanarProjector().project(m, CAM, AUTO)
    ctx = ProjectionContext(target_crs="EPSG:3826", height_strategy=HeightStrategy.LRF_RAY_VERTICAL)
    fp = PlanarProjector().project(m, CAM, ctx)
    assert fp.lrf_quality.status is LRFQualityStatus.SUSPECT
    assert FootprintWarning.LRF_SUSPECT in fp.warnings and fp.height_confidence == 0.6


def test_auto_never_uses_relative_altitude():
    for m in (meta(lrf=False), meta(status="Error"), meta(distance=5000.0)):
        with pytest.raises(HeightUnresolvedError, match="HEIGHT_UNRESOLVED"):
            PlanarProjector().project(m, CAM, AUTO)


def test_distance_and_pose_validity():
    assert assess_lrf(meta(distance=5000.0), None, None).status is LRFQualityStatus.INVALID
    assert assess_lrf(meta(status="Error"), None, None).status is LRFQualityStatus.INVALID
    assert assess_lrf(meta(lrf=False), None, None).status is LRFQualityStatus.MISSING
    q = assess_lrf(meta(pitch=-5), None, None)                    # nearly horizontal ray
    assert q.status is LRFQualityStatus.INVALID
    q = assess_lrf(meta(roll=40), None, None)                     # odd roll → SUSPECT
    assert q.status in (LRFQualityStatus.SUSPECT, LRFQualityStatus.INVALID)


def test_explicit_lrf_local_plane_is_qa_method():
    ctx = ProjectionContext(target_crs="EPSG:3826", height_strategy=HeightStrategy.LRF_LOCAL_PLANE)
    fp = PlanarProjector().project(meta(target_alt=300 - 110), CAM, ctx)
    assert fp.height_m == pytest.approx(110.0)
    assert fp.height_strategy == "LRF_LOCAL_PLANE"
    assert FootprintWarning.ALTITUDE_DIFFERENCE_HEIGHT in fp.warnings


def test_takeoff_relative_only_when_explicit():
    ctx = ProjectionContext(target_crs="EPSG:3826", height_strategy=HeightStrategy.TAKEOFF_RELATIVE)
    fp = PlanarProjector().project(meta(), CAM, ctx)
    assert fp.height_m == 450 and fp.height_confidence == 0.3
    assert {FootprintWarning.ALTITUDE_TAKEOFF_RELATIVE,
            FootprintWarning.LOW_CONFIDENCE_HEIGHT} <= set(fp.warnings)


def test_explicit_lrf_ray_rejects_invalid():
    ctx = ProjectionContext(target_crs="EPSG:3826", height_strategy=HeightStrategy.LRF_RAY_VERTICAL)
    with pytest.raises(FootprintUnavailableError, match="INVALID"):
        PlanarProjector().project(meta(offset_m=60, offset_az=90, distance=130.0), CAM, ctx)


def test_thresholds_configurable():
    m = meta(offset_m=130 * math.sin(math.radians(3)), offset_az=90, distance=130.0)
    assert assess_lrf(m, None, None).status is LRFQualityStatus.VALID
    strict = LRFThresholds(beam_valid_deg=2.0)
    assert assess_lrf(m, None, None, strict).status is LRFQualityStatus.SUSPECT
