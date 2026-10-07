"""
Synthetic geometry tests for the Estimated Ground Footprint (answers derived analytically).

Camera: 4000 x 3000 px, f = 2000 px  → HFOV 90°, VFOV 2·atan(0.75) = 73.74°.
Location on the TWD97 central meridian (121°E) so grid north == true north.
x = grid east, y = grid north (EPSG:3826).
"""

import math
from pathlib import Path

import numpy as np
import pytest
from pyproj import Transformer

from uav_photo_optimizer.camera.models import CameraIntrinsics
from uav_photo_optimizer.footprint.base import (FootprintUnavailableError, ProjectionContext)
from uav_photo_optimizer.footprint.models import FootprintMethod, FootprintWarning
from uav_photo_optimizer.footprint.planar import PlanarProjector
from uav_photo_optimizer.metadata.altitude import HeightStrategy
from uav_photo_optimizer.metadata.models import PhotoMetadata

ct = pytest.importorskip("cameratransform")
from uav_photo_optimizer.footprint.cameratransform_adapter import CameraTransformProjector  # noqa: E402

CRS = "EPSG:3826"
LON, LAT = 121.0, 24.0
X0, Y0 = Transformer.from_crs("EPSG:4326", CRS, always_xy=True).transform(LON, LAT)
CAM = CameraIntrinsics(width_px=4000, height_px=3000, focal_px=2000, cx_px=2000, cy_px=1500,
                       focal_source="test", principal_point_source="test")
CTX = ProjectionContext(target_crs=CRS, height_strategy=HeightStrategy.TAKEOFF_RELATIVE)
PROJECTORS = [PlanarProjector(), CameraTransformProjector()]
IDS = ["planar", "cameratransform"]


def meta(yaw=0.0, pitch=-90.0, roll=0.0, height=100.0, **kw) -> PhotoMetadata:
    return PhotoMetadata(photo_id="p", filename="p.jpg", path=Path("p.jpg"),
                         latitude=LAT, longitude=LON, relative_altitude=height,
                         gimbal_yaw=yaw, gimbal_pitch=pitch, gimbal_roll=roll,
                         dewarp_flag=1, rtk_flag=50, **kw)


def corners(fp) -> np.ndarray:
    """TL, TR, BR, BL relative to the camera, in metres."""
    return np.array(fp.geometry.exterior.coords[:4]) - np.array([X0, Y0])


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_fp01_nadir_center_width_height(proj):
    fp = proj.project(meta(), CAM, CTX)
    assert fp.method is FootprintMethod.PLANAR_ESTIMATED   # relative height (TAKEOFF_RELATIVE)
    c = corners(fp)
    # top of image = heading (north), right of image = east → no mirroring
    np.testing.assert_allclose(c, [[-100, 75], [100, 75], [100, -75], [-100, -75]], atol=1e-6)
    assert fp.geometry.centroid.x == pytest.approx(X0, abs=1e-6)
    assert fp.geometry.centroid.y == pytest.approx(Y0, abs=1e-6)
    np.testing.assert_allclose(fp.principal_ground_xy, (X0, Y0), atol=1e-6)
    assert fp.geometry.area == pytest.approx(200 * 150)


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_fp02_yaw_90_rotates_footprint(proj):
    c = corners(proj.project(meta(yaw=90), CAM, CTX))
    # image top → east, image left → north
    np.testing.assert_allclose(c, [[75, 100], [75, -100], [-75, -100], [-75, 100]], atol=1e-6)
    centroid = c.mean(axis=0)
    np.testing.assert_allclose(centroid, (0, 0), atol=1e-6)     # position unchanged


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
@pytest.mark.parametrize("yaw", [0, 37.5, 90, 180, -120])
def test_fp02_yaw_rotation_general(proj, yaw):
    base = corners(proj.project(meta(yaw=0), CAM, CTX))
    rot = corners(proj.project(meta(yaw=yaw), CAM, CTX))
    a = math.radians(yaw)                       # clockwise seen from above
    r = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
    np.testing.assert_allclose(rot, base @ r.T, atol=1e-6)


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_fp03_dji_pitch_minus_90_is_nadir(proj):
    fp = proj.project(meta(pitch=-90), CAM, CTX)
    np.testing.assert_allclose(fp.principal_ground_xy, (X0, Y0), atol=1e-6)
    assert FootprintWarning.OBLIQUE_VIEW not in fp.warnings
    # Euler-equivalent representation seen in real M4E data: (yaw+180, -90, roll 180)
    alt = proj.project(meta(yaw=180, pitch=-90, roll=180), CAM, CTX)
    np.testing.assert_allclose(corners(alt), corners(fp), atol=1e-6)


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_fp04_oblique_pitch_minus_60(proj):
    fp = proj.project(meta(pitch=-60), CAM, CTX)
    h = 100
    half_v = math.atan(1500 / 2000)                       # 36.87°
    # boresight 30° off nadir toward heading (north)
    np.testing.assert_allclose(fp.principal_ground_xy,
                               (X0, Y0 + h * math.tan(math.radians(30))), atol=1e-6)
    tl, tr, br, bl = corners(fp)
    far_y = h * math.tan(math.radians(30) + half_v)        # top edge (far)
    near_y = h * math.tan(math.radians(30) - half_v)       # bottom edge (slightly behind)
    assert tl[1] == pytest.approx(far_y) and tr[1] == pytest.approx(far_y)
    assert bl[1] == pytest.approx(near_y) and br[1] == pytest.approx(near_y)
    assert tr[0] - tl[0] > br[0] - bl[0] > 0               # trapezoid: far edge wider
    assert fp.geometry.is_valid and fp.geometry.centroid.y > Y0   # shifted forward
    assert FootprintWarning.OBLIQUE_VIEW in fp.warnings


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_fp04_oblique_roll_180_same_polygon(proj):
    """With the principal point at the image centre, roll 180 only relabels corners."""
    a = proj.project(meta(yaw=30, pitch=-60, roll=0), CAM, CTX)
    b = proj.project(meta(yaw=30, pitch=-60, roll=180), CAM, CTX)
    assert a.geometry.equals_exact(b.geometry.normalize(), 1e-6) or \
        a.geometry.symmetric_difference(b.geometry).area < 1e-6


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
@pytest.mark.parametrize("pitch, yaw", [(-90, 0), (-60, 45)])
def test_fp05_altitude_scales_linearly(proj, pitch, yaw):
    c50 = corners(proj.project(meta(yaw=yaw, pitch=pitch, height=50), CAM, CTX))
    c100 = corners(proj.project(meta(yaw=yaw, pitch=pitch, height=100), CAM, CTX))
    np.testing.assert_allclose(c100, 2 * c50, atol=1e-6)


@pytest.mark.parametrize("pose", [(0, -90, 0), (28.8, -60, 0), (-61.7, -60, 180),
                                  (123, -75, 7), (-170, -55, -12)])
def test_cameratransform_matches_reference(pose):
    yaw, pitch, roll = pose
    m = meta(yaw=yaw, pitch=pitch, roll=roll, height=122.3)
    a = PROJECTORS[0].project(m, CAM, CTX)
    b = PROJECTORS[1].project(m, CAM, CTX)
    np.testing.assert_allclose(corners(a), corners(b), atol=1e-6)
    np.testing.assert_allclose(a.principal_ground_xy, b.principal_ground_xy, atol=1e-6)


@pytest.mark.parametrize("proj", PROJECTORS, ids=IDS)
def test_horizon_in_view_is_unavailable(proj):
    with pytest.raises(FootprintUnavailableError, match="horizon"):
        proj.project(meta(pitch=0), CAM, CTX)


# Altitude source handling ----------------------------------------------------------

def test_takeoff_relative_warnings():
    fp = PlanarProjector().project(meta(height=80), CAM, CTX)
    assert fp.height_m == 80
    assert FootprintWarning.ALTITUDE_TAKEOFF_RELATIVE in fp.warnings
    assert FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR in fp.warnings
    assert fp.provenance["altitude_sources"] == ["XMP-drone-dji:RelativeAltitude"]


def test_auto_without_lrf_is_unresolved():
    from uav_photo_optimizer.footprint.base import HeightUnresolvedError
    with pytest.raises(HeightUnresolvedError, match="HEIGHT_UNRESOLVED"):
        PlanarProjector().project(meta(height=90), CAM, ProjectionContext(target_crs=CRS))


def test_user_ground_elevation():
    m = meta(absolute_altitude=300.0)
    ctx = ProjectionContext(target_crs=CRS, height_strategy=HeightStrategy.USER_GROUND_ELEVATION,
                            user_ground_elevation=180.0)
    assert PlanarProjector().project(m, CAM, ctx).height_m == 120


def test_non_positive_height_unavailable():
    with pytest.raises(FootprintUnavailableError, match="non-positive"):
        PlanarProjector().project(meta(height=-5), CAM, CTX)


def test_missing_pose_unavailable():
    m = meta()
    m = m.model_copy(update={"gimbal_yaw": None})
    with pytest.raises(FootprintUnavailableError, match="pose"):
        PlanarProjector().project(m, CAM, CTX)


def test_confidence_and_provenance():
    m = meta(pitch=-60).model_copy(update={"dewarp_flag": 0, "rtk_flag": 34})
    fp = PlanarProjector().project(m, CAM, CTX)
    assert {FootprintWarning.LENS_DISTORTION_IGNORED, FootprintWarning.RTK_NOT_FIXED,
            FootprintWarning.OBLIQUE_VIEW} <= set(fp.warnings)
    assert 0 < fp.confidence < 0.5
    assert fp.provenance["focal_source"] == "test"
    # JSON round-trip (geometry serialized as GeoJSON)
    data = fp.model_dump(mode="json")
    assert data["geometry"]["type"] == "Polygon"
    assert type(fp).model_validate(data).geometry.equals(fp.geometry)
