from pathlib import Path

import numpy as np
import pytest

from uav_photo_optimizer.camera.intrinsics import resolve_intrinsics
from uav_photo_optimizer.camera.lens import DistortionModel, parse_dewarp
from uav_photo_optimizer.camera.models import CameraIntrinsics, LensMode
from uav_photo_optimizer.footprint.base import ProjectionContext
from uav_photo_optimizer.footprint.models import FootprintWarning
from uav_photo_optimizer.footprint.planar import PlanarProjector
from uav_photo_optimizer.metadata.altitude import HeightStrategy
from uav_photo_optimizer.metadata.models import PhotoMetadata

# synthetic values in the DJI DewarpData format (date;fx,fy,cx_off,cy_off,coefficients)
D5 = "2024-01-01;3700.0,3700.0,25.0,-20.0,-0.11,0.004,0.00005,-0.00018,-0.019"
D6 = ("2024-01-01;3700.0,3700.0,27.0,-21.0,-6.37,15.21,0.00007,-0.0002,-4.761,"
      "-6.284,14.703,-3.624")
W, H = 5280, 3956
CORNERS = np.array([[0, 0], [W, 0], [W, H], [0, H]], float)


def test_parse_brown5():
    lens = parse_dewarp(D5, W, H)
    assert lens.model is DistortionModel.BROWN5 and lens.calibration_date == "2024-01-01"
    assert lens.fx == pytest.approx(3700.0)
    assert lens.cx == pytest.approx(2640 + 25.0)      # offset from the photo centre
    assert lens.cy == pytest.approx(1978 - 20.0)
    assert lens.k1 == pytest.approx(-0.11) and lens.k3 == pytest.approx(-0.019)


def test_parse_uses_calibrated_center():
    lens = parse_dewarp(D5, W, H, center_x=2600, center_y=2000)
    assert (lens.cx, lens.cy) == pytest.approx((2625.0, 1980.0))


def test_parse_rational8():
    lens = parse_dewarp(D6, W, H)
    assert lens.model is DistortionModel.K6_RATIONAL
    assert (lens.k4, lens.k5, lens.k6) == pytest.approx((-6.284, 14.703, -3.624))


@pytest.mark.parametrize("bad", [None, "", "2025;1,2,3", "x;a,b,c,d,e,f,g,h,i",
                                 "2025;0,0,0,0,0,0,0,0,0"])
def test_parse_malformed(bad):
    assert parse_dewarp(bad, W, H) is None


def test_roundtrip_and_identity():
    lens = parse_dewarp(D5, W, H)
    grid = np.array([[x, y] for x in np.linspace(0, W, 9) for y in np.linspace(0, H, 7)])
    assert lens.roundtrip_error_px(grid) < 1e-6
    zero = lens.model_copy(update={"k1": 0, "k2": 0, "p1": 0, "p2": 0, "k3": 0})
    np.testing.assert_allclose(zero.undistort_pixels(grid), grid, atol=1e-9)


def test_barrel_moves_corners_outward():
    lens = parse_dewarp(D5, W, H)
    und = lens.undistort_pixels(CORNERS)
    r_rec = np.hypot(CORNERS[:, 0] - lens.cx, CORNERS[:, 1] - lens.cy)
    r_und = np.hypot(und[:, 0] - lens.cx, und[:, 1] - lens.cy)
    assert (r_und > r_rec * 1.08).all()          # k1 < 0 → wider field at the corners


def test_k5_and_k6_agree():
    a, b = parse_dewarp(D5, W, H), parse_dewarp(D6, W, H)
    grid = np.array([[x, y] for x in np.linspace(0, W, 23) for y in np.linspace(0, H, 17)])

    def angles(lens):
        u = lens.undistort_pixels(grid)
        return np.degrees(np.arctan(np.hypot((u[:, 0] - lens.cx) / lens.fx,
                                             (u[:, 1] - lens.cy) / lens.fy)))
    assert np.abs(angles(a) - angles(b)).max() < 0.2


def _meta(**kw):
    return PhotoMetadata(photo_id="p", filename="p", path=Path("p"), image_width=W,
                         image_height=H, calibrated_focal_length=3725.151611,
                         calibrated_optical_center_x=2640.0, calibrated_optical_center_y=1978.0,
                         dewarp_data=D5, **kw)


def test_intrinsics_lens_modes_k6_first():
    both = _meta(dewarp_flag=0, dewarp_data_k6=D6)
    cam = resolve_intrinsics(both)                                   # AUTO
    assert cam.lens_mode is LensMode.K6_RATIONAL
    assert cam.distortion.model is DistortionModel.K6_RATIONAL
    assert resolve_intrinsics(both, lens_mode=LensMode.BROWN5).lens_mode is LensMode.BROWN5
    assert resolve_intrinsics(_meta(dewarp_flag=0)).lens_mode is LensMode.BROWN5   # no K6
    assert resolve_intrinsics(both, lens_mode=LensMode.PINHOLE).lens_mode is LensMode.PINHOLE
    assert resolve_intrinsics(_meta(dewarp_flag=1)).lens_mode is LensMode.PINHOLE   # dewarped
    no = _meta(dewarp_flag=0).model_copy(update={"dewarp_data": None})
    assert resolve_intrinsics(no).lens_mode is LensMode.PINHOLE                     # fallback


def _fp(cam):
    m = PhotoMetadata(photo_id="p", filename="p", path=Path("p"), latitude=24.0, longitude=121.0,
                      relative_altitude=100.0, gimbal_yaw=0.0, gimbal_pitch=-90.0,
                      gimbal_roll=0.0, dewarp_flag=0, rtk_flag=50)
    ctx = ProjectionContext(target_crs="EPSG:3826",
                            height_strategy=HeightStrategy.TAKEOFF_RELATIVE)
    return PlanarProjector().project(m, cam, ctx)


def test_zero_distortion_footprint_equals_pinhole():
    lens = parse_dewarp(D5, W, H).model_copy(update={"k1": 0, "k2": 0, "p1": 0, "p2": 0,
                                                     "k3": 0})
    pin = CameraIntrinsics(width_px=W, height_px=H, focal_px=lens.fx, cx_px=lens.cx,
                           cy_px=lens.cy, focal_source="t", principal_point_source="t")
    dist = pin.model_copy(update={"distortion": lens, "lens_mode": LensMode.BROWN5})
    a, b = _fp(pin), _fp(dist)
    assert len(b.geometry.exterior.coords) == 4 * 16 + 1 and len(b.corner_xy) == 4
    np.testing.assert_allclose(np.array(a.corner_xy), np.array(b.corner_xy), atol=1e-6)
    assert a.geometry.area == pytest.approx(b.geometry.area, rel=1e-9)
    assert FootprintWarning.LENS_DISTORTION_IGNORED in a.warnings
    assert FootprintWarning.LENS_DISTORTION_IGNORED not in b.warnings
    assert b.lens_model == "BROWN5" and b.distortion_model == "BROWN5"
    assert b.lens_mode == "BROWN5" and a.lens_mode == "PINHOLE"


def test_distortion_enlarges_nadir_footprint():
    cam_p = resolve_intrinsics(_meta(dewarp_flag=0), lens_mode=LensMode.PINHOLE)
    cam_d = resolve_intrinsics(_meta(dewarp_flag=0))
    ratio = _fp(cam_d).geometry.area / _fp(cam_p).geometry.area
    assert ratio == pytest.approx(1.1854, abs=0.005)        # barrel distortion: about +18.5 %


def test_k6_footprint_provenance_and_pinhole_warning():
    cam = resolve_intrinsics(_meta(dewarp_flag=0, dewarp_data_k6=D6,
                                   sources={"dewarp_data_k6": "XMP-drone-dji:DewarpDataK6"}))
    fp = _fp(cam)
    assert fp.distortion_model == "K6_RATIONAL" and fp.lens_mode == "K6_RATIONAL"
    assert fp.calibration_source == "XMP-drone-dji:DewarpDataK6"
    assert FootprintWarning.LENS_DISTORTION_IGNORED not in fp.warnings
    pin = _fp(resolve_intrinsics(_meta(dewarp_flag=0).model_copy(update={"dewarp_data": None})))
    assert pin.distortion_model == "PINHOLE"
    assert FootprintWarning.LENS_DISTORTION_IGNORED in pin.warnings
