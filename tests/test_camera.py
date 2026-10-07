from pathlib import Path

import pytest

from uav_photo_optimizer.camera.intrinsics import resolve_intrinsics
from uav_photo_optimizer.camera.models import CameraProfile
from uav_photo_optimizer.camera.profiles import (DEFAULT_REGISTRY, DJI_M4E_WIDE,
                                                 CameraProfileRegistry)
from uav_photo_optimizer.metadata.models import PhotoMetadata


def meta(**kw):
    return PhotoMetadata(photo_id="p", filename="p", path=Path("p"), **kw)


M4E = dict(make="DJI", model="M4E", image_source="WideCamera", image_width=5280,
           image_height=3956, focal_length_mm=12.29, focal_length_35mm=24.0)


def test_m4e_profile_has_no_guessed_values():
    p = DJI_M4E_WIDE
    assert (p.native_width_px, p.native_height_px) == (5280, 3956)
    assert p.sensor_width_mm is None and p.sensor_height_mm is None
    assert p.focal_length_mm is None                    # 24 mm is 35mm-equivalent, not physical
    assert p.horizontal_fov_deg is None and p.vertical_fov_deg is None
    assert p.verified and "enterprise.dji.com" in p.source


def test_registry_lookup_case_insensitive():
    assert DEFAULT_REGISTRY.get("dji", "m4e", "widecamera") is DJI_M4E_WIDE
    assert DEFAULT_REGISTRY.get("DJI", "M3E", "WideCamera") is None
    assert DEFAULT_REGISTRY.get(None, "M4E") is None


def test_metadata_calibrated_first():
    cam = resolve_intrinsics(meta(**M4E, calibrated_focal_length=3725.151611,
                                  calibrated_optical_center_x=2640.0,
                                  calibrated_optical_center_y=1978.0,
                                  sources={"calibrated_focal_length":
                                           "XMP-drone-dji:CalibratedFocalLength"}))
    assert cam.focal_px == 3725.151611
    assert cam.focal_source.startswith("METADATA_CALIBRATED")
    assert (cam.cx_px, cam.cy_px) == (2640.0, 1978.0)
    assert cam.principal_point_source.startswith("METADATA")
    assert cam.horizontal_fov_deg == pytest.approx(70.66, abs=0.01)
    assert cam.vertical_fov_deg == pytest.approx(55.94, abs=0.01)


def test_35mm_equivalent_never_used_as_focal():
    """Only FocalLength + 35mm-equivalent, no calibrated focal, no verified sensor → unknown."""
    assert resolve_intrinsics(meta(**M4E)) is None


def test_verified_profile_sensor_fallback():
    reg = CameraProfileRegistry([CameraProfile(
        make="ACME", model="X1", camera_name="main", sensor_width_mm=13.2, sensor_height_mm=8.8,
        native_width_px=5472, native_height_px=3648, source="test", verified=True)])
    cam = resolve_intrinsics(meta(make="ACME", model="X1", image_width=5472, image_height=3648,
                                  focal_length_mm=8.8), reg)
    assert cam.focal_px == pytest.approx(8.8 * 5472 / 13.2)
    assert cam.focal_source.startswith("METADATA_FOCAL_MM_PROFILE_SENSOR")
    assert cam.principal_point_source == "IMAGE_CENTER"


def test_profile_fov_fallback():
    reg = CameraProfileRegistry([CameraProfile(
        make="ACME", model="X2", camera_name="main", horizontal_fov_deg=90.0,
        source="test", verified=True)])
    cam = resolve_intrinsics(meta(make="ACME", model="X2", image_width=4000, image_height=3000),
                             reg)
    assert cam.focal_px == pytest.approx(2000)


def test_principal_point_outside_image_falls_back_to_centre():
    cam = resolve_intrinsics(meta(**M4E, calibrated_focal_length=3725.0,
                                  calibrated_optical_center_x=-5.0,
                                  calibrated_optical_center_y=1978.0))
    assert (cam.cx_px, cam.cy_px) == (2640.0, 1978.0)
    assert cam.principal_point_source == "IMAGE_CENTER"


def test_unknown_camera():
    assert resolve_intrinsics(meta(make="X", model="Y")) is None
