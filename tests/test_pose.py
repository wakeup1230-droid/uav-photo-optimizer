import math

import numpy as np
import pytest

from uav_photo_optimizer.footprint.pose import DJIPose, grid_north_offset, to_grid_pose


def test_grid_offset_zero_on_central_meridian():
    assert grid_north_offset(121.0, 24.0, "EPSG:3826") == pytest.approx(0, abs=1e-9)


def test_grid_offset_matches_meridian_convergence():
    # TM east of the central meridian: true north lies west of grid north (≈ -Δλ·sinφ)
    expected = -0.55 * math.sin(math.radians(24.87))
    assert grid_north_offset(121.55, 24.87, "EPSG:3826") == pytest.approx(expected, abs=1e-3)


def test_geographic_crs_no_offset():
    assert grid_north_offset(121.55, 24.87, "EPSG:4326") == 0


def test_cameratransform_mapping():
    gp = to_grid_pose(DJIPose(28.8, -60.0, 0.0), 121.0, 24.0, "EPSG:3826")
    assert gp.cameratransform_orientation() == pytest.approx(
        {"heading_deg": 28.8, "tilt_deg": 30.0, "roll_deg": -0.0})
    assert gp.off_nadir_deg == pytest.approx(30)


def test_rotation_nadir_axes():
    """DJI (0, -90, 0): optical axis down, image-top (−down_body) → north, image-right → east."""
    r = to_grid_pose(DJIPose(0, -90, 0), 121.0, 24.0, "EPSG:3826").rotation_ned()
    np.testing.assert_allclose(r @ [1, 0, 0], [0, 0, 1], atol=1e-12)     # forward → down
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 1, 0], atol=1e-12)     # right → east
    np.testing.assert_allclose(r @ [0, 0, -1], [1, 0, 0], atol=1e-12)    # image up → north
