"""Phase 8A visual validation: OpenCV validator on synthetic textures + guard logic."""

import subprocess
import sys

import numpy as np
import pytest

from uav_photo_optimizer.visual.base import PhotoInput, ValidationContext
from uav_photo_optimizer.visual.models import (VisualMatchResult, VisualStatus,
                                               VisualValidationConfig)

cv2 = pytest.importorskip("cv2")
from uav_photo_optimizer.visual.opencv_validator import OpenCVVisualValidator  # noqa: E402


def texture(w=3000, h=1600, seed=1):
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), np.uint8)
    for _ in range(9000):                          # random blobs → rich, unique texture
        x, y = rng.integers(0, w), rng.integers(0, h)
        cv2.circle(img, (int(x), int(y)), int(rng.integers(3, 25)), int(rng.integers(30, 255)), -1)
    return cv2.GaussianBlur(img, (3, 3), 0)


@pytest.fixture(scope="module")
def crops(tmp_path_factory):
    d = tmp_path_factory.mktemp("vis")
    world = texture()
    paths = {}
    for name, x0 in (("a", 0), ("b", 300), ("far", 1700)):     # 1200 px wide crops
        p = d / f"{name}.jpg"
        cv2.imencode(".jpg", world[200:1100, x0:x0 + 1200])[1].tofile(str(p))
        paths[name] = p
    bad = d / "broken.jpg"
    bad.write_bytes(b"not a jpeg")
    paths["broken"] = bad
    return paths


CFG = VisualValidationConfig(max_long_edge=1024)


def pair(crops, a, b):
    v = OpenCVVisualValidator()
    return v.validate_pair(PhotoInput(a, crops[a]), PhotoInput(b, crops[b]), ValidationContext(CFG))


def test_overlapping_pass(crops):
    r = pair(crops, "a", "b")                 # 75 % overlap, pure translation
    assert r.status is VisualStatus.PASS
    assert r.fundamental_inliers >= 100 and r.fundamental_inlier_ratio >= 0.75
    assert min(r.grid_cells_a, r.grid_cells_b) >= 7
    assert r.essential_inliers is None        # no intrinsics → Fundamental only


def test_disjoint_fail(crops):
    r = pair(crops, "a", "far")
    assert r.status is VisualStatus.FAIL


def test_unreadable_unresolved(crops):
    r = pair(crops, "a", "broken")
    assert r.status is VisualStatus.UNRESOLVED and r.warnings


def test_result_json_roundtrip(crops):
    r = pair(crops, "a", "b")
    assert VisualMatchResult.model_validate_json(r.model_dump_json()) == r


def test_detectors_configurable(crops):
    v = OpenCVVisualValidator()
    r = v.validate_pair(PhotoInput("a", crops["a"]), PhotoInput("b", crops["b"]),
                        ValidationContext(VisualValidationConfig(detector="ORB", matcher="BF")))
    assert r.detector == "ORB" and r.ratio_test_matches > 0


def test_core_does_not_import_cv2():
    code = ("import sys, uav_photo_optimizer, uav_photo_optimizer.optimizer.planner, "
            "uav_photo_optimizer.optimizer.validated, uav_photo_optimizer.visual; "
            "print('cv2' in sys.modules)")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert out.stdout.strip() == "False"
