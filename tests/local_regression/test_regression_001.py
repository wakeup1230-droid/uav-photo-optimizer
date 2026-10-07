"""
Regression Baseline 001 as a pytest (local only, excluded by default).

    pytest -m local_regression
"""

import sys
from pathlib import Path

import pytest

from uav_photo_optimizer.paths import DEFAULT_PATHS

pytestmark = [
    pytest.mark.local_regression,
    pytest.mark.skipif(not (DEFAULT_PATHS.shp_dir.is_dir() and DEFAULT_PATHS.photo_dir.is_dir()
                            and (Path(__file__).parent / "baseline_001.json").is_file()),
                       reason="local input data / baseline not present"),
]

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))


def test_regression_001():
    from run_regression_001 import EXPECTED, run_regression
    ok, info = run_regression()
    assert info["actual"] == EXPECTED, info
    assert not info["missing"] and not info["extra"], info
    assert ok
