"""
Regression Baseline 001 (local only — needs the real, non-public input data).

    local test data + the single AOI SHP in input/shp + Buffer 100 m (GPS_POINT mode)

The baseline lives in tests/local_regression/baseline_001.json (git-ignored, never published):
``{"expected": <count>, "selected_photo_ids": [...]}``. The selected count and every photo_id
are compared. Without a baseline file the regression is skipped.

    python scripts/run_regression_001.py
Exit code 0 = PASS, 1 = FAIL.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from uav_photo_optimizer import RunConfig, UAVPhotoOptimizer
from uav_photo_optimizer.paths import find_project_root

BUFFER_M = 100.0
BASELINE_FILE = find_project_root() / "tests" / "local_regression" / "baseline_001.json"


def _baseline() -> dict:
    data = json.loads(BASELINE_FILE.read_text(encoding="utf-8"))
    data.setdefault("expected", len(data["selected_photo_ids"]))
    return data


EXPECTED = _baseline()["expected"] if BASELINE_FILE.is_file() else None


def run_regression() -> tuple[bool, dict]:
    result = UAVPhotoOptimizer(RunConfig.create(buffer_m=BUFFER_M)).run()
    actual = len(result.selected_photos)
    info = {"expected": EXPECTED, "actual": actual, **result.counts(),
            "buffer_crs": result.aoi.buffer_crs if result.aoi else None,
            "missing": [], "extra": []}
    ok = actual == EXPECTED

    if BASELINE_FILE.is_file():
        baseline = set(_baseline()["selected_photo_ids"])
        current = {p.photo_id for p in result.selected_photos}
        info["missing"] = sorted(baseline - current)
        info["extra"] = sorted(current - baseline)
        info["id_compare"] = "done"
        ok = ok and not info["missing"] and not info["extra"]
    else:
        info["id_compare"] = "skipped (no baseline_001.json)"
    return ok, info


def main() -> int:
    ok, info = run_regression()
    print("Regression 001")
    print()
    print("Expected:")
    print(info["expected"])
    print()
    print("Actual:")
    print(info["actual"])
    print()
    print(f"(scanned {info['photos_scanned']} / no GPS {info['no_gps']} / errors {info['errors']}"
          f" / buffer CRS {info['buffer_crs']} / photo_id compare: {info['id_compare']})")
    for key in ("missing", "extra"):
        for pid in info[key]:
            print(f"  {key}: {pid}")
    print()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
