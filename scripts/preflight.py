"""
Local preflight: project layout, tools, environment, tests and Regression 001.

    python scripts/preflight.py            # everything
    python scripts/preflight.py --quick    # skip pytest and Regression 001

Read only: nothing in input/ or output/ is modified.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    results: list[tuple[bool, str, str]] = []

    def report(ok: bool, label: str, detail: str = "") -> bool:
        results.append((ok, label, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {label}")
        if detail:
            print(detail)
        print()
        return ok

    try:
        import uav_photo_optimizer
        from uav_photo_optimizer import UAVPhotoOptimizer
        from uav_photo_optimizer.core.exceptions import UAVPhotoOptimizerError
        from uav_photo_optimizer.paths import DEFAULT_PATHS as P
    except Exception as exc:                       # noqa: BLE001
        report(False, "Core package import", str(exc))
        return 1

    report(P.base_dir.is_dir(), "Project Root", str(P.base_dir))
    report(any(P.shp_dir.glob("*.shp")), r"input\shp", str(P.shp_dir))
    report(P.photo_dir.is_dir(), r"input\photo", str(P.photo_dir))
    report(P.output_dir.is_dir(), "output", str(P.output_dir))

    engine = UAVPhotoOptimizer()
    try:
        report(True, "ExifTool", f"{engine.exiftool.version}  ({engine.exiftool.locate()})")
    except UAVPhotoOptimizerError as exc:
        report(False, "ExifTool", str(exc))

    vi = sys.version_info
    report(vi >= (3, 12), "Python environment", f"Python {vi.major}.{vi.minor}.{vi.micro}  ({sys.executable})")
    report(True, "Core package import", f"uav_photo_optimizer {uav_photo_optimizer.__version__}")

    if not args.quick:
        proc = subprocess.run([sys.executable, "-m", "pytest", "-q"], cwd=P.base_dir,
                              capture_output=True, text=True, encoding="utf-8", errors="replace")
        tail = (proc.stdout.strip().splitlines() or ["(no output)"])[-1]
        report(proc.returncode == 0, "pytest", tail)

        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from run_regression_001 import EXPECTED, run_regression
        try:
            ok, info = run_regression()
            detail = f"Expected = {EXPECTED}\nActual   = {info['actual']}"
            if info["missing"] or info["extra"]:
                detail += f"\nmissing={len(info['missing'])} extra={len(info['extra'])}"
        except UAVPhotoOptimizerError as exc:
            ok, detail = False, str(exc)
        report(ok, "Regression Baseline", detail)

    failed = [label for ok, label, _ in results if not ok]
    print("ALL PASS" if not failed else f"FAILED: {', '.join(failed)}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
