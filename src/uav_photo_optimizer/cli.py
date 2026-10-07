"""
CLI (thin wrapper — no algorithm here).

Official command (v1.0) — the six user parameters:

    uav-photo-optimizer select --aoi area.shp --photos PHOTO_DIR --output OUT_DIR
                               [--buffer 100] [--front 80] [--side 70] [--dry-run]

Photos are only COPIED (never moved, deleted or overwritten) into a new
OUT_DIR/run_YYYYMMDD_HHMMSS/ folder. The tool gives a geometric photo selection
recommendation; it does not guarantee that photogrammetric modelling succeeds.

Other commands:

    uav-photo-optimizer gui      simple GUI (six fields + 開始篩選)
    uav-photo-optimizer serve    REST API v1 on http://127.0.0.1:8765
    uav-photo-optimizer preflight
    uav-photo-optimizer run      legacy GPS-point AOI selection (Regression 001)
    uav-photo-optimizer optimize advanced plan inspection [--visual] [--export] [--json]

Experimental (optional, default OFF, not needed for the normal workflow):

    uav-photo-optimizer optimize --visual          Visual Guard (needs extra `visual`)
    uav-photo-optimizer analyse-terrain --dem ...  Terrain-aware footprints (extra `terrain`)
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

from pydantic import ValidationError

from . import __version__
from .core.config import (DEFAULT_BUFFER_M, DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP,
                          RunConfig)
from .core.engine import UAVPhotoOptimizer
from .core.exceptions import UAVPhotoOptimizerError
from .export import copy_selected, new_run_dir  # noqa: F401  (new_run_dir re-exported)

LINE = "=" * 40


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="uav-photo-optimizer",
                                description="UAV Photo Optimizer / UAV 照片篩選工具")
    p.add_argument("--version", action="version", version=__version__)
    p.add_argument("--base-dir", type=Path, help="Project root (default: auto-detect)")
    p.add_argument("--shapefile", type=Path, help="AOI .shp (default: the single .shp in input/shp)")
    p.add_argument("--photo-dir", type=Path, help="Photo root (default: input/photo)")
    p.add_argument("--exiftool", type=Path, help="ExifTool executable")
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("select", help="Select photos by AOI + overlap and COPY them (official)")
    s.add_argument("--aoi", type=Path, required=True, help="建模範圍 AOI Shapefile (.shp)")
    s.add_argument("--photos", type=Path, required=True, help="照片資料夾 (searched recursively)")
    s.add_argument("--output", type=Path, help="輸出資料夾 (required unless --dry-run)")
    s.add_argument("--buffer", type=float, default=DEFAULT_BUFFER_M, help="Buffer (m), default 100")
    s.add_argument("--front", type=float, default=DEFAULT_FRONT_OVERLAP,
                   help="航向重疊率 %% (65-95), default 80")
    s.add_argument("--side", type=float, default=DEFAULT_SIDE_OVERLAP,
                   help="側向重疊率 %% (65-95), default 70")
    s.add_argument("--dry-run", action="store_true", help="Only report, copy nothing")
    s.add_argument("--json", type=Path, help="Write the result JSON to this file")

    sv = sub.add_parser("serve", help="Start the REST API server (needs extra `api`)")
    sv.add_argument("--host", default="127.0.0.1")
    sv.add_argument("--port", type=int, default=8765)

    sub.add_parser("gui", help="Open the simple GUI (needs extra `gui`)")

    sub.add_parser("preflight", help="Check inputs and tools (read only)")

    r = sub.add_parser("run", help="Run selection")
    r.add_argument("--buffer", type=float, default=DEFAULT_BUFFER_M, help="Buffer (m), >= 0")
    r.add_argument("--front", type=float, default=DEFAULT_FRONT_OVERLAP, help="Front overlap %% (65-95)")
    r.add_argument("--side", type=float, default=DEFAULT_SIDE_OVERLAP, help="Side overlap %% (65-95)")
    r.add_argument("--copy", action="store_true", help="Copy selected photos to --output")
    r.add_argument("--output", type=Path,
                   help="Copy target (default: new folder output/run_YYYYMMDD_HHMMSS/)")
    r.add_argument("--json", type=Path, help="Write SelectionResult JSON to this file")

    o = sub.add_parser("optimize", help="Photo reduction plan (dry run unless --export)")
    o.add_argument("--buffer", type=float, default=DEFAULT_BUFFER_M, help="Buffer (m), >= 0")
    o.add_argument("--front", type=float, default=DEFAULT_FRONT_OVERLAP, help="Front target %% (65-95)")
    o.add_argument("--side", type=float, default=DEFAULT_SIDE_OVERLAP, help="Side target %% (65-95)")
    o.add_argument("--visual", action="store_true",
                   help="Experimental: run the optional Visual Guard (needs extra `visual`)")
    o.add_argument("--json", type=Path, help="Write the SelectionPlan JSON to this file")
    o.add_argument("--export", action="store_true",
                   help="COPY the selected photos (VALID plan; VALIDATED with --visual)")
    o.add_argument("--output", type=Path, help="Export root (default: output/)")

    t = sub.add_parser("analyse-terrain",
                       help="Terrain-aware footprints from a local GeoTIFF (Experimental, dry run)")
    t.add_argument("--dem", type=Path, required=True, help="Local single-band GeoTIFF DEM / DSM")
    t.add_argument("--kind", default="UNKNOWN", choices=["DEM", "DSM", "DERIVED", "UNKNOWN"])
    t.add_argument("--vertical-datum", default="UNKNOWN",
                   choices=["ORTHOMETRIC", "ELLIPSOIDAL", "LOCAL", "UNKNOWN"])
    t.add_argument("--vertical-crs", help="Vertical CRS of the raster, e.g. EPSG:8904")
    t.add_argument("--not-independent", action="store_true",
                   help="Surface derived from the same photos / LRF")
    t.add_argument("--alignment", default="AUTO",
                   choices=["AUTO", "LRF_ANCHORED", "EXPLICIT_VERTICAL_TRANSFORM",
                            "ASSUME_SAME_DATUM"])
    t.add_argument("--explicit-offset", type=float,
                   help="EXPLICIT_VERTICAL_TRANSFORM: Z_terrain = AbsoluteAltitude + offset")
    t.add_argument("--assume-same-datum", action="store_true",
                   help="Confirm ASSUME_SAME_DATUM (photo altitude and raster share a datum)")
    t.add_argument("--engine", default="REFERENCE", choices=["REFERENCE", "WEITSICHT"])
    t.add_argument("--samples", type=int, default=64, help="Boundary rays per photo")
    t.add_argument("--max-failed", type=float, default=0.0,
                   help="Max share of failed boundary rays per photo")
    t.add_argument("--buffer", type=float, default=DEFAULT_BUFFER_M, help="Buffer (m), >= 0")
    t.add_argument("--json", type=Path, help="Write the terrain report JSON to this file")
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        common = dict(base_dir=args.base_dir, photo_dir=args.photo_dir,
                      exiftool_path=args.exiftool)
        if args.command == "preflight":
            engine = UAVPhotoOptimizer(RunConfig.create(shapefile=args.shapefile, **common))
            checks = engine.preflight()
            for c in checks:
                print(f"[{'PASS' if c.ok else 'FAIL'}] {c.name}: {c.detail}")
            return 0 if all(c.ok for c in checks) else 1

        if args.command == "serve":
            from .api.app import serve
            serve(args.host, args.port)
            return 0
        if args.command == "gui":
            from .gui import main as gui_main
            return gui_main()
        if args.command == "select":
            return _select(args, common)
        if args.command == "optimize":
            return _optimize(args, common)
        if args.command == "analyse-terrain":
            return _analyse_terrain(args, common)

        config = RunConfig.create(buffer_m=args.buffer, front_overlap=args.front,
                                  side_overlap=args.side, shapefile=args.shapefile, **common)
        engine = UAVPhotoOptimizer(config)
        result = engine.run(progress=lambda stage, d, t: print(f"{stage}: {d} / {t}"))

        print(LINE)
        for key, value in result.counts().items():
            print(f"{key:<16}{value}")
        for w in result.warnings:
            print(f"WARNING: {w}")
        for e in result.errors:
            print(f"ERROR: {e.path} : {e.message}")

        if args.json:
            args.json.write_text(result.model_dump_json(indent=2), encoding="utf-8")
            print(f"Result JSON: {args.json}")
        if args.copy:
            out_dir = args.output or new_run_dir(engine.paths.output_dir)
            exported = copy_selected(result, out_dir, input_dir=engine.paths.input_dir)
            print(f"Copied: {len(exported.copied)}  Skipped: {len(exported.skipped)}  → {out_dir}")
            for p, why in exported.skipped:
                print(f"SKIP: {p} : {why}")
        return 0
    except UAVPhotoOptimizerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except ImportError as exc:                 # optional extra not installed
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except ValidationError as exc:
        msgs = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in exc.errors())
        print(f"ERROR: invalid parameter — {msgs}", file=sys.stderr)
        return 2


def _select(args, common) -> int:
    from .core.selection import SelectionRequest, select_photos

    req = SelectionRequest(aoi_shapefile=args.aoi, photo_dir=args.photos,
                           output_dir=args.output, buffer_m=args.buffer,
                           front_overlap=args.front, side_overlap=args.side,
                           copy_photos=not args.dry_run)
    labels = {"scan": "搜尋照片", "metadata": "讀取照片資訊", "geometry": "建立照片幾何",
              "optimize": "計算重疊與減量", "copy": "複製照片", "done": "完成"}

    def progress(stage, done, total):
        if stage in ("metadata", "copy") and total:
            if done == total or done % 500 == 0:
                print(f"{labels[stage]}: {done} / {total}")
        elif stage != "done":
            print(f"{labels.get(stage, stage)} ...")

    s = select_photos(req, progress=progress, base_dir=common.get("base_dir"))
    print(LINE)
    print("篩選完成")
    print(f"候選照片：{s.candidate_photos}")
    print(f"保留照片：{s.selected_photos}")
    print(f"減少照片：{s.removed_photos}")
    print(f"減量比例：{s.reduction_percent:.1f}%")
    for n in s.notes:
        print(n)
    print(f"輸出：{s.output_dir}" if s.output_dir else "DRY RUN：未複製照片。")
    if args.json:
        args.json.write_text(s.model_dump_json(indent=1), encoding="utf-8")
    return 0


def _optimize(args, common) -> int:
    config = RunConfig.create(buffer_m=args.buffer, front_overlap=args.front,
                              side_overlap=args.side, shapefile=args.shapefile, **common)
    engine = UAVPhotoOptimizer(config)
    photos = __import__("uav_photo_optimizer.metadata.scanner", fromlist=["x"]).find_photos(
        engine.photo_dir)
    read = engine.exiftool.read(photos, photo_root=engine.photo_dir,
                                progress=lambda d, t: print(f"metadata: {d} / {t}"))
    geometry = engine.geometry_from_metadata(read.photos, len(photos))
    plan = engine.plan_selection(geometry=geometry)
    if args.visual:
        print("visual validation ...")
        plan = engine.validate_selection(plan, geometry, {m.photo_id: m for m in read.photos})
    print(LINE)
    print(f"status {plan.status.value} | candidates {plan.candidate_photo_count} | keep "
          f"{plan.keep_count} | remove {plan.remove_count} | protected {plan.protected_count} | "
          f"reduction {plan.reduction_percent:.1f} % | geometric restores {plan.restored_count}"
          f" | visual restores {plan.visual_restore_count}")
    if args.json:
        args.json.write_text(plan.model_dump_json(indent=1), encoding="utf-8")
        print(f"SelectionPlan JSON: {args.json}")
    if not args.export:
        print("DRY RUN: nothing copied (use --export to copy the selected photos).")
        return 0 if plan.status.value != "INVALID" else 1
    res = engine.export_selection(plan, {m.photo_id: m for m in read.photos}, args.output,
                                  require_visual=args.visual)
    print(f"Exported {res.copied} photos → {res.photos_dir}")
    return 0


def _analyse_terrain(args, common) -> int:
    import json

    from .terrain import (TerrainConfig, TerrainEngine, TerrainSource, TerrainSurfaceKind,
                          VerticalAlignmentMode, VerticalDatumKind)
    from .terrain.compare import compare_footprints

    config = RunConfig.create(buffer_m=args.buffer, shapefile=args.shapefile, **common)
    engine = UAVPhotoOptimizer(config)
    tcfg = TerrainConfig(
        source=TerrainSource(path=args.dem, kind=TerrainSurfaceKind(args.kind),
                             vertical_datum=VerticalDatumKind(args.vertical_datum),
                             vertical_crs=args.vertical_crs,
                             independent=not args.not_independent),
        engine=TerrainEngine(args.engine), alignment=VerticalAlignmentMode(args.alignment),
        explicit_offset_m=args.explicit_offset,
        assume_same_datum_confirmed=args.assume_same_datum,
        boundary_samples=args.samples, max_failed_boundary_ratio=args.max_failed)
    photos = __import__("uav_photo_optimizer.metadata.scanner", fromlist=["x"]).find_photos(
        engine.photo_dir)
    read = engine.exiftool.read(photos, photo_root=engine.photo_dir,
                                progress=lambda d, t: print(f"metadata: {d} / {t}"))
    meta = {m.photo_id: m for m in read.photos}
    geometry = engine.geometry_from_metadata(read.photos, len(photos))
    ta = engine.analyse_terrain(geometry, meta, tcfg)
    print(LINE)
    print(f"terrain status {ta.status.value}")
    for c in ta.diagnostics.checks:
        print(f"[{'PASS' if c.ok else 'FAIL'}] {c.name}: {c.detail}")
    for r in ta.diagnostics.reasons:
        print(f"NOT READY: {r}")
    report = {"status": ta.status.value, "diagnostics": ta.diagnostics.model_dump(mode="json")}
    if ta.geometry is not None:
        _, aoi = engine.build_aoi()
        tfp = {p: f for p, f in ta.geometry.footprints.items()
               if f.method.value == "TERRAIN_RASTER"}
        cmp = compare_footprints(geometry.footprints, tfp, aoi.geometry)
        report.update({"terrain_footprints": ta.terrain_footprints,
                       "issue_counts": ta.issue_counts, "rays": ta.rays,
                       "seconds": ta.seconds, "comparison": cmp,
                       "overlap_planar": geometry.statistics,
                       "overlap_terrain": ta.geometry.statistics})
        print(f"terrain footprints {ta.terrain_footprints} | failed {len(ta.terrain_failed)} "
              f"{ta.issue_counts} | {ta.rays} rays in {ta.seconds:.1f} s")
        print(f"candidates {cmp.get('candidates')}")
    if args.json:
        args.json.write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")
        print(f"Terrain report JSON: {args.json}")
    print("DRY RUN: nothing copied.")
    return 0 if ta.status.value == "READY" else 1


if __name__ == "__main__":
    sys.exit(main())
