"""
Phase 7 synthetic terrain tests T01–T07 + preflight + weitsicht cross-check (Part 22 / 23).

Synthetic GeoTIFFs only (written to tmp_path). Answers are analytic: planes and a tent ridge
whose kinks sit on pixel centres, so the bilinear surface is exactly the analytic surface.
Camera: 4000 × 3000 px, f = 2000 px, on the TWD97 central meridian (grid north = true north).
"""

import math
from pathlib import Path

import numpy as np
import pytest
from pyproj import Transformer
from shapely.geometry import Polygon

rasterio = pytest.importorskip("rasterio")
from rasterio.transform import from_origin  # noqa: E402

from uav_photo_optimizer.camera.models import CameraIntrinsics  # noqa: E402
from uav_photo_optimizer.footprint.base import ProjectionContext  # noqa: E402
from uav_photo_optimizer.footprint.lrf import LRFQualityResult, LRFQualityStatus  # noqa: E402
from uav_photo_optimizer.footprint.models import FootprintMethod, FootprintWarning  # noqa: E402
from uav_photo_optimizer.footprint.planar import PlanarProjector  # noqa: E402
from uav_photo_optimizer.metadata.altitude import HeightStrategy  # noqa: E402
from uav_photo_optimizer.metadata.models import PhotoMetadata  # noqa: E402
from uav_photo_optimizer.terrain import (TerrainConfig, TerrainIssue, TerrainSource,  # noqa: E402
                                         TerrainStatus, VerticalAlignmentMode, VerticalDatumKind)
from uav_photo_optimizer.terrain.alignment import AlignedHeight, align_vertical  # noqa: E402
from uav_photo_optimizer.terrain.footprint import (TerrainFootprintError,  # noqa: E402
                                                   camera_rays, grid_pose_of, project_terrain)
from uav_photo_optimizer.terrain.preflight import terrain_preflight  # noqa: E402
from uav_photo_optimizer.terrain.reference import ReferenceRasterRaySolver  # noqa: E402

CRS = "EPSG:3826"
LON, LAT = 121.0, 24.0
X0, Y0 = Transformer.from_crs("EPSG:4326", CRS, always_xy=True).transform(LON, LAT)
CAM = CameraIntrinsics(width_px=4000, height_px=3000, focal_px=2000, cx_px=2000, cy_px=1500,
                       focal_source="test", principal_point_source="test")
RES = 1.0
HALF = 600                                   # raster half size (m)
ORIGIN = (math.floor(X0) - HALF, math.floor(Y0) + HALF)


def write_dem(path: Path, func, res=RES, half=HALF, nodata=-9999.0, hole=None, crs=CRS,
              bands=1, set_nodata=True) -> Path:
    n = int(2 * half / res)
    x0, y0 = math.floor(X0) - half, math.floor(Y0) + half
    xc = x0 + (np.arange(n) + 0.5) * res
    yc = y0 - (np.arange(n) + 0.5) * res
    gx, gy = np.meshgrid(xc, yc)
    z = func(gx, gy).astype(np.float32)
    if hole is not None:
        xmin, ymin, xmax, ymax = hole
        z[(gx >= xmin) & (gx <= xmax) & (gy >= ymin) & (gy <= ymax)] = nodata
    with rasterio.open(path, "w", driver="GTiff", height=n, width=n, count=bands,
                       dtype="float32", crs=crs, transform=from_origin(x0, y0, res, res),
                       nodata=nodata if set_nodata else None) as ds:
        for b in range(1, bands + 1):
            ds.write(z, b)
    return path


def meta(yaw=0.0, pitch=-90.0, roll=0.0, height=150.0, **kw) -> PhotoMetadata:
    return PhotoMetadata(photo_id=kw.pop("photo_id", "p"), filename="p.jpg", path=Path("p.jpg"),
                         latitude=LAT, longitude=LON, relative_altitude=height,
                         gimbal_yaw=yaw, gimbal_pitch=pitch, gimbal_roll=roll,
                         dewarp_flag=1, rtk_flag=50, **kw)


def planar(m: PhotoMetadata):
    ctx = ProjectionContext(target_crs=CRS, height_strategy=HeightStrategy.TAKEOFF_RELATIVE)
    return PlanarProjector().project(m, CAM, ctx)


def cfg(path, **kw) -> TerrainConfig:
    return TerrainConfig(source=TerrainSource(path=path), **kw)


def plane_hits(o, d, a, s, xref):
    """Rays o + t d on the plane z = a + s (x − xref)."""
    t = (a + s * (o[:, 0] - xref) - o[:, 2]) / (d[:, 2] - s * d[:, 0])
    return o + t[:, None] * d


def ring_dirs(fp, samples=64):
    from uav_photo_optimizer.footprint.base import boundary_pixels
    px, _ = boundary_pixels(CAM, edge_samples=samples // 4)
    return camera_rays(grid_pose_of(fp), CAM, px)


# -- T01 flat ------------------------------------------------------------------------------

@pytest.mark.parametrize("pitch,yaw", [(-90, 0), (-60, 35), (-60, -135)])
def test_t01_flat_equals_planar(tmp_path, pitch, yaw):
    dem = write_dem(tmp_path / "flat.tif", lambda x, y: np.full_like(x, 100.0))
    fp = planar(meta(yaw=yaw, pitch=pitch, height=150))
    with ReferenceRasterRaySolver(dem) as prov:
        tf = project_terrain(fp, CAM, AlignedHeight(photo_id="p", camera_z=250.0,
                                                    method="TEST"), prov, cfg(dem))
    assert tf.method is FootprintMethod.TERRAIN_RASTER
    assert FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR not in tf.warnings
    assert tf.geometry.area == pytest.approx(fp.geometry.area, rel=1e-4)
    assert tf.geometry.hausdorff_distance(fp.geometry) < 0.01
    assert tf.provenance["boundary_failed"] == 0


# -- T02 constant slope -------------------------------------------------------------------

def test_t02_constant_slope_analytic(tmp_path):
    s, a = 0.2, 100.0
    dem = write_dem(tmp_path / "slope.tif", lambda x, y: a + s * (x - X0))
    fp = planar(meta(height=150))
    zc = a + 150.0
    with ReferenceRasterRaySolver(dem) as prov:
        tf = project_terrain(fp, CAM, AlignedHeight(photo_id="p", camera_z=zc, method="TEST"),
                             prov, cfg(dem))
    d = ring_dirs(fp)
    exp = plane_hits(np.tile([X0, Y0, zc], (len(d), 1)), d, a, s, X0)
    got = np.array(tf.geometry.exterior.coords[:-1])
    np.testing.assert_allclose(got, exp[:, :2], atol=0.01)
    assert tf.geometry.area == pytest.approx(Polygon(exp[:, :2]).area, rel=1e-4)
    # east (uphill) edge is closer than on the plane, west (downhill) edge farther
    assert tf.geometry.bounds[2] - X0 < fp.geometry.bounds[2] - X0
    assert X0 - tf.geometry.bounds[0] > X0 - fp.geometry.bounds[0]


# -- T03 ridge (first hit) ----------------------------------------------------------------

def tent(xr, base=100.0, peak=60.0, half=150.0):
    return lambda x, y: base + peak * np.maximum(0.0, 1 - np.abs(x - xr) / half)


def tent_first_hit(o, d, xr, base=100.0, peak=60.0, half=150.0):
    """Exact first intersection with the tent: pieces z = zref + s (x − xref) on [lo, hi]."""
    k = peak / half
    pieces = [(-np.inf, xr - half, 0.0, xr, base), (xr - half, xr, k, xr, base + peak),
              (xr, xr + half, -k, xr, base + peak), (xr + half, np.inf, 0.0, xr, base)]
    out = np.full((len(o), 3), np.nan)
    for i in range(len(o)):
        best = np.inf
        for lo, hi, sl, xref, zref in pieces:
            den = d[i, 2] - sl * d[i, 0]
            if abs(den) < 1e-12:
                continue
            t = (zref + sl * (o[i, 0] - xref) - o[i, 2]) / den
            x = o[i, 0] + t * d[i, 0]
            if 0 < t < best and lo - 1e-9 <= x <= hi + 1e-9:
                best = t
        if np.isfinite(best):
            out[i] = o[i] + best * d[i]
    return out


def test_t03_ridge_first_hit(tmp_path):
    xr = math.floor(X0) + 100.5                      # ridge line on pixel centres
    dem = write_dem(tmp_path / "ridge.tif", tent(xr))
    rng = np.random.default_rng(3)
    n = 400
    o = np.column_stack([np.full(n, X0 - 150), np.full(n, Y0), np.full(n, 320.0)])
    az = rng.uniform(60, 120, n)                     # looking roughly east, across the ridge
    dip = rng.uniform(15, 60, n)
    d = np.column_stack([np.sin(np.radians(az)) * np.cos(np.radians(dip)),
                         np.cos(np.radians(az)) * np.cos(np.radians(dip)),
                         -np.sin(np.radians(dip))])
    exp = tent_first_hit(o, d, xr)
    with ReferenceRasterRaySolver(dem) as prov:
        r = prov.intersect_rays(o, d, max_range=5000)
    ok = np.isfinite(exp).all(axis=1)
    inside = prov.contains_xy(exp[:, 0], exp[:, 1]) & ok
    assert inside.sum() > 300
    np.testing.assert_array_equal(r.valid[inside], True)
    err = np.linalg.norm(r.xyz[inside] - exp[inside], axis=1)
    assert err.max() < 0.02
    # some rays hit the near (west) face although a planar continuation would go behind it
    assert (r.xyz[inside, 0] < xr).sum() > 50 and (r.xyz[inside, 0] > xr).sum() > 20


# -- T04 oblique towards a slope ----------------------------------------------------------

def test_t04_oblique_facing_slope(tmp_path):
    s, a = 0.3, 100.0
    dem = write_dem(tmp_path / "slope.tif", lambda x, y: a + s * (x - X0))
    fp = planar(meta(yaw=90, pitch=-45, height=150))     # looking east = uphill
    zc = a + 150.0
    with ReferenceRasterRaySolver(dem) as prov:
        tf = project_terrain(fp, CAM, AlignedHeight(photo_id="p", camera_z=zc, method="TEST"),
                             prov, cfg(dem))
    d = ring_dirs(fp)
    exp = plane_hits(np.tile([X0, Y0, zc], (len(d), 1)), d, a, s, X0)
    np.testing.assert_allclose(np.array(tf.geometry.exterior.coords[:-1]), exp[:, :2],
                               atol=0.02)
    assert tf.geometry.area < 0.6 * fp.geometry.area   # rising ground → much smaller footprint
    assert FootprintWarning.OBLIQUE_VIEW in tf.warnings


# -- T05 nodata hole ----------------------------------------------------------------------

def test_t05_nodata_hole_issue(tmp_path):
    hole = (X0 + 140, Y0 - 30, X0 + 160, Y0 + 30)   # under the east edge of the footprint
    dem = write_dem(tmp_path / "hole.tif", lambda x, y: np.full_like(x, 100.0), hole=hole)
    fp = planar(meta(height=150))
    h = AlignedHeight(photo_id="p", camera_z=250.0, method="TEST")
    with ReferenceRasterRaySolver(dem) as prov:
        with pytest.raises(TerrainFootprintError) as e:
            project_terrain(fp, CAM, h, prov, cfg(dem))
        assert e.value.issue is TerrainIssue.TERRAIN_NODATA
        part = project_terrain(fp, CAM, h, prov, cfg(dem, max_failed_boundary_ratio=0.25))
        assert FootprintWarning.TERRAIN_PARTIAL_BOUNDARY in part.warnings
        assert part.provenance["boundary_failed"] > 0
        # a ray through the hole is never reported as a hit behind it
        r = prov.intersect_rays([[X0 + 150, Y0, 250.0]], [[0.0, 0.0, -1.0]])
        assert not r.valid[0] and r.issues[0] is TerrainIssue.TERRAIN_NODATA and r.nodata_hit[0]


# -- T06 outside --------------------------------------------------------------------------

def test_t06_outside_raster_no_crash(tmp_path):
    dem = write_dem(tmp_path / "small.tif", lambda x, y: np.full_like(x, 100.0), half=50)
    with ReferenceRasterRaySolver(dem) as prov:
        r = prov.intersect_rays([[X0 + 5000, Y0, 250], [X0, Y0, 250], [X0, Y0, 250]],
                                [[0, 0, -1.0], [1.0, 0, -0.05], [0, 0, 1.0]], max_range=3000)
        assert not r.valid.any()
        assert r.issues[0] is TerrainIssue.TERRAIN_OUTSIDE        # never inside
        assert r.issues[1] is TerrainIssue.TERRAIN_OUTSIDE        # leaves before a hit
        assert r.issues[2] is TerrainIssue.TERRAIN_NO_INTERSECTION  # upwards
        fp = planar(meta(height=150))                               # 300 m wide > raster
        with pytest.raises(TerrainFootprintError) as e:
            project_terrain(fp, CAM, AlignedHeight(photo_id="p", camera_z=250.0, method="T"),
                            prov, cfg(dem))
        assert e.value.issue is TerrainIssue.TERRAIN_OUTSIDE


# -- T07 vertical datum gate --------------------------------------------------------------

def _anchor_fp(pid, laser_h=150.0, status=LRFQualityStatus.VALID):
    fp = planar(meta(photo_id=pid, height=laser_h))
    return fp.model_copy(update={"height_strategy": "LRF_RAY_VERTICAL", "lrf_quality":
                                 LRFQualityResult(status=status, laser_vertical_height_m=laser_h)})


def test_t07_unknown_datum_refused(tmp_path):
    dem = write_dem(tmp_path / "flat.tif", lambda x, y: np.full_like(x, 100.0))
    src = TerrainSource(path=dem)                         # vertical datum UNKNOWN
    m = {"p": meta(absolute_altitude=270.0)}
    fps = {"p": planar(m["p"])}                           # no LRF anchor
    with ReferenceRasterRaySolver(src) as prov:
        assert prov.vertical_datum is VerticalDatumKind.UNKNOWN
        a = align_vertical(prov, TerrainConfig(source=src), m, fps, target_crs=CRS)
        assert a.status is TerrainStatus.NOT_READY and a.mode is VerticalAlignmentMode.LRF_ANCHORED
        a = align_vertical(prov, TerrainConfig(
            source=src, alignment=VerticalAlignmentMode.ASSUME_SAME_DATUM), m, fps)
        assert a.status is TerrainStatus.NOT_READY        # not confirmed by the user
        a = align_vertical(prov, TerrainConfig(
            source=src, alignment=VerticalAlignmentMode.EXPLICIT_VERTICAL_TRANSFORM,
            explicit_offset_m=-20.0), m, fps)
        assert a.status is TerrainStatus.NOT_READY        # DEM datum unknown
        a = align_vertical(prov, TerrainConfig(
            source=src, alignment=VerticalAlignmentMode.ASSUME_SAME_DATUM,
            assume_same_datum_confirmed=True), m, fps)
        assert a.status is TerrainStatus.READY and a.heights["p"].camera_z == 270.0
        assert a.warnings


def test_lrf_anchored_alignment(tmp_path):
    dem = write_dem(tmp_path / "flat.tif", lambda x, y: np.full_like(x, 100.0))
    src = TerrainSource(path=dem)
    ids = [f"a{i}" for i in range(25)]
    # AbsoluteAltitude in an unknown datum, 20 m above the DEM datum (e.g. a geoid offset)
    m = {p: meta(photo_id=p, absolute_altitude=270.0, lrf_target_lat=LAT, lrf_target_lon=LON)
         for p in ids}
    m["n"] = meta(photo_id="n", absolute_altitude=275.0)
    fps = {p: _anchor_fp(p) for p in ids}
    with ReferenceRasterRaySolver(src) as prov:
        a = align_vertical(prov, TerrainConfig(source=src), m, fps, target_crs=CRS)
        assert a.status is TerrainStatus.READY, a.reasons
        assert a.offset_median_m == pytest.approx(-20.0, abs=1e-3)
        assert a.heights["a0"].camera_z == pytest.approx(250.0, abs=1e-3)
        assert a.heights["n"].method == "FLIGHT_OFFSET"
        assert a.heights["n"].camera_z == pytest.approx(255.0, abs=1e-3)
        noise = np.random.default_rng(0).normal(0, 4.0, len(ids))
        bad = {p: _anchor_fp(p, laser_h=150.0 + e) for p, e in zip(ids, noise)}
        a = align_vertical(prov, TerrainConfig(source=src, max_anchor_offset_spread_m=1.0),
                           m, bad, target_crs=CRS)
        assert a.status is TerrainStatus.NOT_READY        # anchors disagree


# -- preflight ----------------------------------------------------------------------------

def test_preflight_checks(tmp_path):
    flat = lambda x, y: np.full_like(x, 100.0)  # noqa: E731
    ok = write_dem(tmp_path / "ok.tif", flat)
    aoi = Polygon([(X0 - 50, Y0 - 50), (X0 + 50, Y0 - 50), (X0 + 50, Y0 + 50), (X0 - 50, Y0 + 50)])
    d = terrain_preflight(TerrainSource(path=ok), aoi=aoi, aoi_crs=CRS)
    assert d.status is TerrainStatus.READY, d.reasons
    assert d.aoi_coverage == 1.0
    far = Polygon([(0, 0), (10, 0), (10, 10)])
    assert terrain_preflight(TerrainSource(path=ok), aoi=far, aoi_crs=CRS).status \
        is TerrainStatus.NOT_READY
    for name, kw in [("ll", {"crs": "EPSG:4326", "res": 0.0001, "half": 0.01}),
                     ("nocrs", {"crs": None}), ("two", {"bands": 2}),
                     ("nond", {"set_nodata": False})]:
        p = write_dem(tmp_path / f"{name}.tif", flat, **kw)
        assert terrain_preflight(TerrainSource(path=p)).status is TerrainStatus.NOT_READY, name
    assert terrain_preflight(TerrainSource(path=tmp_path / "missing.tif")).status \
        is TerrainStatus.NOT_READY
    assert terrain_preflight(TerrainSource(path=ok, vertical_unit="foot")).status \
        is TerrainStatus.NOT_READY


def test_disk_and_preload_identical(tmp_path):
    dem = write_dem(tmp_path / "ridge.tif", tent(math.floor(X0) + 100.5))
    rng = np.random.default_rng(1)
    o = np.column_stack([X0 + rng.uniform(-100, 100, 200), Y0 + rng.uniform(-100, 100, 200),
                         np.full(200, 400.0)])
    d = np.column_stack([rng.uniform(-0.6, 0.6, 200), rng.uniform(-0.6, 0.6, 200),
                         -np.ones(200)])
    with ReferenceRasterRaySolver(dem, preload=True) as a, \
            ReferenceRasterRaySolver(dem, preload=False) as b:
        ra, rb = a.intersect_rays(o, d), b.intersect_rays(o, d)
        np.testing.assert_array_equal(ra.valid, rb.valid)
        np.testing.assert_allclose(ra.xyz, rb.xyz, atol=1e-9)
        assert b.grid.reads > 0


# -- Part 23: weitsicht cross-check -------------------------------------------------------

def test_weitsicht_cross_check(tmp_path):
    pytest.importorskip("weitsicht")
    from uav_photo_optimizer.terrain.weitsicht_provider import WeitsichtRasterTerrainProvider
    s, a = 0.3, 100.0
    dem = write_dem(tmp_path / "slope.tif", lambda x, y: a + s * (x - X0),
                    hole=(X0 + 200, Y0 - 20, X0 + 240, Y0 + 20))
    rng = np.random.default_rng(7)
    n = 300
    o = np.column_stack([X0 + rng.uniform(-150, 150, n), Y0 + rng.uniform(-150, 150, n),
                         np.full(n, 450.0)])
    d = np.column_stack([rng.uniform(-0.5, 0.5, n), rng.uniform(-0.5, 0.5, n), -np.ones(n)])
    src = TerrainSource(path=dem, vertical_crs="EPSG:8904",
                        vertical_datum=VerticalDatumKind.ORTHOMETRIC)
    with ReferenceRasterRaySolver(src) as ref, WeitsichtRasterTerrainProvider(src) as ws:
        rr, rw = ref.intersect_rays(o, d), ws.intersect_rays(o, d)
    # every reference hit is a weitsicht hit; the only difference is policy: a ray whose path
    # crosses a nodata hole *before* its hit is TERRAIN_NODATA for the reference solver
    # (first hit not provable), while weitsicht does not inspect the path
    assert not (rr.valid & ~rw.valid).any()
    diff = rw.valid & ~rr.valid
    assert all(rr.issues[i] is TerrainIssue.TERRAIN_NODATA for i in np.flatnonzero(diff))
    same_fail = ~rr.valid & ~rw.valid
    assert all(rr.issues[i] is rw.issues[i] for i in np.flatnonzero(same_fail))
    both = rr.valid & rw.valid
    assert both.sum() > 250
    assert np.abs(rr.xyz[both, :2] - rw.xyz[both, :2]).max() < 0.05
    assert np.abs(rr.xyz[both, 2] - rw.xyz[both, 2]).max() < 0.02
