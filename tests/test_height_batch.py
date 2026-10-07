"""Height Strategy v2 batch resolution: validated LRF → neighbour interpolation → UNRESOLVED."""

import pytest
from pyproj import Geod

from synthetic_flight import ABS_ALT, lawnmower
from uav_photo_optimizer.flight.strip import analyse_flights
from uav_photo_optimizer.footprint.base import ProjectionContext
from uav_photo_optimizer.footprint.batch import NeighbourConfig, project_all
from uav_photo_optimizer.footprint.planar import PlanarProjector
from uav_photo_optimizer.metadata.altitude import HeightMethod, HeightStatus, HeightStrategy

CRS = "EPSG:3826"
GEOD = Geod(ellps="WGS84")


def break_lrf(m):
    """Move the laser target 60 m sideways → beam misaligned → INVALID."""
    lon, lat, _ = GEOD.fwd(m.longitude, m.latitude, (m.gimbal_yaw + 90) % 360, 60)
    return m.model_copy(update={"lrf_target_lon": lon, "lrf_target_lat": lat})


def run(photos, **kw):
    a = analyse_flights(photos, CRS)
    return a, project_all(photos, a, PlanarProjector(), ProjectionContext(target_crs=CRS),
                          neighbours=NeighbourConfig(**kw) if kw else NeighbourConfig())


def test_all_valid_use_lrf_ray():
    _, res = run(lawnmower(strips=2, photo_spacing=15))
    assert {r.method for r in res.resolutions.values()} == {HeightMethod.LRF_RAY_VERTICAL}
    assert all(fp.height_m == pytest.approx(100.0) for fp in res.footprints.values())


def test_invalid_middle_photo_interpolated():
    photos = lawnmower(strips=1, photo_spacing=15)          # 41 photos, one strip
    photos[10] = break_lrf(photos[10])
    _, res = run(photos)
    r = res.resolutions[photos[10].photo_id]
    assert r.method is HeightMethod.NEIGHBOR_LRF_INTERPOLATED and r.lrf_status == "INVALID"
    assert r.neighbours == [photos[9].photo_id, photos[11].photo_id]
    fp = res.footprints[photos[10].photo_id]
    assert fp.height_m == pytest.approx(100.0, abs=1e-6)       # flat ground
    assert fp.height_confidence == 0.7 and "NEIGHBOR_HEIGHT_INTERPOLATED" in fp.warnings


def test_strip_end_nearest_or_unresolved():
    photos = lawnmower(strips=1, photo_spacing=15)
    photos[-1] = break_lrf(photos[-1])                       # only a previous neighbour, 15 m
    _, res = run(photos)
    r = res.resolutions[photos[-1].photo_id]
    assert r.method is HeightMethod.NEIGHBOR_LRF_NEAREST
    assert res.footprints[photos[-1].photo_id].height_confidence == 0.5
    far = lawnmower(strips=1, photo_spacing=30)              # neighbour 30 m > 20 m limit
    far[-1] = break_lrf(far[-1])
    _, res2 = run(far)
    assert res2.resolutions[far[-1].photo_id].status is HeightStatus.HEIGHT_UNRESOLVED
    assert far[-1].photo_id not in res2.footprints


def test_no_interpolation_across_gap_or_turn():
    photos = lawnmower(strips=2, photo_spacing=15, turn_photos=3)
    a, _ = run(photos)
    turn = next(p for p in photos if a.photos[p.photo_id].strip_id is None)
    idx = photos.index(turn)
    photos[idx] = break_lrf(turn)
    _, res = run(photos)
    assert res.resolutions[turn.photo_id].status is HeightStatus.HEIGHT_UNRESOLVED
    # interpolation distance limit
    p = lawnmower(strips=1, photo_spacing=15)
    for i in (9, 10, 11, 12, 13):          # 5 invalid in a row → nearest valid 45 m away
        p[i] = break_lrf(p[i])
    _, res3 = run(p)
    assert res3.resolutions[p[11].photo_id].status is HeightStatus.HEIGHT_UNRESOLVED
    assert res3.resolutions[p[9].photo_id].method is HeightMethod.NEIGHBOR_LRF_NEAREST


def test_never_relative_altitude_in_auto():
    photos = lawnmower(strips=1, lrf=False)
    _, res = run(photos)
    assert not res.footprints
    assert all(r.status is HeightStatus.HEIGHT_UNRESOLVED for r in res.resolutions.values())


def test_interpolates_ground_not_height():
    photos = lawnmower(strips=1, photo_spacing=15)
    # sloped ground: LRF ground rises 1 m per photo; camera altitude constant
    sloped = []
    for i, m in enumerate(photos):
        h = 100.0 - i
        sloped.append(m.model_copy(update={"lrf_target_distance": h,
                                           "lrf_target_abs_alt": ABS_ALT - h}))
    sloped[10] = break_lrf(sloped[10])
    _, res = run(sloped)
    assert res.footprints[sloped[10].photo_id].height_m == pytest.approx(90.0, abs=1e-6)
