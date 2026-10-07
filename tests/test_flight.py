from datetime import datetime, timedelta

import pytest

from synthetic_flight import lawnmower
from uav_photo_optimizer.flight.models import CaptureType, SegmentType, ViewDirection
from uav_photo_optimizer.flight.strip import (FlightConfig, analyse_flights, angdiff, axis_mean,
                                              capture_type, view_direction)

CRS = "EPSG:3826"


def test_strips_back_and_forth():
    photos = lawnmower(strips=4)
    a = analyse_flights(photos, CRS)
    assert len(a.flights) == 1 and len(a.strips) == 4
    travel = [s.travel_heading for s in a.strips]
    assert travel == pytest.approx([90, 270, 90, 270], abs=0.01)
    assert all(s.axis_heading == pytest.approx(90, abs=0.01) for s in a.strips)
    assert all(len(s.photo_ids) == 21 for s in a.strips)          # 600 m / 30 m + 1
    assert all(s.cross_track_rms_m < 1e-6 for s in a.strips)


def test_turn_photos_not_in_strips():
    photos = lawnmower(strips=3, turn_photos=3)
    a = analyse_flights(photos, CRS)
    turns = [p for p in a.photos.values() if p.segment_type is not SegmentType.STRIP]
    assert len(turns) == 6
    assert all(p.strip_id is None for p in turns)
    in_strips = {pid for s in a.strips for pid in s.photo_ids}
    assert not in_strips & {p.photo_id for p in turns}


def test_strip_heading_ignores_gimbal_yaw():
    photos = lawnmower(strips=2)
    # gimbal pointing sideways must not change strips
    photos = [p.model_copy(update={"gimbal_yaw": (p.flight_yaw + 120) % 360}) for p in photos]
    a = analyse_flights(photos, CRS)
    assert [round(s.travel_heading) for s in a.strips] == [90, 270]


def test_flight_split_on_time_gap_and_sequence_reset():
    m1 = lawnmower(strips=2, folder="A")
    m2 = lawnmower(strips=2, folder="A", start=m1[-1].capture_time + timedelta(seconds=300))
    a = analyse_flights(m1 + m2, CRS)
    assert len(a.flights) == 2
    assert "time gap" in a.flights[1].split_reason or "sequence" in a.flights[1].split_reason
    # sequence reset alone (no time gap, same folder)
    m3 = lawnmower(strips=2, folder="B", start=m1[-1].capture_time + timedelta(seconds=2),
                   seq_start=1)
    a2 = analyse_flights(m1 + [p.model_copy(update={"photo_id": "A/" + p.filename})
                               for p in m3], CRS)
    assert len(a2.flights) == 2 and "sequence reset" in a2.flights[1].split_reason


def test_folder_is_not_the_only_criterion():
    m = lawnmower(strips=2, folder="A")
    half = len(m) // 2
    moved = m[:half] + [p.model_copy(update={"photo_id": "B/" + p.filename}) for p in m[half:]]
    assert len(analyse_flights(moved, CRS, FlightConfig(split_on_folder=False)).flights) == 1


def test_capture_type_thresholds_configurable():
    cfg = FlightConfig()
    assert capture_type(-90, cfg) is CaptureType.NADIR
    assert capture_type(-87, cfg) is CaptureType.NADIR
    assert capture_type(-60, cfg) is CaptureType.OBLIQUE
    assert capture_type(-10, cfg) is CaptureType.OTHER
    assert capture_type(-75, FlightConfig(nadir_max_pitch=-70)) is CaptureType.NADIR


def test_view_direction_relative_to_travel():
    photos = lawnmower(strips=2, oblique_every=1)
    a = analyse_flights(photos, CRS)
    obl = [p for p in a.photos.values() if p.capture_type is CaptureType.OBLIQUE
           and p.segment_type is SegmentType.STRIP]
    assert obl and all(p.view_direction is ViewDirection.FORWARD_LEFT for p in obl)
    assert all(p.relative_view_yaw == pytest.approx(-45, abs=0.01) for p in obl)


@pytest.mark.parametrize("rel, expected", [(0, "FORWARD"), (44, "FORWARD_RIGHT"),
                                           (-135, "BACKWARD_LEFT"), (180, "BACKWARD"),
                                           (-90, "LEFT")])
def test_view_direction_sectors(rel, expected):
    assert view_direction(rel, FlightConfig()).value == expected


def test_angle_helpers():
    assert angdiff(10, 350) == pytest.approx(20)
    assert angdiff(350, 10) == pytest.approx(-20)
    assert axis_mean([89, 271]) == pytest.approx(90, abs=1e-6)     # 271 ≡ 91 as an axis
