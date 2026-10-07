from datetime import datetime
from pathlib import Path

import pytest

from conftest import X0, Y0, to_lonlat, write_jpg
from uav_photo_optimizer.core.exceptions import ExifToolNotFoundError, PhotoSourceError
from uav_photo_optimizer.metadata.altitude import AltitudeDatum
from uav_photo_optimizer.metadata.dji import parse_gps, record_to_metadata
from uav_photo_optimizer.metadata.exiftool import ExifToolAdapter
from uav_photo_optimizer.metadata.scanner import find_photos
from uav_photo_optimizer.paths import ProjectPaths

# Shape of a real DJI M4E record from ``exiftool -json -n -a -G1`` (positions replaced by synthetic values).
DJI_RECORD = {
    "SourceFile": "x.JPG", "File:FileType": "JPEG",
    "Composite:GPSLatitude": 24.0, "Composite:GPSLongitude": 121.0,
    "GPS:GPSAltitude": 592.502, "GPS:GPSAltitudeRef": 0,
    "IFD0:Make": "DJI", "IFD0:Model": "M4E",
    "XMP-drone-dji:ProductName": "DJI Matrice 4E", "XMP-drone-dji:ImageSource": "WideCamera",
    "XMP-drone-dji:AbsoluteAltitude": "+592.502", "XMP-drone-dji:RelativeAltitude": "+451.659",
    "XMP-drone-dji:GpsStatus": "RTK", "XMP-drone-dji:AltitudeType": "RtkAlt",
    "XMP-drone-dji:RtkFlag": 50, "XMP-drone-dji:RtkStdLon": 0.00266,
    "XMP-drone-dji:RtkStdLat": 0.00269, "XMP-drone-dji:RtkStdHgt": 0.00722,
    "ExifIFD:DateTimeOriginal": "2025:05:20 10:12:34",
    "XMP-drone-dji:UTCAtExposure": "2025:05:20 02:12:54.019996",
    "XMP-drone-dji:FlightYawDegree": "+73.00", "XMP-drone-dji:FlightPitchDegree": -9.70,
    "XMP-drone-dji:FlightRollDegree": "+11.20",
    "XMP-drone-dji:GimbalYawDegree": "+28.80", "XMP-drone-dji:GimbalPitchDegree": -60.00,
    "XMP-drone-dji:GimbalRollDegree": "+0.00",
    "ExifIFD:FocalLength": 12.29, "ExifIFD:FocalLengthIn35mmFormat": 24,
    "XMP-drone-dji:CalibratedFocalLength": 3725.151611,
    "XMP-drone-dji:CalibratedOpticalCenterX": 2640.0,
    "XMP-drone-dji:CalibratedOpticalCenterY": 1978.0,
    "XMP-drone-dji:DewarpFlag": 0,
    "File:ImageWidth": 5280, "File:ImageHeight": 3956,
    "XMP-drone-dji:LRFStatus": "Normal", "XMP-drone-dji:LRFTargetDistance": 140.577,
    "XMP-drone-dji:LRFTargetAbsAlt": 469.3, "XMP-drone-dji:LRFTargetLat": 24.0005,
    "XMP-drone-dji:LRFTargetLon": 121.0003,
}


def test_record_to_metadata_dji():
    m = record_to_metadata(DJI_RECORD, Path("a/x.JPG"), "a/x.JPG")
    assert m.photo_id == "a/x.JPG" and m.filename == "x.JPG"
    assert (m.make, m.model, m.product_name, m.image_source) == (
        "DJI", "M4E", "DJI Matrice 4E", "WideCamera")
    assert (m.latitude, m.longitude) == (24.0, 121.0) and m.has_gps
    assert m.gps_altitude == 592.502 and m.absolute_altitude == 592.502
    assert m.relative_altitude == 451.659
    assert m.agl is None and m.ground_elevation is None
    assert m.absolute_altitude_datum is AltitudeDatum.UNKNOWN
    assert (m.rtk_flag, m.rtk_std_lon, m.rtk_std_lat, m.rtk_std_hgt) == (50, 0.00266, 0.00269, 0.00722)
    assert m.capture_time == datetime(2025, 5, 20, 10, 12, 34)
    assert m.capture_time_utc == datetime(2025, 5, 20, 2, 12, 54, 19996)
    assert (m.flight_yaw, m.flight_pitch, m.flight_roll) == (73.0, -9.7, 11.2)
    assert (m.gimbal_yaw, m.gimbal_pitch, m.gimbal_roll) == (28.8, -60.0, 0.0)
    assert m.has_gimbal_pose and m.lrf_valid
    assert (m.image_width, m.image_height) == (5280, 3956)


def test_focal_lengths_never_merged():
    m = record_to_metadata(DJI_RECORD, Path("x.JPG"), "x.JPG")
    assert m.focal_length_mm == 12.29                     # physical (mm)
    assert m.focal_length_35mm == 24.0                    # 35 mm equivalent
    assert m.calibrated_focal_length == 3725.151611       # pixels
    assert (m.calibrated_optical_center_x, m.calibrated_optical_center_y) == (2640.0, 1978.0)


def test_provenance_records_source_tags():
    m = record_to_metadata(DJI_RECORD, Path("x.JPG"), "x.JPG")
    assert m.sources["relative_altitude"] == "XMP-drone-dji:RelativeAltitude"
    assert m.sources["calibrated_focal_length"] == "XMP-drone-dji:CalibratedFocalLength"
    assert m.sources["gimbal_yaw"] == "XMP-drone-dji:GimbalYawDegree"
    assert m.sources["latitude"] == "Composite:GPSLatitude"
    assert m.sources["image_width"] == "File:ImageWidth"


def test_record_minimal_all_optional():
    m = record_to_metadata({"File:FileType": "JPEG"}, Path("x.jpg"), "x.jpg")
    assert not m.has_gps and m.capture_time is None and m.gimbal_pitch is None
    assert not m.has_gimbal_pose and not m.lrf_valid and m.sources == {}


def test_absolute_altitude_not_filled_from_gps_altitude():
    m = record_to_metadata({"GPS:GPSAltitude": 12.5}, Path("x.jpg"), "x.jpg")
    assert m.gps_altitude == 12.5 and m.absolute_altitude is None


def test_gps_altitude_below_sea_level():
    m = record_to_metadata({"GPS:GPSAltitude": 3.0, "GPS:GPSAltitudeRef": 1}, Path("x"), "x")
    assert m.gps_altitude == -3.0


def test_image_size_fallback_tag():
    m = record_to_metadata({"ExifIFD:ExifImageWidth": 10, "ExifIFD:ExifImageHeight": 8},
                           Path("x"), "x")
    assert (m.image_width, m.image_height) == (10, 8)
    assert m.sources["image_width"] == "ExifIFD:ExifImageWidth"


@pytest.mark.parametrize("lat, lon, ok", [
    (24.8, 121.5, True), (-33.9, -70.6, True),
    (0, 0, False), (91, 121, False), (24, 181, False), ("24", 121, False), (None, None, False),
])
def test_parse_gps(lat, lon, ok):
    rec = {"Composite:GPSLatitude": lat, "Composite:GPSLongitude": lon}
    assert (parse_gps(rec) is not None) is ok


def test_find_photos_recursive(tmp_path):
    for rel in ("a/1.JPG", "a/b/2.jpeg", "a/b/c/子資料夾 中文/3.jpg", "a/4.JPEG"):
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b"x")
    (tmp_path / "a" / "v.MP4").write_bytes(b"x")
    (tmp_path / "a" / "r.DNG").write_bytes(b"x")
    assert len(find_photos(tmp_path)) == 4


def test_find_photos_errors(tmp_path):
    with pytest.raises(PhotoSourceError):
        find_photos(tmp_path / "missing")
    with pytest.raises(PhotoSourceError):
        find_photos(tmp_path)


def test_exiftool_not_found_clear_error(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path))
    monkeypatch.delenv("UAV_EXIFTOOL_PATH", raising=False)
    adapter = ExifToolAdapter(paths=ProjectPaths(tmp_path))
    with pytest.raises(ExifToolNotFoundError, match="project-local"):
        adapter.locate()


def test_exiftool_explicit_bad_path(tmp_path):
    with pytest.raises(ExifToolNotFoundError, match="explicit"):
        ExifToolAdapter(tmp_path / "nope.exe").locate()


def test_exiftool_env_fallback(tmp_path, monkeypatch, exiftool_exe):
    monkeypatch.setenv("PATH", str(tmp_path))
    monkeypatch.setenv("UAV_EXIFTOOL_PATH", str(exiftool_exe))
    assert ExifToolAdapter(paths=ProjectPaths(tmp_path)).locate() == str(exiftool_exe)


def test_exiftool_read_synthetic(tmp_path, exiftool_exe):
    root = tmp_path / "照片 root"
    good = write_jpg(root / "A" / "GPS.JPG", X0, Y0)
    nogps = write_jpg(root / "B" / "NOGPS.jpg")
    broken = root / "B" / "BROKEN.JPG"
    broken.write_bytes(b"not a jpeg")

    res = ExifToolAdapter(exiftool_exe).read([good, nogps, broken], photo_root=root)
    by_id = {m.photo_id: m for m in res.photos}
    assert set(by_id) == {"A/GPS.JPG", "B/NOGPS.jpg"}
    lon, lat = to_lonlat(X0, Y0)
    assert by_id["A/GPS.JPG"].latitude == pytest.approx(lat, abs=1e-6)
    assert by_id["A/GPS.JPG"].longitude == pytest.approx(lon, abs=1e-6)
    assert by_id["A/GPS.JPG"].make == "DJI"
    assert by_id["A/GPS.JPG"].image_width == 16
    assert not by_id["B/NOGPS.jpg"].has_gps
    assert [p for p, _ in res.errors] == [broken]
