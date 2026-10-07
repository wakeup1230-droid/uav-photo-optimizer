"""
Normalization: ExifTool JSON (``-json -n -a -G1``) → PhotoMetadata.

TAG_MAP is the single, explicit mapping ``field ← Group:Tag`` with its parser.
DJI writes many XMP values as signed strings (``"+73.00"``); parsers handle that.
GPS position uses the Composite (signed) values — identical to Regression 001.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from .models import PhotoMetadata


def _float(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            return None
    return None


def _int(value: Any) -> Optional[int]:
    f = _float(value)
    return int(f) if f is not None else None


def _str(value: Any) -> Optional[str]:
    if value is None:
        return None
    s = str(value).strip()
    return s or None


def _datetime(value: Any) -> Optional[datetime]:
    if not isinstance(value, str):
        return None
    text = value.strip()
    for fmt in ("%Y:%m:%d %H:%M:%S.%f", "%Y:%m:%d %H:%M:%S"):
        try:
            return datetime.strptime(text[:26], fmt)
        except ValueError:
            continue
    return None


Parser = Callable[[Any], Any]

# field -> (Group:Tag, parser). Order inside a field list = priority (first present wins).
TAG_MAP: dict[str, list[tuple[str, Parser]]] = {
    "make":                        [("IFD0:Make", _str)],
    "model":                       [("IFD0:Model", _str)],
    "product_name":                [("XMP-drone-dji:ProductName", _str)],
    "image_source":                [("XMP-drone-dji:ImageSource", _str)],
    "relative_altitude":           [("XMP-drone-dji:RelativeAltitude", _float)],
    "absolute_altitude":           [("XMP-drone-dji:AbsoluteAltitude", _float)],
    "gps_status":                  [("XMP-drone-dji:GpsStatus", _str)],
    "altitude_type":               [("XMP-drone-dji:AltitudeType", _str)],
    "rtk_flag":                    [("XMP-drone-dji:RtkFlag", _int)],
    "rtk_std_lon":                 [("XMP-drone-dji:RtkStdLon", _float)],
    "rtk_std_lat":                 [("XMP-drone-dji:RtkStdLat", _float)],
    "rtk_std_hgt":                 [("XMP-drone-dji:RtkStdHgt", _float)],
    "capture_time":                [("ExifIFD:DateTimeOriginal", _datetime)],
    "capture_time_utc":            [("XMP-drone-dji:UTCAtExposure", _datetime)],
    "flight_yaw":                  [("XMP-drone-dji:FlightYawDegree", _float)],
    "flight_pitch":                [("XMP-drone-dji:FlightPitchDegree", _float)],
    "flight_roll":                 [("XMP-drone-dji:FlightRollDegree", _float)],
    "gimbal_yaw":                  [("XMP-drone-dji:GimbalYawDegree", _float)],
    "gimbal_pitch":                [("XMP-drone-dji:GimbalPitchDegree", _float)],
    "gimbal_roll":                 [("XMP-drone-dji:GimbalRollDegree", _float)],
    "focal_length_mm":             [("ExifIFD:FocalLength", _float)],
    "focal_length_35mm":           [("ExifIFD:FocalLengthIn35mmFormat", _float)],
    "calibrated_focal_length":     [("XMP-drone-dji:CalibratedFocalLength", _float)],
    "calibrated_optical_center_x": [("XMP-drone-dji:CalibratedOpticalCenterX", _float)],
    "calibrated_optical_center_y": [("XMP-drone-dji:CalibratedOpticalCenterY", _float)],
    "dewarp_flag":                 [("XMP-drone-dji:DewarpFlag", _int)],
    "dewarp_data":                 [("XMP-drone-dji:DewarpData", _str)],
    "dewarp_data_k6":              [("XMP-drone-dji:DewarpDataK6", _str)],
    "image_width":                 [("File:ImageWidth", _int), ("ExifIFD:ExifImageWidth", _int)],
    "image_height":                [("File:ImageHeight", _int), ("ExifIFD:ExifImageHeight", _int)],
    "lrf_status":                  [("XMP-drone-dji:LRFStatus", _str)],
    "lrf_target_distance":         [("XMP-drone-dji:LRFTargetDistance", _float)],
    "lrf_target_abs_alt":          [("XMP-drone-dji:LRFTargetAbsAlt", _float)],
    "lrf_target_lat":              [("XMP-drone-dji:LRFTargetLat", _float)],
    "lrf_target_lon":              [("XMP-drone-dji:LRFTargetLon", _float)],
}

GPS_LAT_TAG = "Composite:GPSLatitude"
GPS_LON_TAG = "Composite:GPSLongitude"
GPS_ALT_TAG = "GPS:GPSAltitude"
GPS_ALT_REF_TAG = "GPS:GPSAltitudeRef"
FILETYPE_TAG = "File:FileType"
ERROR_TAG = "ExifTool:Error"

EXIFTOOL_TAGS: tuple[str, ...] = tuple(dict.fromkeys(
    ["-" + ERROR_TAG, "-" + FILETYPE_TAG, "-" + GPS_LAT_TAG, "-" + GPS_LON_TAG,
     "-" + GPS_ALT_TAG, "-" + GPS_ALT_REF_TAG]
    + ["-" + tag for entries in TAG_MAP.values() for tag, _ in entries]))


def parse_gps(record: dict) -> Optional[tuple[float, float]]:
    """Return (lon, lat) or None when the record has no usable GPS position."""
    lat = record.get(GPS_LAT_TAG)
    lon = record.get(GPS_LON_TAG)
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return None
    if lat == 0 and lon == 0:      # common "no fix" default
        return None
    return float(lon), float(lat)


def record_to_metadata(record: dict, path: Path, photo_id: str) -> PhotoMetadata:
    values: dict[str, Any] = {}
    sources: dict[str, str] = {}

    for field, entries in TAG_MAP.items():
        for tag, parser in entries:
            value = parser(record.get(tag))
            if value is not None:
                values[field] = value
                sources[field] = tag
                break

    gps = parse_gps(record)
    if gps:
        values["longitude"], values["latitude"] = gps
        sources["longitude"], sources["latitude"] = GPS_LON_TAG, GPS_LAT_TAG

    gps_alt = _float(record.get(GPS_ALT_TAG))
    if gps_alt is not None:
        if _int(record.get(GPS_ALT_REF_TAG)) == 1:      # 1 = below sea level
            gps_alt = -gps_alt
        values["gps_altitude"] = gps_alt
        sources["gps_altitude"] = GPS_ALT_TAG

    return PhotoMetadata(photo_id=photo_id, filename=path.name, path=path,
                         sources=sources, **values)
