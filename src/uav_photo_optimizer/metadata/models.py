"""
Normalized photo metadata.

Every value is an observation from one explicit source tag (see ``metadata/dji.py`` TAG_MAP).
Semantically different tags are never merged into one field:

* ``focal_length_mm``          ExifIFD:FocalLength              physical lens focal length (mm)
* ``focal_length_35mm``        ExifIFD:FocalLengthIn35mmFormat  35 mm *equivalent* (not physical)
* ``calibrated_focal_length``  XMP-drone-dji:CalibratedFocalLength  pinhole focal length (pixels)

``sources`` records, per filled field, the ``Group:Tag`` it came from (provenance).
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from .altitude import AltitudeDatum


class PhotoMetadata(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_id: str                  # stable id: path relative to the photo root, POSIX style
    filename: str
    path: Path

    make: Optional[str] = None
    model: Optional[str] = None
    product_name: Optional[str] = None      # e.g. "DJI Matrice 4E"
    image_source: Optional[str] = None      # DJI camera on multi-camera payloads, e.g. "WideCamera"

    latitude: Optional[float] = None        # WGS84 degrees (signed)
    longitude: Optional[float] = None

    gps_altitude: Optional[float] = None    # m, EXIF GPS (sign from GPSAltitudeRef)
    absolute_altitude: Optional[float] = None   # m, DJI AbsoluteAltitude
    absolute_altitude_datum: AltitudeDatum = AltitudeDatum.UNKNOWN
    relative_altitude: Optional[float] = None   # m above TAKE-OFF point (not AGL)
    ground_elevation: Optional[float] = None    # reserved: DEM / user (None from metadata)
    agl: Optional[float] = None                 # reserved: only with a provable source

    gps_status: Optional[str] = None        # e.g. "RTK"
    altitude_type: Optional[str] = None     # e.g. "RtkAlt"
    rtk_flag: Optional[int] = None          # 0 none, 16 single, 32–49 float, 50 fixed
    rtk_std_lon: Optional[float] = None     # m
    rtk_std_lat: Optional[float] = None
    rtk_std_hgt: Optional[float] = None

    capture_time: Optional[datetime] = None        # ExifIFD:DateTimeOriginal (local, naive)
    capture_time_utc: Optional[datetime] = None    # XMP-drone-dji:UTCAtExposure

    flight_yaw: Optional[float] = None      # degrees, DJI NED / ZYX
    flight_pitch: Optional[float] = None
    flight_roll: Optional[float] = None

    gimbal_yaw: Optional[float] = None      # degrees, DJI NED / ZYX (earth frame)
    gimbal_pitch: Optional[float] = None    # -90 = nadir, 0 = horizontal
    gimbal_roll: Optional[float] = None

    focal_length_mm: Optional[float] = None
    focal_length_35mm: Optional[float] = None
    calibrated_focal_length: Optional[float] = None       # px
    calibrated_optical_center_x: Optional[float] = None   # px
    calibrated_optical_center_y: Optional[float] = None   # px
    dewarp_flag: Optional[int] = None                      # 0 = image NOT undistorted
    dewarp_data: Optional[str] = None                      # raw DJI DewarpData string
    dewarp_data_k6: Optional[str] = None                   # raw DJI DewarpDataK6 string

    image_width: Optional[int] = None       # px (actual JPEG frame)
    image_height: Optional[int] = None

    lrf_status: Optional[str] = None        # laser range finder
    lrf_target_distance: Optional[float] = None   # m, slant range along boresight
    lrf_target_abs_alt: Optional[float] = None    # m, same datum as absolute_altitude
    lrf_target_lat: Optional[float] = None
    lrf_target_lon: Optional[float] = None

    sources: dict[str, str] = Field(default_factory=dict)   # field -> "Group:Tag"

    @property
    def has_gps(self) -> bool:
        return self.latitude is not None and self.longitude is not None

    @property
    def has_gimbal_pose(self) -> bool:
        return None not in (self.gimbal_yaw, self.gimbal_pitch, self.gimbal_roll)

    @property
    def lrf_valid(self) -> bool:
        return (self.lrf_status == "Normal" and self.lrf_target_abs_alt is not None
                and self.absolute_altitude is not None)
