"""
Synthetic lawn-mower missions on the TWD97 central meridian (grid north == true north).

Camera: 4000 x 3000 px, f = 2000 px, nadir, yaw = travel heading, height 100 m
→ footprint 200 m across-track × 150 m along-track (image top = forward).
Strips run east/west (90° / 270°), spaced ``strip_spacing`` m apart (north), photos every
``photo_spacing`` m, 1 s apart; turns are short north-going legs.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path

import math

from pyproj import Geod, Transformer

from uav_photo_optimizer.metadata.models import PhotoMetadata

X0, Y0 = 250000.0, 2655000.0          # EPSG:3826 on the central meridian (121°E)
TO_WGS = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)
GEOD = Geod(ellps="WGS84")
ABS_ALT = 500.0          # synthetic AbsoluteAltitude; LRF ground = ABS_ALT − height


def lrf_fields(lon, lat, height, pitch, yaw):
    """Consistent laser data: ray along the optical axis hits flat ground ``height`` below."""
    dep = math.radians(-pitch)
    horiz = 0.0 if abs(pitch) > 89.999 else height / math.tan(dep)
    tlon, tlat, _ = GEOD.fwd(lon, lat, yaw, horiz) if horiz else (lon, lat, 0)
    return dict(lrf_status="Normal", lrf_target_distance=math.hypot(height, horiz),
                lrf_target_lat=tlat, lrf_target_lon=tlon, lrf_target_abs_alt=ABS_ALT - height,
                absolute_altitude=ABS_ALT)


def lawnmower(*, strips=3, length=600.0, photo_spacing=30.0, strip_spacing=60.0,
              turn_photos=3, oblique_every=0, start=datetime(2026, 8, 1, 10, 0, 0),
              folder="M1", seq_start=1, height=100.0, x0=X0, y0=Y0,
              lrf=True) -> list[PhotoMetadata]:
    out, t, seq = [], start, seq_start

    def add(x, y, travel, pitch=-90.0, yaw=None):
        nonlocal t, seq
        lon, lat = TO_WGS.transform(x, y)
        g_yaw = travel if yaw is None else yaw
        extra = lrf_fields(lon, lat, height, pitch, g_yaw) if lrf else {}
        out.append(PhotoMetadata(
            photo_id=f"{folder}/DJI_{t:%Y%m%d%H%M%S}_{seq:04d}_V.JPG",
            filename=f"DJI_{t:%Y%m%d%H%M%S}_{seq:04d}_V.JPG", path=Path(f"{folder}/{seq}.JPG"),
            latitude=lat, longitude=lon, relative_altitude=height,
            capture_time=t, capture_time_utc=t, flight_yaw=travel,
            gimbal_yaw=g_yaw, gimbal_pitch=pitch, gimbal_roll=0.0,
            image_width=4000, image_height=3000, calibrated_focal_length=2000.0,
            calibrated_optical_center_x=2000.0, calibrated_optical_center_y=1500.0,
            dewarp_flag=1, rtk_flag=50, **extra))
        t += timedelta(seconds=1)
        seq += 1

    n = int(length // photo_spacing) + 1
    for k in range(strips):
        y = y0 + k * strip_spacing
        east = k % 2 == 0
        travel = 90.0 if east else 270.0
        for i in range(n):
            x = x0 + (i * photo_spacing if east else length - i * photo_spacing)
            add(x, y, travel)
            if oblique_every and i % oblique_every == 0:
                add(x, y, travel, pitch=-60.0, yaw=(travel - 45.0) % 360)
        if k < strips - 1:
            xe = x0 + (length if east else 0.0)
            for j in range(1, turn_photos + 1):
                add(xe + (10 if east else -10) * (1 if j < turn_photos else 0),
                    y + strip_spacing * j / (turn_photos + 1), 0.0)
    return out
