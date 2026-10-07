"""
CameraProfileRegistry.

Policy: **metadata first, verified profile fallback second, unknown otherwise.**
Profiles only hold values with a documented source; everything else is None.
Lookup key: (make, model, image_source) — case-insensitive; image_source may be None.
"""

from __future__ import annotations

from typing import Iterable, Optional

from .models import CameraProfile

DJI_M4E_SPEC_URL = "https://enterprise.dji.com/matrice-4-series/specs"

DJI_M4E_WIDE = CameraProfile(
    make="DJI",
    model="M4E",
    camera_name="Wide camera",
    image_source="WideCamera",
    native_width_px=5280,
    native_height_px=3956,
    source=f"{DJI_M4E_SPEC_URL} (max image size 5280x3956); "
           "confirmed by File:ImageWidth/ImageHeight on real M4E photos",
    verified=True,
    notes=("DJI states 4/3 CMOS, FOV 84°, 24 mm equivalent, f/2.8-f/11. "
           "Sensor size in mm, physical focal length and whether 84° is diagonal are NOT "
           "officially stated, so sensor_*_mm / focal_length_mm / *_fov_deg are None. "
           "Per-photo metadata provides FocalLength 12.29 mm and CalibratedFocalLength "
           "3725.15 px, which the intrinsics resolver uses first."),
)


def _key(make: Optional[str], model: Optional[str], image_source: Optional[str]):
    norm = lambda s: s.strip().upper() if s else None   # noqa: E731
    return norm(make), norm(model), norm(image_source)


class CameraProfileRegistry:
    def __init__(self, profiles: Iterable[CameraProfile] = ()):
        self._profiles: dict[tuple, CameraProfile] = {}
        for p in profiles:
            self.register(p)

    def register(self, profile: CameraProfile) -> None:
        self._profiles[_key(profile.make, profile.model, profile.image_source)] = profile

    def get(self, make: Optional[str], model: Optional[str],
            image_source: Optional[str] = None) -> Optional[CameraProfile]:
        if not make or not model:
            return None
        return (self._profiles.get(_key(make, model, image_source))
                or self._profiles.get(_key(make, model, None)))

    def __len__(self) -> int:
        return len(self._profiles)


DEFAULT_REGISTRY = CameraProfileRegistry([DJI_M4E_WIDE])


def get_profile(make: Optional[str], model: Optional[str],
                image_source: Optional[str] = None) -> Optional[CameraProfile]:
    return DEFAULT_REGISTRY.get(make, model, image_source)
