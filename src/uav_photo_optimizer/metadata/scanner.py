"""Recursive photo discovery (read only)."""

from __future__ import annotations

from pathlib import Path

from ..core.config import PHOTO_EXTENSIONS
from ..core.exceptions import PhotoSourceError


def find_photos(photo_dir: Path) -> list[Path]:
    """All JPG / JPEG files under ``photo_dir`` (any depth), sorted."""
    if not photo_dir.is_dir():
        raise PhotoSourceError(f"找不到照片資料夾：{photo_dir}")
    photos = sorted(p for p in photo_dir.rglob("*")
                    if p.suffix.lower() in PHOTO_EXTENSIONS and p.is_file())
    if not photos:
        raise PhotoSourceError(f"照片資料夾內（含所有子資料夾）找不到任何 JPG 照片：{photo_dir}")
    return photos
