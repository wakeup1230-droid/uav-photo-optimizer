"""
Shared synthetic test data. Nothing here touches the real input/ or output/.

Test AOI: a 200 m x 200 m square in EPSG:3826 (central Taiwan).
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import geopandas as gpd
import pytest
from pyproj import Transformer
from shapely.geometry import box

from uav_photo_optimizer.core.exceptions import ExifToolNotFoundError
from uav_photo_optimizer.metadata.exiftool import ExifToolAdapter

X0, Y0, X1, Y1 = 305000.0, 2655000.0, 305200.0, 2655200.0   # synthetic site
SQUARE = box(X0, Y0, X1, Y1)
TO_WGS84 = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)


def east_of_square(distance: float) -> tuple[float, float]:
    """EPSG:3826 point ``distance`` m east of the square's right-edge midpoint."""
    return X1 + distance, (Y0 + Y1) / 2


def to_lonlat(x: float, y: float) -> tuple[float, float]:
    return TO_WGS84.transform(x, y)


def make_project(tmp_path: Path, name: str = "專案 測試") -> Path:
    """Project root with CJK + space in its name, same layout as the real project."""
    base = tmp_path / name
    for d in ("input/shp", "input/photo", "output"):
        (base / d).mkdir(parents=True)
    return base


def write_shp(base: Path, geom=SQUARE, crs: str = "EPSG:3826", stem: str = "area") -> Path:
    gdf = gpd.GeoDataFrame({"id": [1]}, geometry=[geom], crs="EPSG:3826").to_crs(crs)
    path = base / "input" / "shp" / f"{stem}.shp"
    gdf.to_file(path)
    return path


def _dms(value: float):
    value = abs(value)
    d = int(value)
    m = int((value - d) * 60)
    s = (value - d - m / 60) * 3600
    return (float(d), float(m), round(s, 6))


def write_jpg(path: Path, x: float | None = None, y: float | None = None) -> Path:
    """JPG; when EPSG:3826 (x, y) is given, the matching WGS84 GPS EXIF is written."""
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (16, 16), "white")
    exif = Image.Exif()
    exif[0x010F] = "DJI"
    exif[0x0110] = "TEST"
    if x is not None:
        lon, lat = to_lonlat(x, y)
        exif[0x8825] = {
            1: "N" if lat >= 0 else "S", 2: _dms(lat),
            3: "E" if lon >= 0 else "W", 4: _dms(lon),
        }
    img.save(path, "JPEG", exif=exif.tobytes())
    return path


def write_dji_jpg(path: Path, x: float, y: float, *, yaw=0.0, pitch=-90.0, roll=0.0,
                  relative_altitude=100.0, focal_px=8.0) -> Path:
    """16x16 JPG with GPS EXIF + DJI XMP pose (f = 8 px → 90° HFOV / VFOV)."""
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    lon, lat = to_lonlat(x, y)
    exif = Image.Exif()
    exif[0x010F], exif[0x0110] = "DJI", "SYNTH"
    exif[0x8825] = {1: "N", 2: _dms(lat), 3: "E", 4: _dms(lon)}
    attrs = {
        "GimbalYawDegree": f"{yaw:+.2f}", "GimbalPitchDegree": f"{pitch:+.2f}",
        "GimbalRollDegree": f"{roll:+.2f}", "RelativeAltitude": f"{relative_altitude:+.3f}",
        "CalibratedFocalLength": f"{focal_px}", "CalibratedOpticalCenterX": "8",
        "CalibratedOpticalCenterY": "8", "DewarpFlag": "0", "RtkFlag": "50",
    }
    desc = " ".join(f'drone-dji:{k}="{v}"' for k, v in attrs.items())
    xmp = ('<x:xmpmeta xmlns:x="adobe:ns:meta/"><rdf:RDF '
           'xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"><rdf:Description '
           'rdf:about="DJI Meta Data" xmlns:drone-dji="http://www.dji.com/drone-dji/1.0/" '
           f'{desc}/></rdf:RDF></x:xmpmeta>').encode()
    Image.new("RGB", (16, 16), "white").save(path, "JPEG", exif=exif.tobytes(), xmp=xmp)
    return path


def sha1(path: Path) -> str:
    return hashlib.sha1(path.read_bytes()).hexdigest()


@pytest.fixture(scope="session")
def exiftool_exe() -> Path:
    """A working ExifTool (project-local / PATH / env); tests needing it skip otherwise."""
    try:
        return Path(ExifToolAdapter().locate())
    except ExifToolNotFoundError:
        pytest.skip("ExifTool not available")


@pytest.fixture
def project(tmp_path) -> Path:
    base = make_project(tmp_path)
    write_shp(base)
    return base


# --- official pipeline (select_photos / REST API / GUI) ----------------------------------

@pytest.fixture
def dataset(tmp_path, monkeypatch):
    """Synthetic 3-strip mission; ExifTool read stubbed; the visual guard must not run."""
    from shapely.geometry import box

    from synthetic_flight import X0, Y0, lawnmower
    from uav_photo_optimizer.core.engine import UAVPhotoOptimizer
    from uav_photo_optimizer.metadata.exiftool import MetadataReadResult

    base = make_project(tmp_path)
    shp = write_shp(base, geom=box(X0 + 100, Y0 - 50, X0 + 500, Y0 + 170))
    photo_dir = base / "input" / "photo"
    photos = []
    for m in lawnmower(strips=3, photo_spacing=10):
        f = photo_dir / m.photo_id
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_bytes(m.photo_id.encode() * 50)
        photos.append(m.model_copy(update={"path": f}))

    def fake_read(self, files, photo_root=None, progress=None):
        if progress:
            progress(len(files), len(files))
        return MetadataReadResult(photos=photos)

    monkeypatch.setattr(ExifToolAdapter, "read", fake_read)

    def no_visual(*a, **k):
        raise AssertionError("visual guard must be OFF by default")

    monkeypatch.setattr(UAVPhotoOptimizer, "validate_selection", no_visual)
    return base, shp, photo_dir
