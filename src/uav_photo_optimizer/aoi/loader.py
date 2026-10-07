"""Locate and load the AOI shapefile (read only)."""

from __future__ import annotations

from pathlib import Path

import geopandas as gpd

from ..core.exceptions import AOIError


def find_shapefile(shp_dir: Path) -> Path:
    """Return the single .shp in ``shp_dir``; zero or several is an error."""
    if not shp_dir.is_dir():
        raise AOIError(f"找不到 SHP 資料夾：{shp_dir}")
    shp_files = sorted(p for p in shp_dir.iterdir()
                       if p.is_file() and p.suffix.lower() == ".shp")
    if not shp_files:
        raise AOIError(f"SHP 資料夾內找不到 .shp 檔案：{shp_dir}")
    if len(shp_files) > 1:
        names = "\n  ".join(p.name for p in shp_files)
        raise AOIError(f"SHP 資料夾僅允許放置一個 SHP，目前找到 {len(shp_files)} 個：\n  {names}")
    return shp_files[0]


def _sidecar_exists(shp_path: Path, suffix: str) -> bool:
    return any(p.is_file() and p.stem == shp_path.stem and p.suffix.lower() == suffix
               for p in shp_path.parent.iterdir())


def load_shapefile(shp_path: Path, warnings: list[str] | None = None) -> gpd.GeoDataFrame:
    """
    Read the shapefile, drop empty geometries and repair invalid ones (in memory only).
    Non-fatal issues are appended to ``warnings``.
    """
    warnings = warnings if warnings is not None else []
    if not shp_path.is_file():
        raise AOIError(f"找不到 SHP 檔案：{shp_path}")
    for suffix in (".shx", ".dbf"):
        if not _sidecar_exists(shp_path, suffix):
            raise AOIError(f"缺少 SHP 必要附屬檔案：{shp_path.with_suffix(suffix).name}")
    if not _sidecar_exists(shp_path, ".prj"):
        raise AOIError(f"缺少 .prj 檔案，無法得知座標系統：{shp_path.with_suffix('.prj').name}")

    try:
        gdf = gpd.read_file(shp_path)
    except Exception as exc:
        raise AOIError(f"SHP 無法讀取（可能損壞或缺少 .shx / .dbf）：{exc}") from exc

    if gdf.crs is None:
        raise AOIError("SHP 的 CRS 無法辨識（.prj 內容無效）")

    empty = gdf.geometry.isna() | gdf.geometry.is_empty
    if empty.all():
        raise AOIError("SHP 內沒有任何有效的 Geometry（全部為空）")
    if empty.any():
        warnings.append(f"略過 {int(empty.sum())} 筆空的 Geometry")
        gdf = gdf[~empty].copy()

    invalid = ~gdf.geometry.is_valid
    if invalid.any():
        warnings.append(f"發現 {int(invalid.sum())} 筆無效 Geometry，已使用 make_valid 修復")
        gdf.loc[invalid, "geometry"] = gdf.geometry[invalid].make_valid()

    return gdf
