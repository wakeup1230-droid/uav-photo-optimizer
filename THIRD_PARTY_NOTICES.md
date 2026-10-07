# Third-Party Notices

UAV Photo Optimizer itself is licensed under the Apache License 2.0 (see `LICENSE`).
The components below are **not** covered by that license; each keeps its own license.

## ExifTool (external tool — not distributed with this repository)

- Author: Phil Harvey
- Website: https://exiftool.org
- License: same terms as Perl itself — the Artistic License or the GNU General Public License
  (https://dev.perl.org/licenses/)
- The Windows package also bundles a Perl runtime in `exiftool_files/`, which carries its own
  licenses (see the files inside that folder).

How it is used: UAV Photo Optimizer runs ExifTool as a separate process (`exiftool -json -n ...`)
to **read** photo metadata. Photos are never modified. The binary is not committed to this
repository; users install it themselves (see `tools/README.md`). `ExifToolAdapter` locates it
via an explicit path, `UAV_EXIFTOOL_PATH`, `tools/exiftool/`, or `PATH` (in that order).

## Python dependencies (installed via pip, not vendored)

| Package | License |
|---|---|
| GeoPandas | BSD-3-Clause |
| Shapely | BSD-3-Clause |
| pyproj | MIT |
| NumPy | BSD-3-Clause |
| Pydantic | MIT |
| pytest (dev) | MIT |
| Pillow (dev) | MIT-CMU (HPND) |
| cameratransform (optional `footprint`) | MIT |
| opencv-python-headless (optional `visual`) | Apache-2.0 (OpenCV ≥ 4.5; wheels bundle third-party libraries under their own licenses) |
| FastAPI (optional `api` / `gui`) | MIT |
| Uvicorn (optional `api` / `gui`) | BSD-3-Clause |
| httpx (dev) | BSD-3-Clause |
| rasterio (optional `terrain`) | BSD-3-Clause (wheels bundle GDAL and its dependencies under their own licenses) |
| weitsicht (optional `terrain`, **pinned 0.0.4**, Alpha) | Apache-2.0; pulls in trimesh (MIT) and rtree (MIT) |

GeoPandas / Shapely / pyproj wheels bundle GDAL-related, GEOS and PROJ libraries under
their respective open-source licenses.

## Learned matchers (not used)

No learned feature matcher or pretrained weight is included. In particular, Magic Leap
SuperPoint weights (non-commercial licence) must never be bundled. See
`docs/research/LEARNED_MATCHER_OPTIONS.md`.

## Terrain data and geoid grids (not distributed)

No DEM, DSM or geoid / GTG grid is included. Terrain rasters and geoid grids are local files
supplied by the user at run time; national grids (e.g. Taiwan TWHYGEO2014, TWGEOID2018) are
subject to their publishers' terms and are never bundled. The `terrain/` code was written for
this project; no code was copied from weitsicht or from unlicensed sources
(`docs/research/EXISTING_CODE_REUSE.md`).
