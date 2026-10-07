# Changelog

All notable changes to this project are documented here.
Format: [Keep a Changelog](https://keepachangelog.com/), versioning: [SemVer](https://semver.org/).

## [1.0.0] — 2026-10-07

Initial public release.

### Added
- AOI Shapefile + buffer photo selection (CRS handled automatically).
- Recursive photo search; batched ExifTool metadata reading with DJI XMP normalisation.
- PLANAR estimated ground footprints: gimbal pose, lens distortion (DJI DewarpData),
  laser-rangefinder height with a quality gate and neighbour interpolation.
- Flight / flight-strip detection.
- Front / side overlap (nadir) and along / cross-track geometric overlap (oblique).
- Photo selection optimizer: removes photos only while the front / side targets, oblique
  geometric overlap and AOI coverage stay satisfied; never enlarges an existing gap.
- Safe COPY export into a new `run_YYYYMMDD_HHMMSS/` folder with manifest and summary.
- Official pipeline `select_photos(SelectionRequest)`; CLI `select`.
- REST API v1 (FastAPI): `/health`, `POST /jobs`, `GET /jobs`, `GET /jobs/{id}`,
  `GET /jobs/{id}/result`; CLI `serve`.
- Simple desktop GUI (tkinter) on top of the REST API; CLI `gui`.
- Experimental, optional, default OFF: Visual Guard (OpenCV), terrain-aware footprints
  (local GeoTIFF, reference ray solver, weitsicht 0.0.4 backend).

Photo-selection results are geometric recommendations and do not guarantee successful
photogrammetric reconstruction.
