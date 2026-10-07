# Public Release Checklist

Run before every public release. A release is made public only when every item passes.

## Content

- [ ] Licence: Apache-2.0 (`LICENSE`); third-party licences in `THIRD_PARTY_NOTICES.md`
- [ ] `README.md`: scope, disclaimer, feature status, usage
- [ ] `CHANGELOG.md` entry for the version; version in `pyproject.toml` and `__init__.py`
- [ ] `docs/API_DESIGN.md` matches the REST API

## Data and privacy

- [ ] No photos (`*.jpg`, `*.jpeg`, `*.dng`), no shapefiles (`*.shp`, `*.shx`, `*.dbf`,
      `*.prj`), no rasters (`*.tif`, `*.tiff`), no 3-D models (`*.3mx`)
- [ ] `input/` and `output/` contain only their README files
- [ ] No real coordinates, project names, flight dates, file names or case statistics; tests
      and examples use synthetic data only
- [ ] No personal paths, e-mail addresses, API keys, tokens or passwords
- [ ] No ExifTool binary or other third-party binaries
- [ ] Git history contains no earlier commit with sensitive content

## Build and tests

- [ ] `python -m build` produces sdist and wheel
- [ ] Clean virtual environment: `pip install .` → `uav-photo-optimizer --help`
- [ ] `pip install ".[api]"` → `uav-photo-optimizer serve` answers `/api/v1/health`
- [ ] `pip install ".[gui]"` → `uav-photo-optimizer gui` opens
- [ ] `pytest` passes on synthetic data (no real data required)
- [ ] GitHub Actions CI passes (tests, core-only, build)

## Publish

- [ ] Push to a **private** repository first and wait for CI
- [ ] Switch the repository to public
- [ ] Create the GitHub release with the disclaimer: photo-selection results are geometric
      recommendations and do not guarantee successful photogrammetric reconstruction
