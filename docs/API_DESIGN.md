# REST API v1

> This API provides geometric UAV photo-selection recommendations.
> It does not guarantee downstream photogrammetric reconstruction success.

Status: **Available (v1.0).** Server: `uav-photo-optimizer serve` (optional extra `api`),
default `http://127.0.0.1:8765`, OpenAPI docs at `/docs`. Code: `src/uav_photo_optimizer/api/`.

## Principles

* Core Engine → REST API → GUI / external systems. The API calls
  `core.selection.select_photos`; no algorithm is implemented in the API or the GUI.
* Core scope only: AOI source, photo source, output, buffer, front / side overlap.
  Research details (LRF, terrain, visual matching, lens model, ray solver) are **not** part of
  API v1.
* Validation limits are shared with Core / CLI / GUI (`core/config.py`): buffer ≥ 0,
  overlaps 65 – 95 %. Unknown fields are rejected (422).
* Paths refer to the machine running the server. The server binds to 127.0.0.1 by default
  and has no authentication (local tool).
* Jobs run one at a time in a background worker and are kept in memory while the server runs.
* Photos are only copied, into a new `output_dir/run_YYYYMMDD_HHMMSS/` folder; nothing is
  moved, deleted or overwritten.

## Endpoints

| Method | Path | Response |
|---|---|---|
| GET | `/api/v1/health` | `{"status": "ok", "api_version": "v1", "tool_version": "1.0.0"}` |
| POST | `/api/v1/jobs` | 201 `JobResponse`; 400 invalid input (missing file, output inside photo folder); 422 schema error |
| GET | `/api/v1/jobs` | list of `JobResponse`, newest first |
| GET | `/api/v1/jobs/{job_id}` | `JobResponse`; 404 unknown job |
| GET | `/api/v1/jobs/{job_id}/result` | `JobResultResponse`; 409 while queued / running or when the job failed |

### POST /api/v1/jobs — `CreateJobRequest`

```json
{
  "aoi_shapefile": "D:/project/area.shp",
  "photo_dir": "D:/project/photo",
  "output_dir": "D:/project/output",
  "buffer_m": 100,
  "front_overlap": 80,
  "side_overlap": 70,
  "copy_photos": true
}
```

| Field | Default | Rule |
|---|---|---|
| `aoi_shapefile` | — | existing `.shp` |
| `photo_dir` | — | existing folder, searched recursively |
| `output_dir` | null | required when `copy_photos` is true; not inside `photo_dir` |
| `buffer_m` | 100 | ≥ 0 |
| `front_overlap` | 80 | 65 – 95 |
| `side_overlap` | 70 | 65 – 95 |
| `copy_photos` | true | false = report only |

### `JobResponse`

```json
{
  "job_id": "3f2c…",
  "status": "running",
  "progress": {"stage": "metadata", "done": 500, "total": 1500},
  "created_at": "2026-10-07T18:00:00",
  "updated_at": "2026-10-07T18:00:20",
  "request": {"...": "..."},
  "error": null
}
```

`status`: `queued` → `running` → `succeeded` | `failed`.
`progress.stage`: `scan`, `metadata`, `geometry`, `optimize`, `copy`, `done`.
`error.code`: `INVALID_INPUT`, `EXPORT_ERROR`, `PROCESSING_ERROR`, `INTERNAL_ERROR`.

### GET /api/v1/jobs/{job_id}/result — `JobResultResponse`

```json
{
  "job_id": "3f2c…",
  "status": "succeeded",
  "result": {
    "candidate_photos": 1200,
    "selected_photos": 860,
    "removed_photos": 340,
    "reduction_percent": 28.3,
    "selected": [{"filename": "DJI_0001.JPG", "path": "D:/project/photo/…/DJI_0001.JPG"}],
    "removed": [{"filename": "DJI_0002.JPG", "path": "…"}],
    "original_overlap_below_target": false,
    "notes": [],
    "output_dir": "D:/project/output/run_20261007_180102",
    "copied_photos": 860
  }
}
```

* `selected_photos + removed_photos = candidate_photos`.
* `reduction_percent = removed / candidates × 100`.
* When the original photos already miss the target somewhere, `original_overlap_below_target`
  is true and `notes` contains 「部分原始照片重疊率低於設定值。」.
* `original_overlap_below_target`: some parts of the original photos already overlap less
  than requested. The selection keeps the usable photos but cannot improve the original
  capture.

## Internal models (not API v1)

`GeometryAnalysis`, `SelectionPlan` (decisions, baseline defects, safety report),
`VisualValidationReport` and `TerrainAnalysis` remain Python-level internal / debug models
(`UAVPhotoOptimizer.analyse_geometry()`, `plan_selection()`, `validate_selection()`,
`analyse_terrain()`). The full plan of every copy run is also written to
`selection_plan.json` in the run folder.
