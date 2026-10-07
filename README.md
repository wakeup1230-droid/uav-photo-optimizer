# UAV Photo Optimizer
## UAV 照片篩選工具

依指定 AOI、Buffer、航向重疊率與側向重疊率，自動篩選 UAV 建模照片，
以降低後續攝影測量處理的照片數量與運算負荷。

A lightweight UAV photo selection tool based on AOI and geometric image overlap.

> **聲明 / Scope disclaimer**
>
> 本工具依照片 Metadata、空間位置及幾何重疊關係提供 UAV 照片篩選建議。
> 實際攝影測量建模結果仍受影像品質、地形、影像紋理、拍攝姿態、曝光、控制點、
> 建模參數及建模軟體等因素影響。
> 本工具不保證篩選後影像一定可成功完成攝影測量建模。
> 使用者應自行確認最終建模成果。
>
> Photo-selection results are geometric recommendations and do not guarantee successful
> photogrammetric reconstruction.

## Features

| Feature | Status |
|---|---|
| AOI / Buffer Selection | Available |
| Photo Metadata | Available |
| Flight / Strip Detection | Available |
| Front / Side Overlap | Available |
| Oblique Geometric Overlap | Available |
| Photo Selection | Available |
| COPY Export | Available |
| REST API | Available |
| Simple GUI | Available |
| Visual Guard | Experimental / Optional |
| Terrain Raster | Experimental / Optional |

## 使用方式 / Usage

只需要六個參數：

| 參數 | 預設值 | 範圍 |
|---|---|---|
| 建模範圍（AOI Shapefile） | — | `.shp` |
| 照片資料夾 | — | 遞迴搜尋子資料夾 |
| Buffer | 100 m | ≥ 0 |
| 航向重疊率（Front overlap） | 80 % | 65 – 95 % |
| 側向重疊率（Side overlap） | 70 % | 65 – 95 % |
| 輸出資料夾 | — | 不可位於照片資料夾內 |

### GUI

```powershell
uav-photo-optimizer gui
```

```text
UAV 照片篩選工具
建模範圍      [ ................. ] [...]
照片資料夾    [ ................. ] [...]
Buffer        [ 100 ] m
航向重疊率    [ 80 ] %
側向重疊率    [ 70 ] %
輸出資料夾    [ ................. ] [...]
              [ 開始篩選 ]
```

完成畫面（範例數字）：

```text
篩選完成
候選照片：1200
保留照片：860
減少照片：340
減量比例：28.3%
```

若原始照片本身的重疊率就低於設定值，會多一行「部分原始照片重疊率低於設定值。」
工具會保留可用照片，不讓原有缺口因減量而擴大，但無法改善原始拍攝資料。

### CLI

```powershell
uav-photo-optimizer select --aoi D:\project\area.shp --photos D:\project\photos `
    --output D:\project\output --buffer 100 --front 80 --side 70
# 只看結果、不複製：--dry-run
```

### REST API

```powershell
uav-photo-optimizer serve        # http://127.0.0.1:8765/docs
```

```text
POST /api/v1/jobs
{"aoi_shapefile": "D:/project/area.shp", "photo_dir": "D:/project/photos",
 "output_dir": "D:/project/output", "buffer_m": 100, "front_overlap": 80, "side_overlap": 70}

GET /api/v1/jobs/{job_id}/result
{"result": {"candidate_photos": 1200, "selected_photos": 860, "removed_photos": 340,
            "reduction_percent": 28.3, "notes": [], "selected": [...], "removed": [...]}}
```

See [docs/API_DESIGN.md](docs/API_DESIGN.md).

### Python

```python
from uav_photo_optimizer import SelectionRequest, select_photos

s = select_photos(SelectionRequest(aoi_shapefile="D:/project/area.shp",
                                   photo_dir="D:/project/photos",
                                   output_dir="D:/project/output"))
print(s.candidate_photos, s.selected_photos, s.removed_photos, s.reduction_percent)
```

## 輸出 / Output

每次執行建立新資料夾 `輸出資料夾/run_YYYYMMDD_HHMMSS/`，永遠不覆蓋既有檔案，
原始照片不會被移動、刪除或修改：

| File | Content |
|---|---|
| `photos/` | recommended photos (copied) |
| `selection_manifest.csv` | decision (keep / remove) and reason for every candidate |
| `selection_plan.json` | full selection plan |
| `run_summary.json` | summary |

## Pipeline

```text
AOI → Buffer → Photo Metadata → PLANAR Estimated Footprint → Flight / Flight Strip
    → Front / Side / Oblique Geometric Overlap → Photo Selection Optimizer → COPY
```

* [docs/FOOTPRINT_METHOD.md](docs/FOOTPRINT_METHOD.md) — how photo footprints are estimated
* [docs/OVERLAP_METHOD.md](docs/OVERLAP_METHOD.md) — overlap definitions and photo reduction
* [docs/LIMITATIONS.md](docs/LIMITATIONS.md) — what the tool does not do

CRS, ExifTool, camera parameters, laser-rangefinder height and flight-strip detection are
handled automatically.

## 安裝 / Install

Requirements: Python 3.12+ and [ExifTool](tools/README.md) (in `tools/exiftool/` or on `PATH`).

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install ".[gui]"     # core + REST API + GUI
# core only (CLI / Python): pip install .
```

The core selection needs no OpenCV, DEM / DSM or research package.

## Experimental Features

Kept in the code base, but **optional, default OFF and not required for the normal
workflow**. They are not part of the GUI or REST API v1.

| Feature | Notes | Install |
|---|---|---|
| Visual Guard | OpenCV SIFT image-connectivity check (`uav-photo-optimizer optimize --visual`) | `.[visual]` |
| Terrain-aware footprint | local GeoTIFF DEM / DSM ray intersection (`uav-photo-optimizer analyse-terrain`, dry run) | `.[terrain]` |
| weitsicht backend | alternative terrain intersection engine (weitsicht 0.0.4) | `.[terrain]` |

## Development

```powershell
python -m pip install -e ".[dev]"
python -m pytest                 # synthetic data only
```

Architecture: Core Engine → REST API → GUI / external systems. All algorithms live in the
core; the GUI does not re-implement them.

Documents: [CHANGELOG](CHANGELOG.md) · [ROADMAP](docs/ROADMAP.md) ·
[API_DESIGN](docs/API_DESIGN.md) · [TECH_STACK](docs/TECH_STACK.md) ·
[THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md)

## License

Apache License 2.0 — see [LICENSE](LICENSE).
Third-party components keep their own licenses — see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
