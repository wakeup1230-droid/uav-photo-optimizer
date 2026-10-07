# Tech Stack

狀態：**In use** = 正式版使用中；**Optional / Experimental** = 選用、預設關閉；**Future** = 未導入。

| 類別 | 技術 | 用途 | 狀態 |
|---|---|---|---|
| Core | Python 3.12 | 執行環境（`requires-python >= 3.12`） | In use |
| Core | GeoPandas | SHP 讀取、CRS 轉換 | In use |
| Core | Shapely 2 | Buffer、`covers` 空間判定 | In use |
| Core | pyproj | CRS 判定、UTM 推估 | In use |
| Core | NumPy | 數值運算 | In use |
| Core | Pydantic 2 | Data models、統一參數驗證、API schema | In use |
| Metadata | ExifTool | GPS / DJI XMP / EXIF 批次讀取 | In use |
| Footprint | PlanarProjector（NumPy，自行實作） | 預設 Footprint projector（v1.0） | In use |
| Footprint | CameraTransform | 交叉驗證用 projector（optional extra `footprint`） | Optional |
| Lens model | DJI DewarpDataK6 / DewarpData（自行實作 OpenCV rational / Brown 模型） | 鏡頭畸變校正（K6 → Brown5 → Pinhole） | In use |
| Optimization | 自行實作 DP（graph / dynamic programming） | 照片減量 | In use |
| Visual Validation | OpenCV（opencv-python-headless，optional extra `visual`） | SIFT + FLANN + ratio test + USAC-MAGSAC（Essential / Fundamental）連通性驗證 | Experimental，預設 OFF |
| Terrain | Rasterio（optional extra `terrain`） | GeoTIFF DEM / DSM 讀取；`ReferenceRasterRaySolver`（自行實作 ray march + bisection） | Experimental，預設 OFF |
| Terrain | weitsicht **0.0.4（pinned，Alpha）**（optional extra `terrain`） | `MappingRaster` 經 `WeitsichtRasterTerrainProvider` adapter；交叉驗證 | Experimental，預設 OFF |
| Terrain | trimesh | Mesh ray casting（`MeshTerrainProvider` 預留） | Reserved |
| API | FastAPI（optional extra `api`） | REST API v1 | In use (v1.0) |
| API | Pydantic | Request / Response schema | In use |
| API | Uvicorn | ASGI server（`uav-photo-optimizer serve`） | In use (v1.0) |
| GUI | tkinter（標準函式庫） | 簡易 GUI，經 REST API | In use (v1.0) |
| Testing | pytest | 單元 / 整合測試（synthetic data） | In use |
| Testing | Pillow | 產生 synthetic 測試照片（dev only） | In use |
| CI | GitHub Actions | push / pull_request 執行 pytest | In use |
