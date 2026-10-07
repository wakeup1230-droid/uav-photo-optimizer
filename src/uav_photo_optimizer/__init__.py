"""
UAV Photo Optimizer (UAV 照片篩選工具) — Core library.

    from uav_photo_optimizer import SelectionRequest, select_photos
    summary = select_photos(SelectionRequest(aoi_shapefile="area.shp", photo_dir="photos",
                                             output_dir="out", buffer_m=100,
                                             front_overlap=80, side_overlap=70))
"""

__version__ = "1.0.0"

from .core import (AOIConfig, AdvancedOptions, CandidateSelectionMode,  # noqa: E402
                   OptimizerConfig, PhotoItem, PhotoReason, PhotoRecord, RunConfig,
                   SelectionRequest, SelectionResult, SelectionSummary, UAVPhotoOptimizer,
                   UAVPhotoOptimizerError, run, select_photos)

__all__ = ["__version__", "AOIConfig", "CandidateSelectionMode", "OptimizerConfig",
           "PhotoReason", "PhotoRecord", "RunConfig", "SelectionResult",
           "UAVPhotoOptimizer", "UAVPhotoOptimizerError", "run", "AdvancedOptions",
           "PhotoItem", "SelectionRequest", "SelectionSummary", "select_photos"]
