"""Core contract: configuration, result, exceptions and the engine."""

from .config import (AOIConfig, CandidateSelectionMode, OptimizerConfig, RunConfig)
from .exceptions import UAVPhotoOptimizerError
from .result import PhotoReason, PhotoRecord, SelectionResult
from .engine import UAVPhotoOptimizer, run
from .selection import (AdvancedOptions, PhotoItem, SelectionRequest, SelectionSummary,
                        select_photos)

__all__ = ["AOIConfig", "CandidateSelectionMode", "OptimizerConfig", "RunConfig",
           "UAVPhotoOptimizerError", "PhotoReason", "PhotoRecord", "SelectionResult",
           "UAVPhotoOptimizer", "run", "AdvancedOptions", "PhotoItem", "SelectionRequest",
           "SelectionSummary", "select_photos"]
