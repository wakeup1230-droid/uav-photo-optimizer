"""Visual connectivity validation (Phase 8A — Experimental). OpenCV lives behind an adapter."""

from .base import PhotoInput, ValidationContext, VisualValidator
from .models import (EdgeKind, VisualEdge, VisualMatchResult, VisualStatus, VisualThresholds,
                     VisualValidationConfig, VisualValidationReport)

__all__ = ["PhotoInput", "ValidationContext", "VisualValidator", "EdgeKind", "VisualEdge",
           "VisualMatchResult", "VisualStatus", "VisualThresholds", "VisualValidationConfig",
           "VisualValidationReport"]
