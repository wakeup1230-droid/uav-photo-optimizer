"""
VisualValidator interface. The Core (optimizer / guard) depends only on this; concrete
validators live in their own modules and import their libraries lazily:

    OpenCVVisualValidator   (optional extra ``visual`` → opencv-python-headless)
    Future: LightGlueVisualValidator, COLMAPVisualValidator, ...
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from ..camera.models import CameraIntrinsics
from .models import VisualMatchResult, VisualValidationConfig


@dataclass
class PhotoInput:
    photo_id: str
    path: Path
    camera: Optional[CameraIntrinsics] = None     # for undistortion / Essential matrix


@dataclass
class ValidationContext:
    config: VisualValidationConfig = field(default_factory=VisualValidationConfig)


class VisualValidator(ABC):
    name: str = "abstract"

    @abstractmethod
    def validate_pair(self, photo_a: PhotoInput, photo_b: PhotoInput,
                      context: ValidationContext) -> VisualMatchResult:
        """Image-based connectivity of two photos (never modifies the images)."""
