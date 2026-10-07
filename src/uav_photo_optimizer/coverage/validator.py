"""Coverage Guard interface. Planned (Phase 7) — no implementation yet."""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..aoi.buffer import AOIBuffer
from ..footprint.models import PhotoFootprint


class CoverageValidator(ABC):
    @abstractmethod
    def validate(self, aoi: AOIBuffer, selected: list[PhotoFootprint]) -> bool:
        """True when the selected photos still fully cover the AOI."""
