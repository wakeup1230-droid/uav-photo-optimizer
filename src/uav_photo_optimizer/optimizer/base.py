"""Photo selection optimizer interface (Phase 6A: ``GreedyFreeDPOptimizer`` in planner.py)."""

from __future__ import annotations

from abc import ABC, abstractmethod

from shapely.geometry.base import BaseGeometry

from .models import SelectionOptimizerConfig, SelectionPlan


class PhotoSelector(ABC):
    @abstractmethod
    def plan(self, geometry, area: BaseGeometry,
             config: SelectionOptimizerConfig) -> SelectionPlan:
        """Produce a dry-run SelectionPlan from a GeometryAnalysis and the coverage area."""
