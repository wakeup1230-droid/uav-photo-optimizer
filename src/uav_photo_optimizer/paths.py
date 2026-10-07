"""
Project path resolution.

All default locations are derived from a single ``BASE_DIR`` (the project root),
so the project can be moved to another folder / drive / machine without editing code.

Resolution order for the project root:

1. Explicit ``base_dir`` argument
2. Environment variable ``UAV_PHOTO_OPTIMIZER_HOME``
3. The source checkout containing this package (``<root>/src/uav_photo_optimizer``)
4. Current working directory
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

ENV_PROJECT_HOME = "UAV_PHOTO_OPTIMIZER_HOME"
_PACKAGE_DIR = Path(__file__).resolve().parent


def _is_project_root(path: Path) -> bool:
    return (path / "pyproject.toml").is_file() and (path / "src" / "uav_photo_optimizer").is_dir()


def find_project_root(base_dir: str | os.PathLike | None = None) -> Path:
    if base_dir is not None:
        return Path(base_dir).resolve()
    env = os.environ.get(ENV_PROJECT_HOME)
    if env:
        return Path(env).resolve()
    checkout = _PACKAGE_DIR.parents[1]
    if _is_project_root(checkout):
        return checkout
    return Path.cwd().resolve()


@dataclass(frozen=True)
class ProjectPaths:
    """Standard project folder layout, all relative to ``base_dir``."""

    base_dir: Path

    @classmethod
    def discover(cls, base_dir: str | os.PathLike | None = None) -> "ProjectPaths":
        return cls(find_project_root(base_dir))

    @property
    def input_dir(self) -> Path:
        return self.base_dir / "input"

    @property
    def shp_dir(self) -> Path:
        return self.input_dir / "shp"

    @property
    def photo_dir(self) -> Path:
        return self.input_dir / "photo"

    @property
    def output_dir(self) -> Path:
        return self.base_dir / "output"

    @property
    def tools_dir(self) -> Path:
        return self.base_dir / "tools"

    @property
    def exiftool_dir(self) -> Path:
        return self.tools_dir / "exiftool"

    @property
    def exiftool_path(self) -> Path:
        name = "exiftool.exe" if os.name == "nt" else "exiftool"
        return self.exiftool_dir / name


BASE_DIR = find_project_root()
DEFAULT_PATHS = ProjectPaths(BASE_DIR)
