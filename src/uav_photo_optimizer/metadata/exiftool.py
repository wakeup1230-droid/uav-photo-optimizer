"""
ExifToolAdapter: locate ExifTool and read photo metadata in batches (read only).

Resolution order:

1. Explicit path passed by the caller (constructor / ``RunConfig.exiftool_path``)
2. Environment variable ``UAV_EXIFTOOL_PATH``
3. Project-local ``tools/exiftool/exiftool.exe``
4. System ``PATH``

An explicit path is never silently replaced: if it is given but unusable, locate() fails.
The process environment (including PATH) is only read, never modified.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

from ..core.config import EXIFTOOL_BATCH_SIZE
from ..core.exceptions import ExifToolNotFoundError
from ..paths import DEFAULT_PATHS, ProjectPaths
from .dji import ERROR_TAG, EXIFTOOL_TAGS, FILETYPE_TAG, record_to_metadata
from .models import PhotoMetadata

ENV_EXIFTOOL_PATH = "UAV_EXIFTOOL_PATH"
_CREATIONFLAGS = getattr(subprocess, "CREATE_NO_WINDOW", 0)

ProgressCallback = Callable[[int, int], None]


@dataclass
class MetadataReadResult:
    photos: list[PhotoMetadata] = field(default_factory=list)         # readable JPEGs
    errors: list[tuple[Path, str]] = field(default_factory=list)      # (path, message)


def _path_key(path) -> str:
    return os.path.normcase(os.path.normpath(str(path)))


def photo_id_for(path: Path, root: Optional[Path]) -> str:
    if root is not None:
        try:
            return path.relative_to(root).as_posix()
        except ValueError:
            pass
    return path.as_posix()


class ExifToolAdapter:
    def __init__(self, exiftool_path: Optional[Path] = None,
                 paths: ProjectPaths = DEFAULT_PATHS,
                 batch_size: int = EXIFTOOL_BATCH_SIZE):
        self._explicit = Path(exiftool_path) if exiftool_path else None
        self._paths = paths
        self.batch_size = batch_size
        self._executable: Optional[str] = None
        self._version: Optional[str] = None

    # -- discovery ----------------------------------------------------------

    def candidates(self) -> list[tuple[str, Optional[str]]]:
        """(source label, path or None) in search order."""
        if self._explicit is not None:
            return [("explicit", str(self._explicit))]
        return [
            (ENV_EXIFTOOL_PATH, os.environ.get(ENV_EXIFTOOL_PATH)),
            ("project-local", str(self._paths.exiftool_path)),
            ("PATH", shutil.which("exiftool")),
        ]

    def locate(self) -> str:
        """Return a working ExifTool executable path; raise ExifToolNotFoundError otherwise."""
        if self._executable:
            return self._executable
        tried = []
        for source, candidate in self.candidates():
            if not candidate:
                tried.append(f"{source}: (not set)")
                continue
            if not Path(candidate).is_file():
                tried.append(f"{source}: {candidate} (not found)")
                continue
            version = self._probe(candidate)
            if version:
                self._executable, self._version = candidate, version
                return candidate
            tried.append(f"{source}: {candidate} (cannot execute)")
        raise ExifToolNotFoundError(
            "找不到可執行的 ExifTool。已搜尋：\n  " + "\n  ".join(tried) +
            f"\n請設定環境變數 {ENV_EXIFTOOL_PATH}，"
            "或將 exiftool.exe 與 exiftool_files 放入 tools/exiftool/，或加入 PATH。")

    @staticmethod
    def _probe(executable: str) -> Optional[str]:
        try:
            result = subprocess.run([executable, "-ver"], capture_output=True, timeout=60,
                                    creationflags=_CREATIONFLAGS)
        except Exception:
            return None
        version = result.stdout.decode("utf-8", errors="replace").strip()
        return version if result.returncode == 0 and version else None

    @property
    def version(self) -> str:
        self.locate()
        return self._version or ""

    # -- reading ------------------------------------------------------------

    def _run_batch(self, photos: list[Path]) -> dict[str, dict]:
        # One ExifTool process per batch (never per photo). Paths go via a UTF-8 argfile so
        # CJK / spaces are safe. -a -G1: keys are 'Group:Tag' for explicit normalization.
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", suffix=".args",
                                         delete=False) as f:
            for p in photos:
                f.write(str(p) + "\n")
            args_file = f.name
        try:
            cmd = [self.locate(), "-charset", "filename=utf8", "-json", "-n", "-a", "-G1", "-m",
                   *EXIFTOOL_TAGS, "-@", args_file]
            result = subprocess.run(cmd, capture_output=True, creationflags=_CREATIONFLAGS)
        finally:
            os.remove(args_file)
        stdout = result.stdout.decode("utf-8", errors="replace").strip()
        if not stdout:
            return {}
        return {_path_key(r.get("SourceFile", "")): r for r in json.loads(stdout)}

    def read(self, photos: list[Path], photo_root: Optional[Path] = None,
             progress: Optional[ProgressCallback] = None) -> MetadataReadResult:
        """Read metadata for ``photos``. Single-photo failures go to ``errors``, never raise."""
        self.locate()
        out = MetadataReadResult()
        total = len(photos)
        for start in range(0, total, self.batch_size):
            batch = photos[start:start + self.batch_size]
            try:
                records = self._run_batch(batch)
            except Exception as exc:
                out.errors.extend((p, f"ExifTool 執行失敗：{exc}") for p in batch)
                records = None
            if records is not None:
                for p in batch:
                    record = records.get(_path_key(p))
                    if record is None:
                        out.errors.append((p, "ExifTool 未回傳資料"))
                    elif ERROR_TAG in record:
                        out.errors.append((p, str(record[ERROR_TAG])))
                    elif record.get(FILETYPE_TAG) != "JPEG":
                        out.errors.append(
                            (p, f"不是有效的 JPEG（FileType={record.get(FILETYPE_TAG)}）"))
                    else:
                        out.photos.append(record_to_metadata(record, p, photo_id_for(p, photo_root)))
            if progress:
                progress(min(start + len(batch), total), total)
        return out
