"""ExifTool resolution order: explicit → UAV_EXIFTOOL_PATH → project-local → PATH."""

import os
from pathlib import Path

import pytest

from uav_photo_optimizer.core.exceptions import ExifToolNotFoundError
from uav_photo_optimizer.metadata.exiftool import ENV_EXIFTOOL_PATH, ExifToolAdapter
from uav_photo_optimizer.paths import ProjectPaths

EXE = "exiftool.exe" if os.name == "nt" else "exiftool"


@pytest.fixture
def slots(tmp_path, monkeypatch):
    """Four fake executables, one per slot; _probe accepts any existing file."""
    monkeypatch.setattr(ExifToolAdapter, "_probe",
                        staticmethod(lambda exe: "99.0" if Path(exe).is_file() else None))
    project = ProjectPaths(tmp_path / "project")

    def make(path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fake")
        path.chmod(0o755)
        return path

    s = {
        "explicit": make(tmp_path / "explicit" / EXE),
        "env": make(tmp_path / "env" / EXE),
        "local": make(project.exiftool_path),
        "path": make(tmp_path / "onpath" / EXE),
    }
    monkeypatch.setenv(ENV_EXIFTOOL_PATH, str(s["env"]))
    monkeypatch.setenv("PATH", str(s["path"].parent))
    return project, s


def test_explicit_first(slots):
    project, s = slots
    assert ExifToolAdapter(s["explicit"], paths=project).locate() == str(s["explicit"])


def test_env_second(slots):
    project, s = slots
    assert ExifToolAdapter(paths=project).locate() == str(s["env"])


def test_project_local_third(slots, monkeypatch):
    project, s = slots
    monkeypatch.delenv(ENV_EXIFTOOL_PATH)
    assert ExifToolAdapter(paths=project).locate() == str(s["local"])


def test_system_path_fourth(slots, monkeypatch):
    project, s = slots
    monkeypatch.delenv(ENV_EXIFTOOL_PATH)
    s["local"].unlink()
    found = ExifToolAdapter(paths=project).locate()
    assert Path(found).resolve() == s["path"].resolve()


def test_env_set_but_missing_falls_through(slots, monkeypatch, tmp_path):
    project, s = slots
    monkeypatch.setenv(ENV_EXIFTOOL_PATH, str(tmp_path / "nope.exe"))
    assert ExifToolAdapter(paths=project).locate() == str(s["local"])


def test_explicit_bad_path_never_falls_back(slots, tmp_path):
    project, _ = slots
    with pytest.raises(ExifToolNotFoundError, match="explicit"):
        ExifToolAdapter(tmp_path / "missing.exe", paths=project).locate()


def test_nothing_found_lists_all_sources(slots, monkeypatch, tmp_path):
    project, s = slots
    monkeypatch.delenv(ENV_EXIFTOOL_PATH)
    s["local"].unlink()
    monkeypatch.setenv("PATH", str(tmp_path / "empty"))
    with pytest.raises(ExifToolNotFoundError) as exc:
        ExifToolAdapter(paths=project).locate()
    sources = [line.strip().split(":")[0] for line in str(exc.value).splitlines()[1:4]]
    assert sources == [ENV_EXIFTOOL_PATH, "project-local", "PATH"]


def test_path_and_env_not_modified(slots):
    project, _ = slots
    before = dict(os.environ)
    ExifToolAdapter(paths=project).locate()
    assert dict(os.environ) == before
