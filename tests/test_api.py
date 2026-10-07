"""REST API v1 (FastAPI TestClient) on the synthetic mission."""

import time
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient  # noqa: E402

from uav_photo_optimizer import __version__  # noqa: E402
from uav_photo_optimizer.api.app import create_app  # noqa: E402
from uav_photo_optimizer.api.jobs import JobManager  # noqa: E402


def wait(client, job_id, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = client.get(f"/api/v1/jobs/{job_id}")
        if r.json()["status"] in ("succeeded", "failed"):
            return r.json()
        time.sleep(0.05)
    raise TimeoutError


@pytest.fixture
def client(dataset):
    base, shp, photo_dir = dataset
    with TestClient(create_app(JobManager(base_dir=base))) as c:
        yield c, base, shp, photo_dir


def test_health(client):
    c, *_ = client
    assert c.get("/api/v1/health").json() == {"status": "ok", "api_version": "v1",
                                              "tool_version": __version__}


def test_job_lifecycle_copy(client):
    c, base, shp, photo_dir = client
    out = base / "api_out"
    r = c.post("/api/v1/jobs", json={"aoi_shapefile": str(shp), "photo_dir": str(photo_dir),
                                     "output_dir": str(out), "buffer_m": 0,
                                     "front_overlap": 65, "side_overlap": 65})
    assert r.status_code == 201
    job = r.json()
    assert job["status"] in ("queued", "running", "succeeded")
    done = wait(c, job["job_id"])
    assert done["status"] == "succeeded", done
    res = c.get(f"/api/v1/jobs/{job['job_id']}/result").json()["result"]
    assert set(res) == {"candidate_photos", "selected_photos", "removed_photos",
                        "reduction_percent", "selected", "removed",
                        "original_overlap_below_target", "notes", "output_dir",
                        "copied_photos"}
    assert res["candidate_photos"] == res["selected_photos"] + res["removed_photos"]
    assert res["removed_photos"] > 0 and res["copied_photos"] == res["selected_photos"]
    assert len(list((Path(res["output_dir"]) / "photos").iterdir())) == res["selected_photos"]
    assert [j["job_id"] for j in c.get("/api/v1/jobs").json()] == [job["job_id"]]


def test_dry_run_job(client):
    c, base, shp, photo_dir = client
    r = c.post("/api/v1/jobs", json={"aoi_shapefile": str(shp), "photo_dir": str(photo_dir),
                                     "buffer_m": 0, "front_overlap": 65, "side_overlap": 65,
                                     "copy_photos": False})
    done = wait(c, r.json()["job_id"])
    res = c.get(f"/api/v1/jobs/{done['job_id']}/result").json()["result"]
    assert res["output_dir"] is None and res["copied_photos"] == 0


def test_validation_errors(client):
    c, base, shp, photo_dir = client
    body = {"aoi_shapefile": str(shp), "photo_dir": str(photo_dir), "output_dir": str(base / "o")}
    assert c.post("/api/v1/jobs", json={**body, "front_overlap": 60}).status_code == 422
    assert c.post("/api/v1/jobs", json={**body, "visual_guard": True}).status_code == 422
    r = c.post("/api/v1/jobs", json={**body, "aoi_shapefile": str(base / "missing.shp")})
    assert r.status_code == 400
    r = c.post("/api/v1/jobs", json={**body, "output_dir": str(photo_dir / "inside")})
    assert r.status_code == 400
    assert c.get("/api/v1/jobs/nope").status_code == 404
    assert c.get("/api/v1/jobs/nope/result").status_code == 404


def test_failed_job_reports_error(client, monkeypatch):
    c, base, shp, photo_dir = client

    def boom(*a, **k):
        raise RuntimeError("disk full")

    c.app.state.jobs._runner = boom
    r = c.post("/api/v1/jobs", json={"aoi_shapefile": str(shp), "photo_dir": str(photo_dir),
                                     "copy_photos": False})
    done = wait(c, r.json()["job_id"])
    assert done["status"] == "failed" and done["error"]["code"] == "INTERNAL_ERROR"
    assert c.get(f"/api/v1/jobs/{done['job_id']}/result").status_code == 409
