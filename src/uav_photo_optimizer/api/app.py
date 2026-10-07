"""
REST API v1 (FastAPI). Optional extra: ``pip install 'uav-photo-optimizer[api]'``.

    uav-photo-optimizer serve            # http://127.0.0.1:8765/docs

Endpoints:

    GET  /api/v1/health
    POST /api/v1/jobs                    create a selection job (201)
    GET  /api/v1/jobs                    list jobs
    GET  /api/v1/jobs/{job_id}           status + progress
    GET  /api/v1/jobs/{job_id}/result    result (409 until the job succeeded)

Paths in requests refer to the machine running the server. The server binds to 127.0.0.1 by
default (local tool); it has no authentication.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from .. import __version__
from .jobs import InvalidJobRequest, JobManager
from .schemas import (API_VERSION, CreateJobRequest, HealthResponse, JobResponse,
                      JobResultResponse, JobStatus)

PREFIX = f"/api/{API_VERSION}"


def create_app(manager: Optional[JobManager] = None, base_dir: Optional[Path] = None):
    try:
        from fastapi import FastAPI, HTTPException
    except ImportError as exc:  # pragma: no cover - optional extra
        raise ImportError("the REST API needs FastAPI: pip install "
                          "'uav-photo-optimizer[api]'") from exc

    jobs = manager or JobManager(base_dir=base_dir)

    @asynccontextmanager
    async def lifespan(_app):
        yield
        jobs.shutdown()

    app = FastAPI(lifespan=lifespan, title="UAV Photo Optimizer API", version=__version__,
                  description="UAV 照片篩選工具 — select UAV photos by AOI, buffer and front / "
                              "side overlap, and copy the recommended photos. Geometric "
                              "selection recommendation only; modelling success is not "
                              "guaranteed.")
    app.state.jobs = jobs

    @app.get(f"{PREFIX}/health", response_model=HealthResponse)
    def health():
        return HealthResponse(tool_version=__version__)

    @app.post(f"{PREFIX}/jobs", response_model=JobResponse, status_code=201)
    def create_job(request: CreateJobRequest):
        try:
            return jobs.submit(request).response()
        except InvalidJobRequest as exc:
            raise HTTPException(status_code=400, detail=exc.message) from exc

    @app.get(f"{PREFIX}/jobs", response_model=list[JobResponse])
    def list_jobs():
        return [j.response() for j in jobs.list()]

    @app.get(f"{PREFIX}/jobs/{{job_id}}", response_model=JobResponse)
    def get_job(job_id: str):
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status_code=404, detail="job not found")
        return job.response()

    @app.get(f"{PREFIX}/jobs/{{job_id}}/result", response_model=JobResultResponse)
    def get_result(job_id: str):
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status_code=404, detail="job not found")
        if job.status is not JobStatus.SUCCEEDED or job.result is None:
            detail = job.error.message if job.error else f"job is {job.status.value}"
            raise HTTPException(status_code=409, detail=detail)
        return JobResultResponse(job_id=job.job_id, status=job.status, result=job.result)

    return app


def serve(host: str = "127.0.0.1", port: int = 8765) -> None:  # pragma: no cover
    try:
        import uvicorn
    except ImportError as exc:
        raise ImportError("the REST API needs FastAPI + Uvicorn: pip install "
                          "'uav-photo-optimizer[api]'") from exc
    uvicorn.run(create_app(), host=host, port=port, log_level="info")
