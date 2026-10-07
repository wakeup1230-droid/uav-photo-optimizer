"""
In-process job manager for the REST API.

One worker thread runs jobs one at a time (photo selection is I/O and CPU heavy, and
ExifTool / file copies should not compete). Jobs live in memory for the server's lifetime.
The algorithm is ``core.selection.select_photos`` — nothing is re-implemented here.
"""

from __future__ import annotations

import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from pydantic import ValidationError

from ..core.exceptions import ConfigError, ExportError, UAVPhotoOptimizerError
from ..core.selection import SelectionSummary, select_photos, validate_request
from .schemas import (CreateJobRequest, JobError, JobProgress, JobResponse, JobStatus,
                      SelectionResultV1)

Runner = Callable[..., SelectionSummary]


@dataclass
class Job:
    job_id: str
    request: CreateJobRequest
    status: JobStatus = JobStatus.QUEUED
    progress: JobProgress = field(default_factory=JobProgress)
    created_at: datetime = field(default_factory=datetime.now)
    updated_at: datetime = field(default_factory=datetime.now)
    error: Optional[JobError] = None
    result: Optional[SelectionResultV1] = None

    def response(self) -> JobResponse:
        return JobResponse(job_id=self.job_id, status=self.status,
                           progress=self.progress.model_copy(), created_at=self.created_at,
                           updated_at=self.updated_at, request=self.request, error=self.error)


class InvalidJobRequest(Exception):
    def __init__(self, message: str):
        super().__init__(message)
        self.message = message


class JobManager:
    def __init__(self, runner: Runner = select_photos, base_dir: Optional[Path] = None):
        self._runner = runner
        self._base_dir = base_dir
        self._jobs: dict[str, Job] = {}
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="uav-job")

    def submit(self, request: CreateJobRequest) -> Job:
        try:
            validate_request(request.to_selection_request())
        except (ConfigError, ExportError, ValidationError) as exc:
            raise InvalidJobRequest(str(exc)) from exc
        job = Job(job_id=uuid.uuid4().hex, request=request)
        with self._lock:
            self._jobs[job.job_id] = job
        self._pool.submit(self._run, job)
        return job

    def get(self, job_id: str) -> Optional[Job]:
        with self._lock:
            return self._jobs.get(job_id)

    def list(self) -> list[Job]:
        with self._lock:
            return sorted(self._jobs.values(), key=lambda j: j.created_at, reverse=True)

    def shutdown(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)

    # -- worker ------------------------------------------------------------------------

    def _update(self, job: Job, **changes) -> None:
        with self._lock:
            for k, v in changes.items():
                setattr(job, k, v)
            job.updated_at = datetime.now()

    def _run(self, job: Job) -> None:
        self._update(job, status=JobStatus.RUNNING)

        def progress(stage: str, done: int, total: int) -> None:
            self._update(job, progress=JobProgress(stage=stage, done=done, total=total))

        try:
            s = self._runner(job.request.to_selection_request(), progress=progress,
                             base_dir=self._base_dir)
            result = SelectionResultV1(**s.model_dump(include=set(SelectionResultV1.model_fields)))
            self._update(job, status=JobStatus.SUCCEEDED, result=result)
        except (ConfigError, ValidationError) as exc:
            self._update(job, status=JobStatus.FAILED,
                         error=JobError(code="INVALID_INPUT", message=str(exc)))
        except ExportError as exc:
            self._update(job, status=JobStatus.FAILED,
                         error=JobError(code="EXPORT_ERROR", message=str(exc)))
        except UAVPhotoOptimizerError as exc:
            self._update(job, status=JobStatus.FAILED,
                         error=JobError(code="PROCESSING_ERROR", message=str(exc)))
        except Exception as exc:  # noqa: BLE001 - reported to the client, server keeps running
            self._update(job, status=JobStatus.FAILED,
                         error=JobError(code="INTERNAL_ERROR",
                                        message=f"{type(exc).__name__}: {exc}"))
