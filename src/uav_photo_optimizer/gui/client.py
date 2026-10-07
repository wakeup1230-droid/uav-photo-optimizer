"""
Minimal REST API v1 client (standard library only) used by the GUI.

The GUI never calls the algorithm directly: Core Engine → REST API → GUI.
``start_local_server()`` runs the API in a background thread on 127.0.0.1 (free port) when no
external server URL is given (env ``UAV_API_URL``).
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from typing import Optional

ENV_API_URL = "UAV_API_URL"


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


class ApiClient:
    def __init__(self, base_url: str, timeout: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def _call(self, method: str, path: str, body: Optional[dict] = None):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(f"{self.base_url}/api/v1{path}", data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            try:
                detail = json.loads(exc.read().decode("utf-8")).get("detail", exc.reason)
            except Exception:  # noqa: BLE001
                detail = exc.reason
            if isinstance(detail, list):        # FastAPI 422 validation errors
                detail = "; ".join(str(d.get("msg", d)) for d in detail)
            raise ApiError(exc.code, str(detail)) from exc
        except urllib.error.URLError as exc:
            raise ApiError(0, f"無法連線到 API：{exc.reason}") from exc

    def health(self) -> dict:
        return self._call("GET", "/health")

    def create_job(self, request: dict) -> dict:
        return self._call("POST", "/jobs", request)

    def job(self, job_id: str) -> dict:
        return self._call("GET", f"/jobs/{job_id}")

    def result(self, job_id: str) -> dict:
        return self._call("GET", f"/jobs/{job_id}/result")["result"]


class LocalServer:
    """The REST API in a daemon thread on 127.0.0.1."""

    def __init__(self, port: Optional[int] = None, base_dir=None):
        import uvicorn

        from ..api.app import create_app
        from ..api.jobs import JobManager

        self.port = port or _free_port()
        config = uvicorn.Config(create_app(JobManager(base_dir=base_dir)), host="127.0.0.1",
                                port=self.port, log_level="warning")
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, daemon=True,
                                        name="uav-api-server")

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def start(self, timeout: float = 15.0) -> "LocalServer":
        self._thread.start()
        t0 = time.time()
        while not self._server.started:
            if time.time() - t0 > timeout or not self._thread.is_alive():
                raise RuntimeError("local API server did not start")
            time.sleep(0.05)
        return self

    def stop(self) -> None:
        self._server.should_exit = True
        self._thread.join(timeout=5)


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def connect(base_dir=None) -> tuple[ApiClient, Optional[LocalServer]]:
    """Client for ``UAV_API_URL`` if set, otherwise for a freshly started local server."""
    url = os.environ.get(ENV_API_URL)
    if url:
        return ApiClient(url), None
    server = LocalServer(base_dir=base_dir).start()
    return ApiClient(server.url), server
