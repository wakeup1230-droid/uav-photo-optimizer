import pytest
from pydantic import ValidationError

from uav_photo_optimizer import OptimizerConfig, RunConfig
from uav_photo_optimizer.api.schemas import CreateJobRequest, JobResponse, JobStatus
from uav_photo_optimizer.core.config import MAX_OVERLAP, MIN_OVERLAP, AOIConfig
from uav_photo_optimizer.core.exceptions import ConfigError


def test_defaults():
    cfg = RunConfig.create()
    assert cfg.aoi.buffer_m == cfg.optimizer.buffer_m == 100
    assert cfg.optimizer.min_overlap == MIN_OVERLAP == 65
    assert MAX_OVERLAP == 95


@pytest.mark.parametrize("buffer_m", [0, 0.5, 100, 1000])
def test_buffer_valid(buffer_m):
    assert RunConfig.create(buffer_m=buffer_m).aoi.buffer_m == buffer_m


def test_buffer_negative_rejected():
    with pytest.raises(ConfigError):
        RunConfig.create(buffer_m=-0.1)
    with pytest.raises(ValidationError):
        AOIConfig(buffer_m=-1)


@pytest.mark.parametrize("value", [65, 80, 95])
def test_overlap_valid(value):
    cfg = RunConfig.create(front_overlap=value, side_overlap=value)
    assert cfg.optimizer.front_overlap_target == value
    assert cfg.optimizer.side_overlap_target == value


@pytest.mark.parametrize("field", ["front_overlap", "side_overlap"])
@pytest.mark.parametrize("value", [64.99, 0, 95.01, 100])
def test_overlap_out_of_range(field, value):
    with pytest.raises(ConfigError):
        RunConfig.create(**{field: value})


def test_validate_assignment():
    cfg = OptimizerConfig()
    with pytest.raises(ValidationError):
        cfg.front_overlap_target = 60


def test_buffer_must_match():
    with pytest.raises(ValidationError, match="buffer_m"):
        RunConfig(aoi=AOIConfig(buffer_m=50), optimizer=OptimizerConfig(buffer_m=100))


def test_unknown_field_rejected():
    with pytest.raises(ValidationError):
        OptimizerConfig(min_overlap=10)


# API schemas share the Core rules ------------------------------------------------

BASE = {"aoi_shapefile": "a.shp", "photo_dir": "photos"}


@pytest.mark.parametrize("kwargs", [{"buffer_m": -1}, {"front_overlap": 64}, {"side_overlap": 96}])
def test_api_request_same_validation(kwargs):
    with pytest.raises(ValidationError):
        CreateJobRequest(**BASE, **kwargs)


def test_api_request_defaults_and_conversion():
    req = CreateJobRequest(**BASE, output_dir="out")
    assert (req.buffer_m, req.front_overlap, req.side_overlap) == (100, 80, 70)
    sel = CreateJobRequest(**BASE, output_dir="out", buffer_m=50, front_overlap=85,
                           side_overlap=75).to_selection_request()
    assert (sel.buffer_m, sel.front_overlap, sel.side_overlap) == (50, 85, 75)
    assert str(sel.aoi_shapefile) == "a.shp" and sel.copy_photos
    with pytest.raises(ValidationError):
        CreateJobRequest(**BASE, lens_mode="K6")        # research options are not API v1


def test_api_job_response_roundtrip():
    from datetime import datetime
    now = datetime(2026, 10, 7)
    resp = JobResponse(job_id="abc", status=JobStatus.QUEUED, created_at=now, updated_at=now,
                       request=CreateJobRequest(**BASE))
    data = resp.model_dump(mode="json")
    assert data["status"] == "queued" and data["progress"]["done"] == 0
    assert JobResponse.model_validate(data) == resp
