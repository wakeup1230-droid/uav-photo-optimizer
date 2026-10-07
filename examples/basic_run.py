"""
Minimal library usage of the official pipeline.

    python examples/basic_run.py
"""

from pathlib import Path

from uav_photo_optimizer import SelectionRequest, select_photos

summary = select_photos(SelectionRequest(
    aoi_shapefile=Path("input/area.shp"),       # your AOI shapefile
    photo_dir=Path("input/photos"),             # searched recursively
    output_dir=Path("output"),                  # a new run_YYYYMMDD_HHMMSS/ is created
    buffer_m=100, front_overlap=80, side_overlap=70,
    copy_photos=False,                          # True = copy the selected photos
))
print(summary.candidate_photos, summary.selected_photos, summary.removed_photos,
      f"{summary.reduction_percent:.1f} %")
for note in summary.notes:
    print(note)
