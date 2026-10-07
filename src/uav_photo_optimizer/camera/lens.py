"""
Lens calibration from DJI XMP ``DewarpData`` / ``DewarpDataK6``.

Format (documented DJI calibrated intrinsic / distortion metadata; verified on M4E data, see
internal research notes):

    DewarpData   = "YYYY-MM-DD;fx,fy,cx,cy,k1,k2,p1,p2,k3"
    DewarpDataK6 = "YYYY-MM-DD;fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6"

* fx, fy in pixels; **cx, cy are offsets from the photo centre** (pixels). DJI Mavic 3M Image
  Processing Guide: camera matrix = [(fx, 0, CenterX+cx), (0, fy, CenterY+cy), (0, 0, 1)] with
  CenterX/Y = CalibratedOpticalCenterX/Y (W/2, H/2 when absent)
* OpenCV model on normalized coordinates x = (u - cx_abs) / fx, y = (v - cy_abs) / fy:
      radial  = (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)
      x_d = x·radial + 2 p1 x y + p2 (r² + 2x²)
      y_d = y·radial + p1 (r² + 2y²) + 2 p2 x y
  (BROWN5: k4 = k5 = k6 = 0; K6_RATIONAL uses all eight)

Recorded pixels are distorted (``DewarpFlag = 0``). ``undistort_pixels`` maps recorded pixel
positions to ideal pinhole pixels of the same (fx, fy, cx_abs, cy_abs) camera.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

import numpy as np
from pydantic import BaseModel, ConfigDict


class DistortionModel(str, Enum):
    BROWN5 = "BROWN5"              # DewarpData:   k1 k2 p1 p2 k3
    K6_RATIONAL = "K6_RATIONAL"    # DewarpDataK6: k1 k2 p1 p2 k3 k4 k5 k6 (OpenCV rational)


class LensCalibration(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    source: str                 # "XMP-drone-dji:DewarpData" / "...DewarpDataK6"
    calibration_date: Optional[str] = None
    model: DistortionModel
    fx: float
    fy: float
    cx: float                   # absolute pixel coordinate (origin top-left)
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0
    k5: float = 0.0
    k6: float = 0.0

    # -- model ---------------------------------------------------------------------------

    def distort_normalized(self, x: np.ndarray, y: np.ndarray):
        r2 = x * x + y * y
        num = 1 + r2 * (self.k1 + r2 * (self.k2 + r2 * self.k3))
        den = 1 + r2 * (self.k4 + r2 * (self.k5 + r2 * self.k6))
        radial = num / den
        xd = x * radial + 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
        yd = y * radial + self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
        return xd, yd

    def undistort_normalized(self, xd: np.ndarray, yd: np.ndarray, iterations: int = 50):
        """Invert the model with fixed-point iteration (OpenCV ``undistortPoints`` style)."""
        x, y = xd.copy(), yd.copy()
        for _ in range(iterations):
            r2 = x * x + y * y
            num = 1 + r2 * (self.k1 + r2 * (self.k2 + r2 * self.k3))
            den = 1 + r2 * (self.k4 + r2 * (self.k5 + r2 * self.k6))
            radial = num / den
            dx = 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
            dy = self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        return x, y

    def undistort_pixels(self, pixels: np.ndarray) -> np.ndarray:
        """Recorded (distorted) pixels (N, 2) → ideal pinhole pixels of this camera."""
        xd = (pixels[:, 0] - self.cx) / self.fx
        yd = (pixels[:, 1] - self.cy) / self.fy
        x, y = self.undistort_normalized(xd, yd)
        return np.column_stack([x * self.fx + self.cx, y * self.fy + self.cy])

    def roundtrip_error_px(self, pixels: np.ndarray) -> float:
        """Max |distort(undistort(p)) - p| in pixels (numerical check of the inversion)."""
        und = self.undistort_pixels(pixels)
        x = (und[:, 0] - self.cx) / self.fx
        y = (und[:, 1] - self.cy) / self.fy
        xd, yd = self.distort_normalized(x, y)
        back = np.column_stack([xd * self.fx + self.cx, yd * self.fy + self.cy])
        return float(np.abs(back - pixels).max())


def parse_dewarp(value: Optional[str], width: int, height: int,
                 source: str = "XMP-drone-dji:DewarpData",
                 center_x: Optional[float] = None,
                 center_y: Optional[float] = None) -> Optional[LensCalibration]:
    """Parse a DJI DewarpData / DewarpDataK6 string; None if absent or malformed."""
    if not value:
        return None
    date, _, numbers = value.partition(";") if ";" in value else ("", "", value)
    try:
        v = [float(t) for t in numbers.split(",") if t.strip()]
    except ValueError:
        return None
    if len(v) == 9:
        model = DistortionModel.BROWN5
        k4 = k5 = k6 = 0.0
    elif len(v) == 12:
        model = DistortionModel.K6_RATIONAL
        k4, k5, k6 = v[9:12]
    else:
        return None
    fx, fy, cx_off, cy_off, k1, k2, p1, p2, k3 = v[:9]
    if fx <= 0 or fy <= 0:
        return None
    return LensCalibration(source=source, calibration_date=date or None, model=model,
                           fx=fx, fy=fy,
                           cx=(center_x if center_x is not None else width / 2) + cx_off,
                           cy=(center_y if center_y is not None else height / 2) + cy_off,
                           k1=k1, k2=k2, p1=p1, p2=p2, k3=k3, k4=k4, k5=k5, k6=k6)
