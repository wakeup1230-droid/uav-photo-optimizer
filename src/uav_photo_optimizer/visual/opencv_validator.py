"""
OpenCVVisualValidator — CPU baseline (optional extra ``visual``: opencv-python-headless).

Pipeline per pair:
    read (reduced decode, READ ONLY) → resize to ``max_long_edge`` → detect / describe
    (SIFT / AKAZE / ORB) → match (FLANN / BF) + Lowe ratio test
    → lens handling → Fundamental (USAC MAGSAC) on ideal pixels
    → Essential (normalized coords, when intrinsics are known) → Homography (diagnostic only)
    → spatial distribution (4 × 4 grid, convex hull) → status

Lens handling (never mixes distorted pixels with an undistorted camera matrix):
    UNDISTORT_KEYPOINTS  keypoints → original pixels → DewarpDataK6 / DewarpData undistortion
                         → ideal pixels (default; benchmarked in internal research notes)
    UNDISTORT_IMAGE      undistort the resized image (scaled K + OpenCV coefficients) first
    NONE                 distorted pixels, Fundamental matrix only (no Essential)
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from ..core.exceptions import ConfigError
from .base import PhotoInput, ValidationContext, VisualValidator
from .models import VisualMatchResult, VisualStatus, VisualValidationConfig


def _cv2():
    try:
        import cv2
    except ImportError as exc:   # pragma: no cover
        raise ConfigError('OpenCVVisualValidator needs the optional extra "visual": '
                          'pip install "uav-photo-optimizer[visual]"') from exc
    return cv2


class _Features:
    __slots__ = ("pts_work", "pts_ideal", "desc", "shape", "scale", "camera_ok", "error")

    def __init__(self):
        self.pts_work = self.pts_ideal = self.desc = None
        self.shape, self.scale, self.camera_ok, self.error = (0, 0), 1.0, False, None


class OpenCVVisualValidator(VisualValidator):
    name = "OpenCVVisualValidator"

    def __init__(self, config: Optional[VisualValidationConfig] = None):
        self.cv2 = _cv2()
        self.default_config = config or VisualValidationConfig()
        self._cache: dict = {}

    # -- features ------------------------------------------------------------------------

    def _detector(self, cfg: VisualValidationConfig):
        cv2 = self.cv2
        d = cfg.detector.upper()
        if d == "SIFT":
            return cv2.SIFT_create(nfeatures=cfg.max_features)
        if d == "AKAZE":                 # OpenCV 5 moved AKAZE to contrib (xfeatures2d)
            if hasattr(cv2, "AKAZE_create"):
                return cv2.AKAZE_create()
            if hasattr(cv2, "xfeatures2d") and hasattr(cv2.xfeatures2d, "AKAZE_create"):
                return cv2.xfeatures2d.AKAZE_create()
            raise ConfigError("AKAZE not available in this OpenCV build (OpenCV 5: install "
                              "opencv-contrib-python-headless)")
        if d == "ORB":
            return cv2.ORB_create(nfeatures=cfg.max_features)
        raise ConfigError(f"unknown detector {cfg.detector}")

    def _read(self, photo: PhotoInput, cfg: VisualValidationConfig):
        cv2 = self.cv2
        data = np.fromfile(str(photo.path), dtype=np.uint8)        # read-only, unicode-safe
        full_w = photo.camera.width_px if photo.camera else None
        flag = cv2.IMREAD_GRAYSCALE
        reduce = 1
        if full_w:
            for r, f in ((4, cv2.IMREAD_REDUCED_GRAYSCALE_4), (2, cv2.IMREAD_REDUCED_GRAYSCALE_2)):
                if max(full_w, photo.camera.height_px) / r >= cfg.max_long_edge:
                    flag, reduce = f, r
                    break
        img = cv2.imdecode(data, flag)
        if img is None:
            raise ValueError("image could not be decoded")
        h, w = img.shape[:2]
        orig_long = max(w, h) * reduce
        s = min(1.0, cfg.max_long_edge / max(w, h))
        if s < 1.0:
            img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
        return img, img.shape[1] / (orig_long / max(w, h) * w)   # working px per original px

    def features(self, photo: PhotoInput, cfg: VisualValidationConfig) -> _Features:
        key = (photo.photo_id, cfg.detector, cfg.max_long_edge, cfg.lens_handling,
               cfg.max_features)
        if key in self._cache:
            return self._cache[key]
        cv2, f = self.cv2, _Features()
        try:
            img, s = self._read(photo, cfg)
            f.shape, f.scale = img.shape[:2], s
            cam = photo.camera
            f.camera_ok = cam is not None and cfg.lens_handling != "NONE"
            if cfg.lens_handling == "UNDISTORT_IMAGE" and cam is not None and cam.distortion:
                d = cam.distortion
                k = np.array([[d.fx * s, 0, d.cx * s], [0, d.fy * s, d.cy * s], [0, 0, 1]])
                coeffs = np.array([d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6])
                img = cv2.undistort(img, k, coeffs)
            kps, desc = self._detector(cfg).detectAndCompute(img, None)
            pts = np.array([kp.pt for kp in kps], dtype=np.float64).reshape(-1, 2)
            f.pts_work, f.desc = pts, desc
            if (cfg.lens_handling == "UNDISTORT_KEYPOINTS" and cam is not None
                    and cam.distortion is not None and len(pts)):
                f.pts_ideal = cam.distortion.undistort_pixels(pts / s) * s
            else:
                f.pts_ideal = pts
        except Exception as exc:                       # unreadable → VISUAL_UNRESOLVED
            f.error = str(exc)
        self._cache[key] = f
        return f

    # -- matching ------------------------------------------------------------------------

    def _match(self, da, db, cfg: VisualValidationConfig):
        cv2 = self.cv2
        if da is None or db is None or len(da) < 2 or len(db) < 2:
            return [], 0
        binary = da.dtype == np.uint8
        if cfg.matcher.upper() == "FLANN" and not binary:
            m = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=64))
        elif cfg.matcher.upper() == "FLANN":
            m = cv2.FlannBasedMatcher(dict(algorithm=6, table_number=6, key_size=12,
                                           multi_probe_level=1), dict(checks=64))
        else:
            m = cv2.BFMatcher(cv2.NORM_HAMMING if binary else cv2.NORM_L2)
        knn = m.knnMatch(da, db, k=2)
        good = [p[0] for p in knn if len(p) == 2 and p[0].distance < cfg.ratio_test * p[1].distance]
        return good, len(knn)

    @staticmethod
    def _grid(pts: np.ndarray, shape, n: int) -> int:
        if len(pts) == 0:
            return 0
        h, w = shape
        gx = np.clip((pts[:, 0] / w * n).astype(int), 0, n - 1)
        gy = np.clip((pts[:, 1] / h * n).astype(int), 0, n - 1)
        return len(set(zip(gx.tolist(), gy.tolist())))

    def _hull(self, pts: np.ndarray, shape) -> float:
        if len(pts) < 3:
            return 0.0
        hull = self.cv2.convexHull(pts.astype(np.float32))
        return float(self.cv2.contourArea(hull)) / (shape[0] * shape[1])

    # -- main ------------------------------------------------------------------------------

    def validate_pair(self, photo_a: PhotoInput, photo_b: PhotoInput,
                      context: Optional[ValidationContext] = None) -> VisualMatchResult:
        cv2 = self.cv2
        cfg = context.config if context else self.default_config
        th = cfg.thresholds
        t0 = time.perf_counter()
        fa, fb = self.features(photo_a, cfg), self.features(photo_b, cfg)
        r = VisualMatchResult(photo_a=photo_a.photo_id, photo_b=photo_b.photo_id,
                              detector=cfg.detector, matcher=cfg.matcher,
                              image_long_edge=cfg.max_long_edge, status=VisualStatus.UNRESOLVED)

        def done():
            r.processing_time_ms = round((time.perf_counter() - t0) * 1000, 1)
            return r

        if fa.error or fb.error:
            r.warnings.append(f"image unreadable: {fa.error or fb.error}")
            return done()
        r.keypoints_a, r.keypoints_b = len(fa.pts_work), len(fb.pts_work)
        if min(r.keypoints_a, r.keypoints_b) < th.min_keypoints:
            r.warnings.append("too few keypoints")
            return done()
        good, r.raw_matches = self._match(fa.desc, fb.desc, cfg)
        r.ratio_test_matches = len(good)
        if len(good) < max(th.min_ratio_matches, 8):
            r.status = VisualStatus.FAIL
            r.warnings.append("too few ratio-test matches")
            return done()
        ia = np.array([m.queryIdx for m in good])
        ib = np.array([m.trainIdx for m in good])
        pa, pb = fa.pts_ideal[ia], fb.pts_ideal[ib]
        F, mask_f = cv2.findFundamentalMat(pa, pb, cv2.USAC_MAGSAC, cfg.ransac_threshold_px,
                                           cfg.ransac_confidence, 10000)
        if F is None or mask_f is None:
            r.warnings.append("fundamental matrix estimation failed")
            return done()
        mf = mask_f.ravel().astype(bool)
        r.fundamental_inliers = int(mf.sum())
        r.fundamental_inlier_ratio = round(r.fundamental_inliers / len(good), 4)
        primary = mf

        cam_a, cam_b = photo_a.camera, photo_b.camera
        if fa.camera_ok and fb.camera_ok and cam_a is not None and cam_b is not None:
            def norm(p, cam, s):
                fx, fy = cam.focal_px * s, cam.fy * s
                return np.column_stack([(p[:, 0] - cam.cx_px * s) / fx, (p[:, 1] - cam.cy_px * s) / fy])
            na, nb = norm(pa, cam_a, fa.scale), norm(pb, cam_b, fb.scale)
            thr = cfg.ransac_threshold_px / (cam_a.focal_px * fa.scale)
            E, mask_e = cv2.findEssentialMat(na, nb, np.eye(3), cv2.USAC_MAGSAC,
                                             cfg.ransac_confidence, thr)
            if E is not None and mask_e is not None and E.shape == (3, 3):
                me = mask_e.ravel().astype(bool)
                r.essential_inliers = int(me.sum())
                r.essential_inlier_ratio = round(r.essential_inliers / len(good), 4)
                primary = me
            else:
                r.warnings.append("essential matrix estimation failed; Fundamental used")
        H, mask_h = cv2.findHomography(pa, pb, cv2.RANSAC, cfg.ransac_threshold_px * 2)
        if H is not None and mask_h is not None:
            r.homography_inliers = int(mask_h.sum())
            r.homography_inlier_ratio = round(r.homography_inliers / len(good), 4)

        ra, rb = fa.pts_work[ia][primary], fb.pts_work[ib][primary]
        n = cfg.grid
        r.grid_cells_a, r.grid_cells_b = self._grid(ra, fa.shape, n), self._grid(rb, fb.shape, n)
        r.spatial_coverage_a = round(r.grid_cells_a / n ** 2, 4)
        r.spatial_coverage_b = round(r.grid_cells_b / n ** 2, 4)
        r.hull_area_ratio_a = round(self._hull(ra, fa.shape), 4)
        r.hull_area_ratio_b = round(self._hull(rb, fb.shape), 4)

        inl = int(primary.sum())
        ratio = inl / len(good)
        ok = (inl >= th.min_inliers and ratio >= th.min_inlier_ratio
              and min(r.grid_cells_a, r.grid_cells_b) >= th.min_grid_cells)
        r.status = VisualStatus.PASS if ok else VisualStatus.FAIL
        r.confidence = round(min(1.0, inl / (2 * th.min_inliers))
                             * min(1.0, ratio / max(th.min_inlier_ratio, 1e-9))
                             * min(1.0, min(r.grid_cells_a, r.grid_cells_b)
                                   / max(th.min_grid_cells, 1)), 3)
        return done()
