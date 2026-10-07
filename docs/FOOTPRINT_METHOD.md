# Footprint Method

Each photo gets a **PLANAR estimated ground footprint**: the image boundary projected onto a
horizontal ground plane below the camera. It is an estimate, not a surveyed footprint.

## Inputs (read with ExifTool, never modified)

| Input | Source (DJI) |
|---|---|
| Camera position | GPS latitude / longitude (projected to a metric CRS, e.g. the AOI's) |
| Camera orientation | `GimbalYawDegree`, `GimbalPitchDegree`, `GimbalRollDegree` |
| Focal length, principal point | `CalibratedFocalLength`, `CalibratedOpticalCenterX/Y`, image size |
| Lens distortion | `DewarpData` / `DewarpDataK6` when present (otherwise pinhole + warning) |
| Height above ground | laser rangefinder (`LRFTargetDistance`), see below |

All fields are optional in the data model; a photo whose geometry cannot be estimated is
kept (never removed) and selected by its GPS point.

## Steps

1. **Pose.** DJI angles (yaw clockwise from true north, Z-Y-X order) are converted to the grid
   of the projected CRS, including the grid convergence (difference between true and grid
   north).
2. **Image boundary.** 16 points per image edge are taken on the recorded image and
   undistorted with the lens model, so curved edges are represented.
3. **Rays.** Each boundary pixel gives a camera ray `(f, u − cx, v − cy)` in camera axes,
   rotated to grid north-east-down.
4. **Ground plane.** Rays are intersected with a horizontal plane at the camera height
   above ground. The resulting polygon is the footprint.

## Height above ground

1. **Laser height (default).** Laser range × vertical component of the laser ray. A quality
   gate checks the range, the beam angle and that the laser point falls inside the footprint.
2. **Neighbour interpolation.** If the laser of a photo is not valid, the ground elevation is
   interpolated from validated photos of the same flight strip and capture group (limited
   distance and time).
3. **Unresolved.** Otherwise the photo has no footprint and is always kept.

Take-off-relative altitude is never used automatically, because it is not height above
ground.

## Capture types

* Nadir: gimbal pitch ≤ −80°.
* Oblique: other downward pitches, grouped by look direction (45° sectors).

The same method applies to both; oblique footprints are trapezoid-like and longer.

## Optional (experimental, default OFF)

A terrain-aware footprint (rays intersected with a local GeoTIFF DEM / DSM) exists for
research. It is not used by the normal workflow.
