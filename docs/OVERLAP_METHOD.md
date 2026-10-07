# Overlap and Photo Selection Method

## Flight strips

Photos are ordered by capture time and split into flights (time gaps, sequence resets,
implausible jumps). Within a flight, the trajectory heading is computed from consecutive GPS
positions and grouped into straight **strips**; turns and transit legs are recognised
separately. The gimbal yaw is never used to build strips (only to classify the oblique look
direction).

## Overlap definitions

Each strip defines an along-track axis (direction of travel) and a cross-track axis.

| Overlap | Photos compared | Measure |
|---|---|---|
| Front overlap | consecutive nadir photos of one strip | footprints projected on the along-track axis: shared length / footprint length |
| Side overlap | nadir photos of neighbouring parallel strips | same on the cross-track axis |
| Oblique along-track | consecutive oblique photos of one strip **and** look direction | along-track axis |
| Oblique cross-track | oblique photos of neighbouring strips, same look direction | cross-track axis |

Overlap values are percentages (the larger of the two relative shares). The user's front
target applies to front / oblique along-track overlap, the side target to side / oblique
cross-track overlap.

## Photo selection

The optimizer answers one question: **which photos can be omitted while the requested
overlap is kept?**

1. Candidates: photos whose footprint intersects the AOI + buffer (photos without a
   footprint: GPS point inside the area).
2. Existing gaps are recorded first (overlap already below target, coverage holes). Photos in
   and next to such gaps are protected.
3. Protected photos are never removed: unresolved geometry, turns, unusual capture
   directions, photos needed along the AOI boundary.
4. Along each strip (nadir, and each oblique look direction separately) a dynamic program
   keeps the fewest photos such that every pair of consecutive kept photos still meets the
   front target.
5. A whole nadir strip may be removed only if its neighbouring strips still meet the side
   target.
6. Validation: coverage per capture group must not shrink, no new overlap defect may appear
   and no existing defect may get worse. Otherwise photos are restored until it holds.

Selected photos = kept + protected. Reduction = removed / candidates.

If the original photos already miss the target somewhere, the result says so
(「部分原始照片重疊率低於設定值。」). The tool keeps the usable photos and does not make the gap
larger, but it cannot improve the original capture.

## Determinism

The same input and parameters always give the same selection.
