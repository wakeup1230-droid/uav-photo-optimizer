# Limitations

UAV Photo Optimizer provides **geometric UAV photo-selection recommendations**. It does not
guarantee downstream photogrammetric reconstruction success.

## What the tool does not check

* Image quality: blur, exposure, noise, texture (e.g. water, snow, uniform fields).
* Whether enough tie points will be found between the kept photos.
* Control points, camera calibration in the photogrammetry software, processing settings.
* Completeness or accuracy of the resulting DSM / mesh / orthophoto.

Results from Metashape, ContextCapture, ODM or other software depend on these factors. Check
the final reconstruction yourself.

## Geometric assumptions

* Footprints are projected onto a **horizontal ground plane** at the laser-measured height.
  In steep or rugged terrain the real footprint differs (slopes facing the camera shrink it,
  ridges can hide ground). Overlap in such terrain may be over- or underestimated.
* Accuracy depends on the recorded GPS position, gimbal angles and laser range.
* Camera metadata is read from DJI tags. Other aircraft / cameras may lack fields; photos
  without usable geometry are kept rather than removed.
* Overlap is geometric (footprint shapes), not measured from image content.

## Data assumptions

* One AOI shapefile; the photos must contain GPS positions.
* Existing gaps in the original flight data are reported and not enlarged, but not repaired.

## Experimental features

Visual Guard (image matching) and terrain-aware footprints (local DEM / DSM) are optional,
default OFF and experimental. Their results are research aids, not guarantees.
