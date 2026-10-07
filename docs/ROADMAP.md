# Roadmap

## v1.0.0 — released

Scope (frozen): **geometric UAV photo selection recommendation**.

AOI Shapefile + buffer + front overlap + side overlap → recommended photos → COPY to output.

* Pipeline: AOI → Buffer → Photo Metadata → PLANAR Estimated Footprint → Flight / Flight Strip
  → Front / Side / Oblique Geometric Overlap → Photo Selection Optimizer → COPY.
* Interfaces: CLI (`select`), REST API v1 (`serve`), simple GUI (`gui`).
* Defaults: buffer 100 m, front 80 %, side 70 %; overlaps allowed 65 – 95 %.
* Experimental, optional, default OFF: Visual Guard, terrain-aware footprints, weitsicht
  backend.

## Out of scope

The tool is not a photogrammetric quality-assurance system. The following are not planned
unless explicitly requested:

* guaranteeing reconstruction success (Metashape, ContextCapture, ODM, …);
* integration with photogrammetry software;
* learned image matching, AI models;
* mandatory DEM / DSM input;
* repairing gaps that already exist in the original flight data.

## Possible maintenance items

* Camera profiles for more aircraft / cameras.
* Packaging (e.g. a Windows installer) and documentation improvements.
