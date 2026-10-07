"""
Visual Safety Guard (Phase 8A / 6B — Experimental).

Geometric safety (Phase 6A) answers "do the footprints still overlap?". This guard answers
"do the remaining neighbours still match in image content?" for the adjacencies the optimizer
changed. A plan becomes VALIDATED (exportable) only when:

    new coverage holes = 0, new geometric defects = 0, baseline defects worsened = 0
    AND new visual failed edges = 0

Edges validated (never all-pairs):
    NEW_EDGE                 kept neighbours that were NOT adjacent before (removed photos in
                             between) — ALL of them, nadir and oblique, no sampling
    LOW_MARGIN_EDGE / OBLIQUE_EDGE / BASELINE_DEFECT_NEIGHBOR
                             unchanged adjacencies, recorded for information (a failure there is
                             a baseline visual defect, not caused by the optimizer)

A NEW_EDGE that is FAIL or VISUAL_UNRESOLVED restores the middle removed photo
(RESTORED_BY_VISUAL_VALIDATION); the two shorter edges are validated again until PASS or the
edge is an original adjacency. Geometric safety is re-checked afterwards. HEIGHT_UNRESOLVED
photos stay PROTECTED whatever the visual result.
"""

from __future__ import annotations

import time
from collections import Counter, defaultdict
from typing import Optional

from ..core.result import GeometryAnalysis
from ..visual.base import PhotoInput, ValidationContext, VisualValidator
from ..visual.models import (EdgeKind, VisualEdge, VisualStatus, VisualValidationConfig,
                             VisualValidationReport)
from .models import (DecisionStatus, PhotoDecision, PlanStatus, SelectionPlan,
                     SelectionReason)
from .planner import DPSelectionOptimizer, EPS_PCT

R = SelectionReason


def _edges(opt: DPSelectionOptimizer, kept: set, cfg: VisualValidationConfig,
           target: float) -> list[VisualEdge]:
    defect_zone = {p for d in opt.defects for p in d.affected_photos}
    out = []
    for (sid, grp), ps in opt.seq.items():
        if grp == "OTHER":
            continue
        ks = [p for p in ps if p in kept]
        for a, b in zip(ks, ks[1:]):
            between = ps[ps.index(a) + 1:ps.index(b)]
            geo = opt._along(a, b)
            if between:
                kind = EdgeKind.NEW_EDGE
            elif a in defect_zone or b in defect_zone:
                kind = EdgeKind.BASELINE_DEFECT_NEIGHBOR
            elif grp != "NADIR":
                kind = EdgeKind.OBLIQUE_EDGE
            elif geo is None or geo < target + 5:
                kind = EdgeKind.LOW_MARGIN_EDGE
            else:
                kind = EdgeKind.BASELINE_EDGE
            if kind is EdgeKind.BASELINE_EDGE or (kind is not EdgeKind.NEW_EDGE
                                                  and not cfg.validate_unchanged_edges):
                continue
            out.append(VisualEdge(photo_a=a, photo_b=b, strip_id=sid, view_group=grp, kind=kind,
                                  removed_between=between,
                                  geometric_overlap_pct=None if geo is None else round(geo, 3)))
    return out


def validate_plan(geometry: GeometryAnalysis, area, plan: SelectionPlan, metadata: dict,
                  validator: VisualValidator,
                  visual_config: Optional[VisualValidationConfig] = None,
                  max_iterations: int = 50, lens_mode=None) -> SelectionPlan:
    """Run the Visual Safety Guard on a geometric plan → new plan (VALIDATED / INVALID)."""
    from ..camera.intrinsics import resolve_intrinsics
    from ..camera.models import LensMode

    cfg = visual_config or VisualValidationConfig()
    ctx = ValidationContext(cfg)
    t0 = time.perf_counter()
    opt = DPSelectionOptimizer()
    base = opt.plan(geometry, area, plan.config)           # deterministic: rebuild state
    if [d.model_dump() for d in base.decisions] != [d.model_dump() for d in plan.decisions]:
        raise ValueError("plan does not match the geometry / config it was produced from")
    removed = dict(opt.final_removed)
    restored_geo = dict(opt.final_restored)
    visual_restored: dict[str, str] = {}
    inputs: dict[str, PhotoInput] = {}

    def photo(pid):
        if pid not in inputs:
            m = metadata[pid]
            inputs[pid] = PhotoInput(pid, m.path, resolve_intrinsics(m, lens_mode=lens_mode
                                                                     or LensMode.AUTO))
        return inputs[pid]

    cache: dict = {}

    def check(edge: VisualEdge):
        key = (edge.photo_a, edge.photo_b)
        if key not in cache:
            cache[key] = validator.validate_pair(photo(edge.photo_a), photo(edge.photo_b), ctx)
        edge.result = cache[key]
        return edge.result

    report = VisualValidationReport(config=cfg)
    seen_new: dict = {}
    for it in range(1, max_iterations + 1):
        report.iterations = it
        kept = opt.cset - set(removed)
        edges = _edges(opt, kept, cfg, plan.config.front_target_pct)
        failing = []
        for e in edges:
            r = check(e)
            if e.kind is EdgeKind.NEW_EDGE:
                seen_new[(e.photo_a, e.photo_b)] = r.status
                if r.status is not VisualStatus.PASS:
                    failing.append(e)
        if not failing:
            break
        for e in failing:                    # restore the middle removed photo
            mid = e.removed_between[len(e.removed_between) // 2]
            removed.pop(mid, None)
            visual_restored[mid] = (f"{e.result.status.value} {e.photo_a} → {e.photo_b} "
                                    f"({e.result.essential_inliers or e.result.fundamental_inliers}"
                                    " inliers)")
        # geometric re-check after visual restores (restores only add photos)
        for _ in range(plan.config.max_restore_iterations):
            safety, fixes = opt._validate(opt.cset - set(removed), removed, opt.defects)
            if safety.total == 0 or not fixes:
                break
            for pid, why in fixes.items():
                removed.pop(pid, None)
                restored_geo[pid] = why
    kept = opt.cset - set(removed)
    final_edges = _edges(opt, kept, cfg, plan.config.front_target_pct)
    for e in final_edges:
        check(e)
    safety, _ = opt._validate(kept, removed, opt.defects)

    st = Counter(seen_new.values())
    report.edges = final_edges
    report.new_edges = len(seen_new)
    report.new_edges_pass = st[VisualStatus.PASS]
    report.new_edges_fail = st[VisualStatus.FAIL]
    report.new_edges_unresolved = st[VisualStatus.UNRESOLVED]
    report.visual_restored = sorted(visual_restored)
    report.remaining_new_failed_edges = sum(
        e.kind is EdgeKind.NEW_EDGE and e.result.status is not VisualStatus.PASS
        for e in final_edges)
    report.baseline_visual_defects = sum(
        e.kind is not EdgeKind.NEW_EDGE and e.result.status is not VisualStatus.PASS
        for e in final_edges)
    report.images_processed = len(inputs)
    report.processing_time_s = round(time.perf_counter() - t0, 1)

    # rebuild decisions
    edge_of = defaultdict(list)
    for e in final_edges:
        edge_of[e.photo_a].append(e)
        edge_of[e.photo_b].append(e)
    along_next = _along_metrics(opt, kept)
    cross = _cross_metrics(opt, kept)
    decisions = []
    for d in base.decisions:
        p = d.photo_id
        detail = dict(d.detail)
        status, reason = d.status, d.reason
        if p in visual_restored:
            status, reason = DecisionStatus.KEEP, R.RESTORED_BY_VISUAL_VALIDATION
            detail["restored_by"] = visual_restored[p]
        elif p in restored_geo and d.status is DecisionStatus.REMOVE:
            status, reason = DecisionStatus.KEEP, R.RESTORED_BY_VALIDATION
            detail["restored_by"] = restored_geo[p]
        if status is not DecisionStatus.REMOVE:
            es = sorted(edge_of.get(p, []), key=lambda e: (e.result.status is VisualStatus.PASS,
                                                           -(e.result.essential_inliers or 0)))
            if es:
                r = es[0].result                     # worst validated adjacent edge
                detail.update(visual_status=r.status.value,
                              visual_inliers=r.essential_inliers or r.fundamental_inliers,
                              visual_inlier_ratio=r.essential_inlier_ratio
                              or r.fundamental_inlier_ratio)
            detail["along_metric_pct"] = along_next.get(p)
            detail["cross_metric_pct"] = cross.get(p)
        fp = geometry.footprints.get(p)
        detail["height_strategy"] = fp.height_strategy if fp else "HEIGHT_UNRESOLVED"
        detail["height_confidence"] = fp.height_confidence if fp else None
        decisions.append(PhotoDecision(**{**d.model_dump(exclude={"status", "reason", "detail"}),
                                          "status": status, "reason": reason, "detail": detail}))

    counts = defaultdict(lambda: defaultdict(int))
    by_reason = defaultdict(int)
    for d in decisions:
        counts[d.capture_type][d.status.value] += 1
        by_reason[d.reason.value] += 1
    n_rem = sum(d.status is DecisionStatus.REMOVE for d in decisions)
    n_prot = sum(d.status is DecisionStatus.PROTECTED for d in decisions)
    geometry_ok = safety.total == 0
    visual_ok = report.remaining_new_failed_edges == 0
    status = PlanStatus.VALIDATED if geometry_ok and visual_ok else PlanStatus.INVALID
    warnings = list(base.warnings)
    if report.baseline_visual_defects:
        warnings.append(f"{report.baseline_visual_defects} unchanged adjacencies fail visual "
                        "matching (baseline visual defects, not caused by the optimizer)")
    return base.model_copy(update=dict(
        status=status, decisions=decisions, keep_count=len(decisions) - n_rem - n_prot,
        remove_count=n_rem, protected_count=n_prot,
        reduction_percent=round(100.0 * n_rem / max(1, len(decisions)), 3),
        counts_by_capture={k: dict(v) for k, v in counts.items()},
        counts_by_reason=dict(by_reason), restored_count=len(restored_geo),
        visual_restore_count=len(visual_restored), safety=safety,
        constraints_passed=geometry_ok and visual_ok, geometry_valid=geometry_ok,
        visual_valid=visual_ok, exportable=status is PlanStatus.VALIDATED,
        coverage_after=opt._coverage_summary(kept), visual=report.model_dump(mode="json"),
        warnings=warnings))


def _along_metrics(opt, kept) -> dict:
    out = {}
    for (sid, grp), ps in opt.seq.items():
        ks = [p for p in ps if p in kept]
        for a, b in zip(ks, ks[1:]):
            v = opt._along(a, b)
            if v is not None:
                for p in (a, b):
                    out[p] = round(min(out.get(p, v), v), 3)
    return out


def _cross_metrics(opt, kept) -> dict:
    from ..flight.models import CaptureType
    out = {}
    for cap in (CaptureType.NADIR, CaptureType.OBLIQUE):
        for (a, side), (b, v, sb) in opt._partners(kept, cap).items():
            if b is not None:
                out[a] = round(min(out.get(a, v), v), 3)
    return out
