"""Find the drone among the foreground points of one frame."""

from __future__ import annotations

import numpy as np
from sklearn.cluster import DBSCAN

# Why a frame produced no association (Tracker.diag["reason"]):
#   ok               a cluster was associated
#   ok_sparse        track active, no cluster, but >= --sparse-min-points box
#                    points within --sparse-radius of the prediction: used as
#                    the measurement with --sparse-noise x measurement std
#   no_points        no foreground points inside the crop box
#   too_few_points   fewer than --min-points points in the box (DBSCAN not run)
#   too_many_points  more than --max-points to cluster (DBSCAN not run), so a
#                    cluttered frame cannot stall the loop
#   dbscan_noise     points exist but DBSCAN formed no cluster (sparser than
#                    --min-samples within --eps); check near_pts: if > 0 the
#                    drone was probably there but too sparse
#   few_points       the relevant cluster has fewer than --min-points points
#   too_small        relevant cluster extent below --min-size
#   too_big          relevant cluster extent above --max-size
#   outside_gate     relevant cluster is drone-like but cand_dist > gate
#   ambiguous        no track and two comparable clusters (--ambiguity-ratio)
#   lost_timeout     coasted longer than --max-coast, track dropped
# "Relevant cluster" = the cluster nearest the prediction (or previous
# candidate) when there is one, else the largest cluster.
REASONS = ("ok", "ok_sparse", "no_points", "too_few_points", "too_many_points", "dbscan_noise",
           "few_points", "too_small", "too_big", "outside_gate", "ambiguous", "lost_timeout")


def cluster_verdict(n, extent, dist, args):
    """First check a cluster fails, or 'ok'. dist is None without a reference."""
    if n < args.min_points:
        return "few_points"
    if extent < args.min_size:
        return "too_small"
    if extent > args.max_size:
        return "too_big"
    if dist is not None and dist > args.gate:
        return "outside_gate"
    return "ok"


def select_candidate(points, ref, args):
    """Pick the drone candidate among DBSCAN clusters.

    ref: where the drone is expected (camera frame), or None if unknown.
    Returns (cluster points or None, status, number of valid clusters, diag).
    diag explains the decision: reason (see REASONS), near_pts / near_dist
    (box points within --gate of ref / distance of the closest one), and
    cand_* for the relevant cluster, plus every cluster with its verdict.
    """
    diag = {"reason": "ok", "near_pts": None, "near_dist": None, "cand_n": None,
            "cand_extent": None, "cand_dist": None, "clusters": []}
    if ref is not None and len(points):
        d = np.linalg.norm(points - ref, axis=1)
        diag["near_pts"], diag["near_dist"] = int((d <= args.gate).sum()), float(d.min())
    if len(points) == 0:
        diag["reason"] = "no_points"
        return None, "none", 0, diag
    if len(points) < args.min_points:
        diag["reason"] = "too_few_points"
        return None, "none", 0, diag

    # DBSCAN cost grows quickly with the point count (about 120 ms at 5000), and
    # a live loop can't afford that. With an expected position, only the points
    # around it matter; without one, there is no way to choose.
    max_points = getattr(args, "max_points", 0)
    if max_points and len(points) > max_points:
        if ref is not None:
            points = points[d <= args.gate]
        if ref is None or len(points) > max_points:
            diag["reason"] = "too_many_points"
            return None, "none", 0, diag
        if len(points) == 0:
            diag["reason"] = "no_points"
            return None, "none", 0, diag

    labels = DBSCAN(eps=args.eps, min_samples=args.min_samples).fit_predict(points)
    ids, counts = np.unique(labels[labels >= 0], return_counts=True)
    if len(ids) == 0:
        diag["reason"] = "dbscan_noise"
        return None, "none", 0, diag

    clusters = []  # every DBSCAN cluster, with its measurements and verdict
    for lab, n in zip(ids, counts):
        c = points[labels == lab]
        centroid = c.mean(axis=0)
        extent = float((c.max(axis=0) - c.min(axis=0)).max())
        dist = None if ref is None else float(np.linalg.norm(centroid - ref))
        clusters.append({"points": c, "centroid": centroid, "n": int(n), "extent": extent,
                         "dist": dist, "verdict": cluster_verdict(n, extent, dist, args)})
    diag["clusters"] = clusters

    def explain(c, reason=None):
        diag.update(reason=reason or c["verdict"], cand_n=c["n"],
                    cand_extent=c["extent"], cand_dist=c["dist"])

    if args.no_bg:  # legacy: largest cluster wins
        best = max(clusters, key=lambda c: c["n"])
        if best["n"] >= args.min_points:
            explain(best, "ok")
            return best["points"], "candidate", len(clusters), diag
        explain(best, "few_points")
        return None, "none", len(clusters), diag

    if ref is not None:
        # Reference = track prediction or previous candidate (camera frame):
        # nearest acceptable cluster; otherwise explain the nearest one overall
        valid = [c for c in clusters if c["verdict"] == "ok"]
        n_drone_like = sum(c["verdict"] in ("ok", "outside_gate") for c in clusters)
        if valid:
            best = min(valid, key=lambda c: c["dist"])
            explain(best)
            return best["points"], "candidate", n_drone_like, diag
        explain(min(clusters, key=lambda c: c["dist"]))
        return None, "none", n_drone_like, diag

    valid = sorted((c for c in clusters if c["verdict"] == "ok"),
                   key=lambda c: c["n"], reverse=True)
    if not valid:
        explain(max(clusters, key=lambda c: c["n"]))
        return None, "none", 0, diag
    if len(valid) > 1 and valid[1]["n"] >= args.ambiguity_ratio * valid[0]["n"]:
        explain(valid[0], "ambiguous")
        return None, "ambiguous", len(valid), diag
    explain(valid[0])
    return valid[0]["points"], "candidate", len(valid), diag
