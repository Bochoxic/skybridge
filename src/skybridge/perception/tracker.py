"""Turn per-frame detections into a coasting constant-velocity track."""

from __future__ import annotations

import numpy as np

from .detector import select_candidate
from .types import DroneEstimate


class ConstantVelocityKF:
    """Kalman filter on [x, y, z, vx, vy, vz] with white-noise acceleration."""

    def __init__(self, accel_std, meas_std):
        self.q = accel_std ** 2
        self.R = np.eye(3) * meas_std ** 2
        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))
        self.x = None
        self.P = None

    def reset(self):
        self.x = self.P = None

    def predict(self, dt):
        if self.x is None:
            return
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        # Discrete white-noise acceleration model, per axis
        q = self.q * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                               [dt ** 3 / 2, dt ** 2]])
        Q = np.kron(q, np.eye(3))
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z, noise_scale=1.0):
        """noise_scale multiplies the measurement std (less trusted measurement)."""
        R = self.R * noise_scale ** 2
        if self.x is None:
            self.x = np.concatenate((z, np.zeros(3)))
            self.P = np.diag([*np.diag(R), 1.0, 1.0, 1.0])
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P


class Tracker:
    """Candidate confirmation plus a coasting constant-velocity KF track.

    Points and centroids are in the camera frame; the filter runs in world.
    """

    def __init__(self, args, cam_R, cam_t):
        self.args = args
        self.R, self.t = cam_R, cam_t
        self.kf = ConstantVelocityKF(args.accel_std, args.meas_std)
        self.streak, self.prev = 0, None  # tentative candidate (no track yet)
        self.prev_t = self.last_update_t = None
        self.pred = None  # world-frame prediction this frame (before update), if tracking
        self.n_tracks = 0  # tracks confirmed so far
        self.diag = {}  # why this frame was / wasn't associated (see REASONS)
        # Output blending: the filter itself stays exact; the *output* position
        # absorbs each correction and eases into it with time constant blend_tau,
        # so re-acquisition after coasting doesn't jump. Live-compatible.
        self.blend_tau = max(getattr(args, "blend_ms", 0.0), 0.0) / 1000.0
        self.offset = np.zeros(3)

    def _decay(self, dt):
        if self.blend_tau > 0:
            return np.exp(-dt / self.blend_tau)
        return 0.0

    def output(self, ahead=0.0):
        """Smoothed world position `ahead` seconds after the last frame
        (constant-velocity prediction plus the decaying blend offset), or None."""
        if self.kf.x is None:
            return None
        return self.kf.x[:3] + self.kf.x[3:] * ahead + self.offset * self._decay(ahead)

    def estimate(self, t, seq, status) -> DroneEstimate:
        """An immutable snapshot of the track after the frame at time `t`."""
        reason = self.diag.get("reason", "")
        if self.kf.x is None:
            return DroneEstimate(t=t, seq=seq, status=status, valid=False, reason=reason)
        pos = self.output()
        vel = self.kf.x[3:]
        std = np.sqrt(np.diag(self.kf.P)[:3])
        return DroneEstimate(
            t=t, seq=seq, status=status, valid=True,
            pos=(float(pos[0]), float(pos[1]), float(pos[2])),
            vel=(float(vel[0]), float(vel[1]), float(vel[2])),
            pos_std=(float(std[0]), float(std[1]), float(std[2])),
            since_update=t - self.last_update_t,
            reason=reason,
        )

    def to_world(self, p):
        return self.R @ p + self.t

    def to_camera(self, p):
        return self.R.T @ (p - self.t)

    def step(self, t, points):
        """One frame. Returns (cluster, status, n_clusters, centroid_cam,
        world_pos, detected); centroid_cam / world_pos are None without a cluster."""
        a = self.args
        dt = 0.0 if self.prev_t is None else t - self.prev_t
        self.prev_t = t
        self.kf.predict(dt)
        self.offset = self.offset * self._decay(dt)

        # A live filter means a confirmed track: search around its prediction
        tracking = self.kf.x is not None
        self.pred = self.kf.x[:3].copy() if tracking else None
        ref = self.to_camera(self.pred) if tracking else self.prev
        cluster, status, n_clusters, self.diag = select_candidate(points, ref, a)
        self.diag["gate"] = a.gate
        self.diag["since_update"] = (t - self.last_update_t) if tracking else None
        sparse = False
        radius = getattr(a, "sparse_radius", 0.0)
        if (tracking and cluster is None and radius > 0 and len(points)
                and self.diag["reason"] in ("too_few_points", "dbscan_noise", "few_points")):
            # Track active, no cluster, but a few points right at the prediction:
            # too sparse for DBSCAN (partial depth), still almost surely the drone
            near = points[np.linalg.norm(points - ref, axis=1) <= radius]
            if len(near) >= a.sparse_min_points:
                cluster, status, sparse = near, "candidate", True

        centroid = world_pos = None
        detected = False
        if cluster is None:
            if tracking:
                status = "coasting"
            else:
                self.streak, self.prev = 0, None
        else:
            centroid = cluster.mean(axis=0)
            world_pos = self.to_world(centroid)
            if tracking or (self.prev is not None
                            and np.linalg.norm(centroid - self.prev) <= a.gate):
                self.streak += 1
            else:
                self.streak = 1
            self.prev = centroid
            # Confirmed track: a cluster at the prediction is the drone, no re-persist
            detected = tracking or self.streak >= a.persist

        if detected:
            status = "detected"
            if not tracking:
                self.n_tracks += 1
                self.offset = np.zeros(3)
                self.kf.update(world_pos)
            else:
                before = self.kf.x[:3].copy()
                self.kf.update(world_pos, a.sparse_noise if sparse else 1.0)
                if self.blend_tau > 0:
                    self.offset = self.offset + before - self.kf.x[:3]
            self.last_update_t = t
            if sparse:
                self.diag.update(cand_n=len(cluster), cand_dist=float(
                    np.linalg.norm(centroid - ref)), cand_extent=float(
                    (cluster.max(axis=0) - cluster.min(axis=0)).max()))
            self.diag["reason"] = "ok_sparse" if sparse else "ok"
        elif tracking and t - self.last_update_t > a.max_coast:
            status = "lost"
            self.diag["reason"] = "lost_timeout"
            self.offset = np.zeros(3)
            self.kf.reset()
            self.last_update_t = None
            self.streak, self.prev = 0, None
        return cluster, status, n_clusters, centroid, world_pos, detected
