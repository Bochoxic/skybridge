"""One depth frame in, one result out. The same code runs live and on recordings.

No threads, no I/O, no clock: everything time-related comes from the frame.

    warm-up     the first ``bg_frames`` frames build the static background
    foreground  pixels in front of that background
    crop        3D points inside the flight volume
    track       cluster, associate, filter
    plate       the drone's yaw from the plate on top, on detected frames
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from .background import DepthBackground
from .config import PipelineParams
from .geometry import BackProjector, camera_to_world, crop_box
from .plate import PlateParams, measure_plate
from .tracker import Tracker
from .types import CameraModel, DepthFrame, DroneEstimate, PerceptionResult


class DepthPipeline:
    def __init__(self, params: PipelineParams, camera: CameraModel,
                 plate: PlateParams = PlateParams()) -> None:
        self.params = params
        self.plate = plate
        self.camera = camera
        self.project = BackProjector(camera.intrinsics, camera.depth_scale, params.stride)
        self.box = (np.array([params.x_min, params.y_min, params.z_min]),
                    np.array([params.x_max, params.y_max, params.z_max]))
        self.cam_R, self.cam_t = camera_to_world(params.cam_height, params.cam_pitch)
        self.background = None if params.no_bg else DepthBackground(
            camera.depth_scale, params.stride, params.bg_delta, params.bg_delta_rel)
        self.tracker = Tracker(params, self.cam_R, self.cam_t)
        self.n_frames = 0

    @property
    def warming_up(self) -> bool:
        return self.background is not None and self.n_frames < self.params.bg_frames

    def process(self, frame: DepthFrame) -> PerceptionResult:
        if self.warming_up:
            return self._warmup(frame)
        self.n_frames += 1

        fg = None if self.background is None else self.background.foreground(frame.depth)
        cropped = crop_box(self.project(frame.depth, fg), self.box)
        cluster, status, n_clusters, centroid, world_pos, detected = \
            self.tracker.step(frame.t, cropped)

        estimate = self.tracker.estimate(frame.t, frame.seq, status)
        plate = None
        if detected and self.plate.enabled:
            plate = measure_plate(frame.depth, self.camera, self.cam_R, self.cam_t,
                                  world_pos, self.plate)
            if plate is not None:
                estimate = replace(estimate, plate_yaw=float(plate.yaw_deg),
                                   plate_elong=float(plate.elong))

        kf = self.tracker.kf
        return PerceptionResult(
            frame=frame,
            estimate=estimate,
            foreground=fg,
            cropped=cropped,
            cluster=cluster,
            n_clusters=n_clusters,
            centroid=centroid,
            world_pos=world_pos,
            detected=detected,
            prediction=self.tracker.pred,
            filtered=None if kf.x is None else kf.x[:3].copy(),
            velocity=None if kf.x is None else kf.x[3:].copy(),
            output=self.tracker.output(),
            plate=plate,
            diag=self.tracker.diag,
        )

    def _warmup(self, frame: DepthFrame) -> PerceptionResult:
        self.background.add(frame.depth)
        self.n_frames += 1
        ready = self.n_frames == self.params.bg_frames
        if ready:
            self.background.finalize()
        return PerceptionResult(
            frame=frame,
            estimate=DroneEstimate(t=frame.t, seq=frame.seq, status="warmup",
                                   valid=False, reason="warmup"),
            warmup=True,
            background_ready=ready,
        )
