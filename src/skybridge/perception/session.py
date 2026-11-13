"""Run perception next to a flight: everything ``rc_fly.py --perception`` needs.

Perception runs as a child process, so nothing it does can stall the RC loop.
If it dies, flight carries on and the status line says so.
"""

from __future__ import annotations

import ctypes
import logging
import os
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from ..bus import Bus
from ..config import DroneConfig
from .config import PerceptionConfig
from .link import PerceptionLink, vis_status

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "perception_live.py"
DEFAULT_CONFIG = ROOT / "configs" / "perception.yaml"
LOG_DIR = ROOT / "data" / "live"
APP_ID_LIVE = "skybridge_live"  # viz.APP_ID, without importing rerun here


def die_with_parent(parent_pid: int) -> None:
    """Get SIGTERM when `parent_pid` dies, however it dies (Linux only).

    A perception process left running holds the camera, so every later run
    fails with "Device or resource busy". Call after installing the SIGTERM
    handler, so the process shuts down cleanly.
    """
    PR_SET_PDEATHSIG = 1
    ctypes.CDLL(None, use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
    if os.getppid() != parent_pid:  # it died before the call above took effect
        os.kill(os.getpid(), signal.SIGTERM)


def rc_channel_names(cfg: DroneConfig) -> dict[int, str]:
    names = {cfg.channels.roll: "roll", cfg.channels.pitch: "pitch",
             cfg.channels.throttle: "throttle", cfg.channels.yaw: "yaw"}
    if "arm" in cfg.modes:
        names[cfg.modes["arm"].channel] = "arm"
    return names


class PerceptionSession:
    def __init__(
        self,
        bus: Bus,
        drone: DroneConfig,
        *,
        config_path: str | Path = DEFAULT_CONFIG,
        viewer: bool = True,
        record: bool = False,
        replay: str | None = None,
        color: bool = False,
    ) -> None:
        self._bus = bus
        self._drone = drone
        self._config_path = Path(config_path)
        self.config = PerceptionConfig.from_yaml(self._config_path)
        self._viewer = viewer
        self._record = record
        self._replay = replay
        self._color = color
        self.run_id = datetime.now().strftime("flight_%Y-%m-%d_%H-%M-%S")
        self.log_path = LOG_DIR / f"{self.run_id}_perception.log"
        self.link: PerceptionLink | None = None
        self._child: subprocess.Popen[bytes] | None = None
        self._sinks: list = []
        self._recs: list = []
        self._log_file = None

    def start(self) -> None:
        self.link = PerceptionLink(self.config.link.host, self.config.link.port, self._bus)
        self.link.start()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        viewer_mode = self._start_logging()

        self._log_file = open(self.log_path, "wb")
        command = [sys.executable, str(SCRIPT), "--config", str(self._config_path),
                   "--run-id", self.run_id, "--viewer", viewer_mode,
                   "--parent-pid", str(os.getpid())]
        if self._record:
            command.append("--record")
        if self._replay:
            command += ["--replay", self._replay]
        if self._color:
            command.append("--color")
        # Its own session: Ctrl-C reaches only this process, which then shuts
        # the child down in order, so a recording is closed properly.
        self._child = subprocess.Popen(command, stdout=self._log_file, stderr=subprocess.STDOUT,
                                       start_new_session=True)

    def _start_logging(self) -> str:
        """Send RC and telemetry to the viewer, and with the states to a file. Never fatal.

        The viewer and the file are separate streams: the file gets the states
        too, but the viewer already has them from the perception process.

        Returns how the perception process should reach the viewer.
        """
        if not self._viewer and not self._record:
            return "off"
        mode = "off"
        try:
            import rerun as rr

            if self._viewer:
                try:
                    # Explicit: with set_sinks() the default batching holds data for up to 200 ms.
                    rec = rr.RecordingStream(APP_ID_LIVE, recording_id=self.run_id,
                                             batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY())
                    rec.spawn(connect=False, memory_limit=self.config.viewer.memory_limit)
                    rec.set_sinks(rr.GrpcSink())
                    self._log_bus_to(rec, log_estimates=False)
                    mode = "connect"
                except Exception as exc:
                    print(f"no Rerun viewer ({exc}); continuing without it")
            if self._record:
                self._start_file(rr)
        except Exception as exc:
            log.warning("RC and telemetry logging is off: %s", exc)
            print(f"RC and telemetry logging is off ({exc})")
        return mode

    def _start_file(self, rr) -> None:
        from .viz import make_blueprint

        rec = rr.RecordingStream(APP_ID_LIVE, recording_id=self.run_id)
        layout = make_blueprint(self.config.viewer.trail_seconds, live=True, detail=False,
                                depth=False)
        rec.save(str(LOG_DIR / f"{self.run_id}_flight.rrd"), default_blueprint=layout)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True, recording=rec)
        self._log_bus_to(rec, log_estimates=True)

    def _log_bus_to(self, rec, *, log_estimates: bool) -> None:
        from .viz import RerunBusSink

        sink = RerunBusSink(self._bus, rec, rc_channel_names(self._drone),
                            log_estimates=log_estimates)
        sink.start()
        self._recs.append(rec)
        self._sinks.append(sink)

    @property
    def child_running(self) -> bool:
        return self._child is not None and self._child.poll() is None

    def status_text(self) -> str:
        latest = self.link.latest()
        if latest is None and not self.child_running:
            return "VIS LOST (not running)"
        return vis_status(latest)

    def close(self) -> None:
        if self._child is not None:
            self._child.terminate()
            try:
                self._child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._child.kill()
                self._child.wait()
        for sink in self._sinks:
            sink.close()
        for rec in self._recs:
            rec.flush()
            rec.disconnect()
        if self.link is not None:
            self.link.close()
        if self._log_file is not None:
            self._log_file.close()
