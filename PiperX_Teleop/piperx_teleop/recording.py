"""Bounded background writer; manifest is committed after successful writes."""
import copy
import json
import os
import time
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread
from datetime import datetime
from uuid import uuid4

import cv2

from .core import CAMERAS, VECTOR_NAMES, History


class Recorder:
    def __init__(self, root, cfg, mock=False):
        self.root, self.cfg, self.mock = Path(root), cfg, mock
        self.thread = None
        self.finished = Event()
        self.path = None
        self.error = None
        self.result = None
        self.count = self.dropped = 0
        self.writes = History()

    @property
    def active(self):
        return self.busy and not self.finished.is_set()

    @property
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, start_time):
        if self.busy:
            return
        self.path = self.root / (datetime.now().strftime("episode_%Y%m%d_%H%M%S_") + uuid4().hex[:6])
        self.count = self.dropped = 0
        self.error = None
        self.result = None
        self.writes = History()
        self.start_time = start_time
        self.end_time = None
        self.reason, self.complete = "controller_stop", True
        self.config_snapshot = copy.deepcopy(self.cfg)
        self.queue = Queue(maxsize=60)
        self.finished = Event()
        self.thread = Thread(target=self._write, daemon=True, name="episode-writer")
        self.thread.start()

    def add(self, sample, grid_index):
        if not self.active:
            return
        if self.error:
            raise RuntimeError(self.error)
        try:
            self.queue.put_nowait((sample, grid_index))
        except Full as error:
            self.error = "Episode writer queue full; disk cannot sustain recording"
            raise RuntimeError(self.error) from error

    def _write(self):
        try:
            self.path.mkdir(parents=True)
            for name in CAMERAS:
                (self.path / "images" / name).mkdir(parents=True)
            self._manifest("recording", "")
            with (self.path / "samples.jsonl").open("w", encoding="utf-8") as stream:
                # The producer requests completion without waiting for the disk.
                while not self.finished.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=.02)
                    except Empty:
                        continue
                    sample, grid_index = item
                    row = {key: value for key, value in sample.items() if key != "images"}
                    row.update(grid_index=grid_index, frame_index=self.count, images={})
                    for name, bgr in sample["images"].items():
                        relative = f"images/{name}/{self.count:08d}.jpg"
                        if not cv2.imwrite(str(self.path / relative), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                            raise OSError(f"Could not write {relative}")
                        row["images"][name] = relative
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
                    stream.flush()
                    self.count += 1
                    self.writes.add(time.perf_counter(), self.count)
                os.fsync(stream.fileno())
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
        try:
            result = "complete" if self.complete and not self.error and self.count else "incomplete"
            self._manifest(result, self.reason)
            self.result = result
        except Exception as error:
            self.error = f"Could not finalize episode: {type(error).__name__}: {error}"

    def stop(self, reason="controller_stop", complete=True, end_time=None):
        """Stop accepting samples; the writer drains and commits in the background."""
        if not self.active:
            return
        self.reason, self.complete, self.end_time = reason, complete, end_time
        self.finished.set()

    def wait(self, timeout=None):
        """Only for shutdown/offline callers, never the robot control loop."""
        if self.thread is not None:
            self.thread.join(timeout)
        return not self.busy

    def _manifest(self, status, reason):
        manifest = {
            "schema": "piperx.rgb_action.v1", "status": status, "reason": reason,
            "simulated": self.mock, "frames": self.count, "dropped_grid_ticks": self.dropped,
            "fps": self.config_snapshot["record_hz"], "task": self.config_snapshot["task"],
            "start_monotonic_s": self.start_time, "end_monotonic_s": self.end_time,
            "vector_names": VECTOR_NAMES,
            "action_semantics": "absolute TCP target + gripper width; last host-dispatched command <= observation time",
            "pose_convention": "each arm base; meters; quaternion xyzw; TCP offset in config; gripper meters",
            "synchronization": "software alignment; see per-frame timestamp_basis and source_timestamps",
            "config": self.config_snapshot, "error": self.error,
        }
        temp = self.path / "manifest.tmp"
        temp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        temp.replace(self.path / "manifest.json")

