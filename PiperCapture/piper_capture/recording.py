"""Bounded background writer; manifest is committed after successful writes."""
import copy
from collections import Counter
import csv
import json
import os
import re
import time
from pathlib import Path, PureWindowsPath
from queue import Empty, Full, Queue
from threading import Event, Thread

import cv2
import numpy as np

from .core import ACTION_SEMANTICS, CAMERAS, VECTOR_NAMES, History


class Recorder:
    def __init__(self, root, cfg, mock=False):
        self.root, self.cfg, self.mock = Path(root), cfg, mock
        self.thread = None
        self.finished = Event()
        self.path = None
        self.error = None
        self.result = None
        self.count = self.dropped = 0
        self.drop_reasons = Counter()
        self.missing_run = self.longest_missing_run = 0
        self.writes = History()

    @property
    def active(self):
        return self.busy and not self.finished.is_set()

    @property
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def task_directory(self, task):
        if (not isinstance(task, str) or not task.strip() or task != task.strip()
                or re.search(r'[<>:"/\\|?*\x00-\x1f]', task) or task.endswith('.')
                or PureWindowsPath(task).is_reserved()):
            raise ValueError("任务名不能为空，也不能包含路径或 Windows 文件名禁用字符")
        path = self.root / task
        if path.resolve().parent != self.root.resolve():
            raise ValueError("任务目录必须在 output 内")
        return path

    def next_index(self, task):
        folder = self.task_directory(task)
        numbers = [int(match[1]) for p in folder.glob("episode_*")
                   if (match := re.fullmatch(r"episode_(\d+)", p.name))]
        return max(numbers, default=0) + 1

    def start(self, start_time, *, task=None, capture_index=None):
        if self.busy:
            return
        task = self.cfg["task"] if task is None else task
        folder = self.task_directory(task)
        capture_index = self.next_index(task) if capture_index is None else capture_index
        if type(capture_index) is not int or capture_index < 1:
            raise ValueError("采集序号必须是正整数")
        path = folder / f"episode_{capture_index:03d}"
        try:
            path.mkdir(parents=True)  # Reserve atomically; never overwrite an existing recording.
        except FileExistsError:
            raise ValueError(f"采集目录已存在，请更换序号：{path}") from None
        self.path = path
        self.count = self.dropped = 0
        self.drop_reasons = Counter()
        self.missing_run = self.longest_missing_run = 0
        self.error = None
        self.result = None
        self.writes = History()
        self.start_time = start_time
        before = time.perf_counter()
        unix_time = time.time()
        self.unix_offset = unix_time - (before + time.perf_counter()) / 2
        self.end_time = None
        self.reason, self.complete = "controller_stop", True
        self.config_snapshot = copy.deepcopy(self.cfg)
        for key in ("task", "task_id", "capture_index"):
            self.config_snapshot.pop(key, None)
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
            self.missing_run = 0
        except Full as error:
            self.error = "Episode writer queue full; disk cannot sustain recording"
            raise RuntimeError(self.error) from error

    def drop(self, reason):
        self.dropped += 1
        self.drop_reasons[reason] += 1
        self.missing_run += 1
        self.longest_missing_run = max(self.longest_missing_run, self.missing_run)

    def _disk_row(self, sample, grid_index):
        """Persist Unix seconds only; the acquisition clock remains monotonic internally."""
        row = {key: value for key, value in sample.items()
               if key not in ("images", "timestamp_monotonic_s", "action_timestamp_monotonic_s",
                              "source_timestamps", "camera_metadata")}
        row.update(
            timestamp_unix_s=sample["timestamp_monotonic_s"] + self.unix_offset,
            action_timestamp_unix_s=sample["action_timestamp_monotonic_s"] + self.unix_offset,
            grid_index=grid_index, frame_index=self.count,
            videos={name: f"{name}.mp4" for name in CAMERAS},
        )

        def unix_stamps(value):
            if isinstance(value, dict):
                return {key: unix_stamps(item) for key, item in value.items()}
            return value + self.unix_offset

        row["source_timestamps_unix_s"] = unix_stamps(sample["source_timestamps"])
        row["camera_metadata"] = {}
        for name, meta in sample.get("camera_metadata", {}).items():
            row["camera_metadata"][name] = {
                k: v for k, v in meta.items() if k not in ("device_ms", "receive_s")}
            if "receive_s" in meta:
                row["camera_metadata"][name]["receive_unix_s"] = meta["receive_s"] + self.unix_offset
        return row

    def _check_videos(self):
        if not self.count:
            return
        for name in CAMERAS:
            video = cv2.VideoCapture(str(self.path / f"{name}.mp4"))
            try:
                if not video.isOpened() or int(video.get(cv2.CAP_PROP_FRAME_COUNT)) != self.count:
                    raise OSError(f"{name}.mp4 frame count does not match saved rows")
            finally:
                video.release()

    def _write(self):
        videos = {}
        try:
            for name in CAMERAS:
                camera = self.config_snapshot["cameras"][name]
                size = camera["width"], camera["height"]
                if any(d <= 0 or d % 2 for d in size):
                    raise ValueError("MP4 frame dimensions must be positive even numbers")
                video = cv2.VideoWriter(str(self.path / f"{name}.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"), self.config_snapshot["record_hz"], size)
                videos[name] = video
                if not video.isOpened():
                    raise OSError(f"Could not open {name}.mp4 for recording")
            self._manifest("recording", "")
            with ((self.path / "samples.jsonl").open("w", encoding="utf-8") as stream,
                  (self.path / "action.csv").open("w", encoding="utf-8", newline="") as action_file):
                actions = csv.writer(action_file)
                actions.writerow(["timestamp_unix_s", *VECTOR_NAMES])
                # The producer requests completion without waiting for the disk.
                while not self.finished.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=.02)
                    except Empty:
                        continue
                    sample, grid_index = item
                    row = self._disk_row(sample, grid_index)
                    if np.asarray(row["action"]).shape != (16,) or not np.isfinite(row["action"]).all():
                        raise ValueError("Action must contain 16 finite epose values")
                    encoded = json.dumps(row, allow_nan=False) + "\n"
                    for name in CAMERAS:
                        camera = self.config_snapshot["cameras"][name]
                        bgr = sample["images"][name]
                        if bgr.shape != (camera["height"], camera["width"], 3) or bgr.dtype != np.uint8:
                            raise ValueError(f"Invalid RGB frame: {name}")
                        videos[name].write(bgr)
                    stream.write(encoded)
                    actions.writerow([format(row["timestamp_unix_s"], ".6f"), *row["action"]])
                    stream.flush()
                    action_file.flush()
                    self.count += 1
                    self.writes.add(time.perf_counter(), self.count)
                os.fsync(stream.fileno())
                os.fsync(action_file.fileno())
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
        finally:
            for video in videos.values():
                try:
                    video.release()  # MP4 headers must be finalized before marking the episode complete.
                except Exception as error:
                    self.error = f"Could not finalize MP4: {error}"
        try:
            if not self.error:
                self._check_videos()
            result = "complete" if self.complete and not self.error and self.count else "incomplete"
            self._manifest(result, self.reason)
            self.result = result
        except Exception as error:
            self.error = f"Could not finalize episode: {type(error).__name__}: {error}"
            self.result = "incomplete"
            try:
                self._manifest("incomplete", self.reason)
            except OSError:
                pass

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
            "schema": "pipercapture.rgb_action.v2", "status": status, "reason": reason,
            "simulated": self.mock, "frames": self.count, "dropped_grid_ticks": self.dropped,
            "drop_reasons": dict(self.drop_reasons),
            "longest_missing_run_s": self.longest_missing_run / self.config_snapshot["record_hz"],
            "fps": self.config_snapshot["record_hz"], "storage_layout": "task/capture_index",
            "start_unix_s": self.start_time + self.unix_offset,
            "end_unix_s": self.end_time + self.unix_offset if self.end_time is not None else None,
            "timestamp_clock": "Unix seconds; fixed mapping from monotonic clock at episode start",
            "videos": {name: f"{name}.mp4" for name in CAMERAS},
            "action_file": "action.csv",
            "frame_alignment": "MP4 frame N = action.csv data row N = samples.jsonl row N (zero-based)",
            "video_timeline": "constant fps; original time gaps are retained in Unix timestamps and grid_index",
            "vector_names": VECTOR_NAMES,
            "action_semantics": ACTION_SEMANTICS,
            "action_horizon_s": 1 / self.config_snapshot["record_hz"],
            "last_observation_policy": "discarded if next-grid action is beyond episode end or unavailable",
            "pose_convention": "each arm base; meters; quaternion xyzw; TCP offset in config; gripper meters",
            "synchronization": "software alignment; see per-frame timestamp_basis and source_timestamps",
            "config": self.config_snapshot, "error": self.error,
        }
        temp = self.path / "manifest.tmp"
        temp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        temp.replace(self.path / "manifest.json")

