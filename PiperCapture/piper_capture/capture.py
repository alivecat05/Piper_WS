"""Small capture API. One worker owns device polling and home motion."""
import copy
import json
from pathlib import Path
import threading
import time

import numpy as np

from .cameras import Cameras
from .core import ARMS, CAMERAS, History, Synchronizer
from .recording import Recorder
from .robot import MockPiperX, PiperX

SHARED_CAN = "leader_follower_shared_can"
HOME_UNAVAILABLE = "当前为主从共用 CAN：尚未实现主从返回自定义初始位，未发送运动命令"


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if cfg.get("topology", "independent") not in ("independent", SHARED_CAN):
        raise ValueError("Unknown robot topology")
    if set(cfg["arms"]) != set(ARMS) or set(cfg["cameras"]) != set(CAMERAS):
        raise ValueError("Configure two arms and three cameras")
    if not 1 <= cfg["record_hz"] <= 60 or not 30 <= cfg["poll_hz"] <= 200:
        raise ValueError("Invalid recording/poll frequency")
    sync = cfg["sync"]
    if min(sync.values()) <= 0 or sync["delay_ms"] < max(
            sync["camera_tolerance_ms"], 1000/cfg["record_hz"] + sync["state_tolerance_ms"]):
        raise ValueError("Sync delay must cover RGB matching and next-step action tolerance")
    for arm in cfg["arms"].values():
        if not 1 <= arm["home_speed_percent"] <= 20:
            raise ValueError("Home speed must be between 1 and 20 percent")
    return cfg


class Capture:
    def __init__(self, cfg, *, mock=False, home_path="home.json", cameras=None, robots=None):
        self.cfg, self.mock = copy.deepcopy(cfg), mock
        self.home_path = Path(home_path)
        self.cameras = cameras if cameras is not None else Cameras(cfg, mock)
        self.robots = robots if robots is not None else {}
        self.states = {s: History() for s in ARMS}
        self.recorder = Recorder(cfg["output"], self.cfg, mock)
        self.action_rate = History()
        self.lock, self.done = threading.RLock(), threading.Event()
        self.thread = None
        self.mode, self.message = "warming", "Waiting for cameras and robot feedback"
        self.end_time = None
        self.home_job = None
        self.commanded = set()

    @property
    def shared_can(self):
        return self.cfg.get("topology") == SHARED_CAN

    @property
    def home_available(self):
        return self.mock or not self.shared_can

    def open(self):
        try:
            factory = MockPiperX if self.mock else PiperX
            for side in ARMS:
                if side not in self.robots:
                    self.robots[side] = factory(self.cfg["arms"][side])
            self.cameras.start()
            self.started = time.perf_counter()
            self.thread = threading.Thread(target=self._run, daemon=True, name="piper-capture")
            self.thread.start()
            return self
        except Exception:
            self.close()
            raise

    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.close()

    def _fresh_states(self, now):
        result = {}
        for side in ARMS:
            item = self.states[side].latest()
            if item is None or any(not -.05 <= now - t <= self.cfg["feedback_timeout_s"]
                                   for t in item.value["source_timestamps"].values()):
                return None
            result[side] = item.value
        return result

    def _cameras_ready(self, now):
        return not self.cameras.errors and all(
            (item := self.cameras.histories[n].latest()) is not None
            and -.05 <= now - item.t <= self.cfg["camera_timeout_s"] for n in CAMERAS)

    def _require_idle(self):
        if self.mode != "idle" or self.recorder.busy:
            raise RuntimeError(f"Not ready: {self.mode}")
        states = self._fresh_states(time.perf_counter())
        if states is None:
            raise RuntimeError("Robot feedback is stale")
        return states

    def record(self, *, task=None, capture_index=None):
        with self.lock:
            self._require_idle()
            task = self.cfg["task"] if task is None else task
            if not isinstance(task, str) or not task.strip():
                raise ValueError("请填写任务名")
            if not self._cameras_ready(time.perf_counter()):
                raise RuntimeError("Cameras are not ready")
            task = task.strip()
            self.sync = Synchronizer(self.cameras.histories, self.states, self.cfg["sync"],
                                     1 / self.cfg["record_hz"])
            self.action_rate = History()
            self.start_time, self.grid, self.end_time = time.perf_counter(), 0, None
            self.recorder.start(self.start_time, task=task, capture_index=capture_index)
            self.cfg["task"] = task
            self.mode, self.message = "recording", "Recording manual demonstration"
            return self.status()

    def stop(self):
        with self.lock:
            if self.mode == "recording":
                self.end_time = time.perf_counter()
                self.mode, self.message = "saving", "Aligning final frames and saving"
            return self.status()

    def _home_identity(self):
        return {s: {k: self.cfg["arms"][s][k] for k in ("interface", "channel", "firmware", "tcp_offset")}
                for s in ARMS}

    def set_home(self):
        with self.lock:
            states = self._require_idle()
            arms = {s: {"joints": states[s]["joints"], "gripper_m": states[s]["vector"][7]} for s in ARMS}
            for s in ARMS:
                self.robots[s].validate_home(arms[s]["joints"], arms[s]["gripper_m"])
            saved = {"simulated": self.mock, "devices": self._home_identity(), "arms": arms,
                     "scope": "followers_only" if self.shared_can else "two_independent_arms"}
            self.home_path.parent.mkdir(parents=True, exist_ok=True)
            temp = self.home_path.with_suffix(".tmp")
            temp.write_text(json.dumps(saved, indent=2), encoding="utf-8")
            temp.replace(self.home_path)
            self.message = ("Follower reference poses saved; leader poses are not included" if self.shared_can
                            else "Initial joint poses and gripper widths saved")
            return self.status()

    def go_home(self):
        with self.lock:
            if not self.home_available:
                raise RuntimeError(HOME_UNAVAILABLE)
            self._require_idle()
            home = json.loads(self.home_path.read_text(encoding="utf-8"))
            if home["simulated"] != self.mock or home["devices"] != self._home_identity():
                raise ValueError("Home belongs to different devices or simulation mode; save it again")
            for s in ARMS:
                self.robots[s].validate_home(home["arms"][s]["joints"], home["arms"][s]["gripper_m"])
            self.home_targets = home["arms"]
            self.home_queue, self.home_job = list(ARMS), None
            self.commanded.clear()
            self.mode, self.message = "returning", "Returning left then right; C cancels"
            return self.status()

    def cancel_home(self):
        with self.lock:
            if self.mode == "returning":
                states = self._fresh_states(time.perf_counter())
                try:
                    if states is None:
                        raise RuntimeError("Cannot hold: feedback is stale")
                    for side in self.commanded:
                        self.robots[side].hold(states[side])
                    self.commanded.clear()
                    self.home_job = None
                    self.mode, self.message = "idle", "Return cancelled; arms hold their positions"
                except Exception as error:
                    self._fault(str(error))
            return self.status()

    def _home_tick(self, states, now):
        if self.home_job is None:
            if not self.home_queue:
                self.commanded.clear()
                self.mode, self.message = "idle", "At home; use the robot's teaching control before dragging"
                return
            side = self.home_queue.pop(0)
            self.commanded.add(side)
            self.home_job = {"side": side, "stage": "enabling", "start": now, "settled": None}
            self.robots[side].hold(states[side])  # Replace old targets before enabling motors.
        job = self.home_job
        side, robot, target = job["side"], self.robots[job["side"]], self.home_targets[job["side"]]
        if now - job["start"] > self.cfg["home_timeout_s"]:
            raise TimeoutError(f"{side}: home motion timed out")
        if job["stage"] == "enabling":
            if robot.enable():
                robot.move_home(target)
                job["stage"] = "moving"
            elif now - job["start"] > 5:
                raise TimeoutError(f"{side}: enable timed out")
            return
        reached = (np.max(np.abs(np.array(states[side]["joints"]) - target["joints"])) < .03
                   and abs(states[side]["vector"][7] - target["gripper_m"]) < .003)
        job["settled"] = (job["settled"] or now) if reached else None
        if job["settled"] is not None and now - job["settled"] >= .3:
            self.home_job = None

    def _record_tick(self, now):
        limit = now - self.cfg["sync"]["delay_ms"] / 1000
        if self.end_time is not None:
            limit = min(limit, self.end_time)
        while (t := self.start_time + self.grid / self.cfg["record_hz"]) <= limit:
            if self.end_time is not None and t + 1/self.cfg["record_hz"] > self.end_time:
                self.recorder.drop("episode_end")
                self.grid += 1
                continue
            sample = self.sync.sample(t)
            if sample is None:
                self.recorder.drop(self.sync.rejection)
            else:
                self.recorder.add(sample, self.grid)
                self.action_rate.add(t, self.grid)
            self.grid += 1
        if self.end_time is not None and now >= self.end_time + self.cfg["sync"]["delay_ms"] / 1000:
            self.recorder.stop("operator_stop", end_time=self.end_time)

    def _fault(self, reason):
        self.mode, self.message = "fault", reason
        self.recorder.stop(reason, complete=False, end_time=time.perf_counter())
        for side in list(self.commanded):
            try:
                self.robots[side].stop()
            except Exception as error:
                self.message += f"; {side} stop failed: {error}"
        self.commanded.clear()

    def _tick(self, now):
        for side, robot in self.robots.items():
            sample = robot.read()
            if sample is not None:
                t, value = sample
                if (np.asarray(value["vector"]).shape != (8,) or not np.isfinite(value["vector"]).all()
                        or np.asarray(value["joints"]).shape != (6,) or not np.isfinite(value["joints"]).all()):
                    raise ValueError(f"{side}: invalid robot feedback")
                self.states[side].add(t, value)
        states = self._fresh_states(now)
        camera_ready = self._cameras_ready(now)
        if self.mode == "warming":
            if states is not None and camera_ready:
                self.mode, self.message = "idle", "Ready; manual teaching mode is controlled on the robot"
            else:
                waiting = [f"{s}: {getattr(self.robots[s], 'feedback_issue', '') or 'waiting for fresh feedback'}"
                           for s in ARMS if (item := self.states[s].latest()) is None or
                           any(not -.05 <= now-t <= self.cfg["feedback_timeout_s"]
                               for t in item.value["source_timestamps"].values())]
                if not camera_ready:
                    waiting.append(f"cameras: {self.cameras.errors or 'warming up'}")
                self.message = "Waiting | " + " | ".join(waiting)
                if now - self.started > 12:
                    raise TimeoutError(self.message)
            return
        if states is None:
            detail = "; ".join(f"{s}: {getattr(r, 'feedback_issue', '')}" for s, r in self.robots.items()
                               if getattr(r, "feedback_issue", ""))
            raise TimeoutError("Robot feedback timed out" + (f"; {detail}" if detail else ""))
        if self.cameras.errors or not camera_ready:
            raise TimeoutError(f"Camera feedback timed out: {self.cameras.errors}")
        if self.recorder.error:
            raise RuntimeError(self.recorder.error)
        if self.mode == "returning":
            self._home_tick(states, now)
        elif self.mode in ("recording", "saving"):
            if self.recorder.active:
                self._record_tick(now)
            if self.mode == "saving" and not self.recorder.busy:
                self.mode = "idle"
                self.message = f"{self.recorder.result}: {self.recorder.path}"

    def _run(self):
        while not self.done.is_set():
            start = time.perf_counter()
            with self.lock:
                if self.mode != "fault":
                    try:
                        self._tick(start)
                    except Exception as error:
                        self._fault(f"{type(error).__name__}: {error}")
            self.done.wait(max(0, 1 / self.cfg["poll_hz"] - (time.perf_counter() - start)))

    def status(self):
        with self.lock:
            now = time.perf_counter()
            return {"mode": self.mode, "message": self.message,
                    "home_available": self.home_available,
                    "home_unavailable_reason": "" if self.home_available else HOME_UNAVAILABLE,
                    "recording": self.mode == "recording", "saving": self.mode == "saving",
                    "frames": self.recorder.count, "dropped": self.recorder.dropped,
                    "drop_reasons": dict(self.recorder.drop_reasons),
                    "episode": str(self.recorder.path) if self.recorder.path else None,
                    "fps": {"cameras": {n: self.cameras.histories[n].fps(now) for n in CAMERAS},
                            "feedback": {s: self.states[s].fps(now) for s in ARMS},
                            "action": self.action_rate.fps(now), "recording": self.recorder.writes.fps(now)}}

    def close(self):
        self.done.set()
        if self.thread is not None:
            self.thread.join()
        with self.lock:
            if self.commanded:
                self._fault("Closed during home motion")
            if self.recorder.active:
                self.recorder.stop("closed_before_save", complete=False, end_time=time.perf_counter())
            self.recorder.wait()
            self.cameras.close()
            for robot in self.robots.values():
                robot.close()
            self.mode = "closed"
