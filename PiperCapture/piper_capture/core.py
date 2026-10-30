"""Timestamp histories, RGB/state alignment, and next-step demonstration actions."""
from collections import deque
from dataclasses import dataclass
from itertools import product
from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation

CAMERAS = ("head", "left_wrist", "right_wrist")
ARMS = ("left", "right")
VECTOR_NAMES = [f"{side}.{field}" for side in ARMS
                for field in ("x", "y", "z", "qx", "qy", "qz", "qw", "gripper_m")]
ACTION_SEMANTICS = "next-grid measured TCP pose + gripper width; demonstration target, not a sent command"


@dataclass(frozen=True)
class Sample:
    t: float
    value: object


class History:
    def __init__(self, capacity=240):
        self.items = deque(maxlen=capacity)
        self.lock = Lock()

    def add(self, t, value):
        with self.lock:
            if not self.items or t > self.items[-1].t:
                self.items.append(Sample(t, value))

    def snapshot(self):
        with self.lock:
            return list(self.items)

    def latest(self):
        items = self.snapshot()
        return items[-1] if items else None

    def fps(self, now):
        items = [x for x in self.snapshot() if now - 2 <= x.t <= now]
        if len(items) < 2 or now - items[-1].t > .5:
            return 0.
        return (len(items) - 1) / (items[-1].t - items[0].t)


def pose_to_vector(pose, width):
    return [*pose[:3], *Rotation.from_euler("xyz", pose[3:]).as_quat(), width]


class Synchronizer:
    def __init__(self, cameras, states, cfg, period=1/30):
        self.cameras, self.states, self.cfg = cameras, states, cfg
        self.period = period
        self.last_frames = {}
        self.rejection = None

    def _states(self, t, after=None):
        tolerance = self.cfg["state_tolerance_ms"] / 1000
        result = {}
        for side in ARMS:
            candidates = [x for x in self.states[side].snapshot()
                          if abs(x.t - t) <= tolerance
                          and all(abs(stamp - t) <= tolerance
                                  for stamp in x.value["source_timestamps"].values())
                          and (after is None or all(stamp > after
                               for field, stamp in x.value["source_timestamps"].items()
                               if field in ("tcp", "gripper") or field.startswith("end_pose_")))]
            result[side] = min(candidates, key=lambda x: abs(x.t - t), default=None)
        return result if all(result.values()) else None

    def _reject(self, reason):
        self.rejection = reason
        return None

    def sample(self, t):
        """Jointly match RGB; next-step action needs robot feedback, not next-step RGB."""
        self.rejection = None
        candidates = {}
        for name in CAMERAS:
            near = [x for x in self.cameras[name].snapshot()
                    if abs(x.t - t) <= self.cfg["camera_tolerance_ms"] / 1000]
            if not near:
                return self._reject(f"camera_missing:{name}")
            candidates[name] = [x for x in near if x.value["frame_number"] > self.last_frames.get(name, -1)]
            if not candidates[name]:
                return self._reject(f"camera_reused:{name}")
        # Independent nearest frames can straddle the grid and violate the span
        # even when a neighbouring combination satisfies the SAME tolerances.
        def score(group):
            stamps = [x.t for x in group]
            return max(stamps) - min(stamps), sum(abs(stamp-t) for stamp in stamps)
        group = min(product(*(candidates[n] for n in CAMERAS)), key=score)
        if score(group)[0] > self.cfg["camera_span_ms"] / 1000:
            return self._reject("camera_span")
        frames = dict(zip(CAMERAS, group))
        states = self._states(t)
        if states is None:
            return self._reject("state_missing_or_stale")
        future = self._states(t + self.period, after=t)
        if future is None:
            return self._reject("action_missing_or_stale")
        vector = [v for side in ARMS for v in states[side].value["vector"]]
        action = [v for side in ARMS for v in future[side].value["vector"]]
        for offset in (0, 8):
            if np.dot(action[offset+3:offset+7], vector[offset+3:offset+7]) < 0:
                action[offset+3:offset+7] = [-v for v in action[offset+3:offset+7]]
        ids = {n: x.value["frame_number"] for n, x in frames.items()}
        self.last_frames = ids
        return {
            "timestamp_monotonic_s": t,
            "observation.state": vector,
            "action": action,
            "action_timestamp_monotonic_s": t + self.period,
            "joints": {side: states[side].value["joints"] for side in ARMS},
            "source_timestamps": {
                "cameras": {n: x.t for n, x in frames.items()},
                "states": {n: x.value["source_timestamps"] for n, x in states.items()},
                "actions": {n: x.value["source_timestamps"] for n, x in future.items()},
            },
            "camera_metadata": {n: {k: v for k, v in x.value.items() if k != "bgr"} for n, x in frames.items()},
            "images": {n: x.value["bgr"] for n, x in frames.items()},
        }
