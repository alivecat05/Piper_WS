"""Small data types, pose mapping, and time-alignment rules."""
from collections import deque
from dataclasses import dataclass
from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation

CAMERAS = ("head", "left_wrist", "right_wrist")
ARMS = ("left", "right")
VECTOR_NAMES = [f"{side}.{field}" for side in ARMS
                for field in ("x", "y", "z", "qx", "qy", "qz", "qw", "gripper_m")]


@dataclass(frozen=True)
class Sample:
    t: float  # Host monotonic seconds, never wall clock or headset clock.
    value: object


class History:
    def __init__(self, capacity=240):
        self.items = deque(maxlen=capacity)
        self.lock = Lock()

    def add(self, t, value):
        with self.lock:
            if self.items and t <= self.items[-1].t:
                return  # Do not renew cached feedback or accept clock regressions.
            self.items.append(Sample(t, value))

    def snapshot(self):
        with self.lock:
            return list(self.items)

    def latest(self):
        items = self.snapshot()
        return items[-1] if items else None

    def select(self, t, tolerance, causal=False):
        items = self.snapshot()
        if causal:
            items = [x for x in items if x.t <= t]
        sample = min(items, key=lambda x: abs(x.t - t), default=None)
        return sample if sample and abs(sample.t - t) <= tolerance else None

    def fps(self, now, window=2.):
        items = [x for x in self.snapshot() if now - window <= x.t <= now]
        if len(items) < 2 or now - items[-1].t > .5:
            return 0.
        return (len(items) - 1) / (items[-1].t - items[0].t)


def pose_to_vector(pose6, width):
    return [*pose6[:3], *Rotation.from_euler("xyz", pose6[3:]).as_quat(), width]


def vector_to_pose(vector):
    return [*vector[:3], *Rotation.from_quat(vector[3:7]).as_euler("xyz")]


class PoseMapper:
    """Relative pose clutch: each press captures fresh controller/TCP origins."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.basis = np.array(cfg["xr_to_base"], dtype=float)
        if not (np.allclose(self.basis.T @ self.basis, np.eye(3), atol=1e-6)
                and np.isclose(np.linalg.det(self.basis), 1)):
            raise ValueError("xr_to_base must be a right-handed 3x3 rotation")
        self.anchor = None
        self.target = None

    def release(self, state):
        self.anchor = None
        self.target = list(state)

    def update(self, hand, state, dt):
        width = (1 - hand["trigger"]) * self.cfg["gripper_max_m"]
        if not hand["grip"]:
            if self.anchor is not None or self.target is None:
                self.release(state)
            self.target[7] = width
            return list(self.target)
        if self.anchor is None:
            self.anchor = (np.array(hand["position"]), Rotation.from_quat(hand["quaternion"]), list(state))
            self.target = list(state)
            self.target[7] = width
            return list(self.target)  # Clutch press itself never moves the robot.
        origin_p, origin_r, origin_tcp = self.anchor
        position = np.array(origin_tcp[:3]) + self.cfg["position_scale"] * self.basis @ (
            np.array(hand["position"]) - origin_p)
        position = np.clip(position, self.cfg["workspace_min"], self.cfg["workspace_max"])
        delta = position - self.target[:3]
        limit = self.cfg["max_speed_m_s"] * dt
        position = np.array(self.target[:3]) + delta * min(1., limit / max(np.linalg.norm(delta), 1e-9))
        basis_r = Rotation.from_matrix(self.basis)
        desired_r = (basis_r * Rotation.from_quat(hand["quaternion"]) * origin_r.inv()
                     * basis_r.inv() * Rotation.from_quat(origin_tcp[3:7]))
        current_r = Rotation.from_quat(self.target[3:7])
        rotvec = (desired_r * current_r.inv()).as_rotvec()
        rotvec *= min(1., self.cfg["max_rotation_rad_s"] * dt / max(np.linalg.norm(rotvec), 1e-9))
        quaternion = (Rotation.from_rotvec(rotvec) * current_r).as_quat()
        # Keep quaternion sign continuous within the episode.
        if np.dot(quaternion, self.target[3:7]) < 0:
            quaternion = -quaternion
        self.target = [*position, *quaternion, width]
        return list(self.target)


class Synchronizer:
    def __init__(self, cameras, states, actions, cfg):
        self.cameras, self.states, self.actions, self.cfg = cameras, states, actions, cfg
        self.last_frames = {}

    def sample(self, t):
        cfg = self.cfg
        frames = {name: hist.select(t, cfg["camera_tolerance_ms"] / 1000)
                  for name, hist in self.cameras.items()}
        states = {name: hist.select(t, cfg["state_tolerance_ms"] / 1000)
                  for name, hist in self.states.items()}
        actions = {name: hist.select(t, cfg["action_max_age_ms"] / 1000, causal=True)
                   for name, hist in self.actions.items()}
        if not all([*frames.values(), *states.values(), *actions.values()]):
            return None
        stamps = [x.t for x in frames.values()]
        if max(stamps) - min(stamps) > cfg["camera_span_ms"] / 1000:
            return None
        for name, state in states.items():
            # Every field used by observation.state must be fresh, including gripper.
            if any(abs(stamp - t) > cfg["state_tolerance_ms"] / 1000
                   for stamp in state.value["source_timestamps"].values()):
                return None
        ids = {name: x.value["frame_number"] for name, x in frames.items()}
        if any(ids[name] == self.last_frames.get(name) for name in ids):
            return None
        self.last_frames = ids
        return {
            "timestamp_monotonic_s": t,
            "action": [v for side in ARMS for v in actions[side].value],
            "observation.state": [v for side in ARMS for v in states[side].value["vector"]],
            "joints": {side: states[side].value["joints"] for side in ARMS},
            "source_timestamps": {
                "cameras": {n: x.t for n, x in frames.items()},
                "states": {n: x.value["source_timestamps"] for n, x in states.items()},
                "actions": {n: x.t for n, x in actions.items()},
            },
            "camera_metadata": {n: {k: v for k, v in x.value.items() if k != "bgr"}
                                for n, x in frames.items()},
            "images": {n: x.value["bgr"] for n, x in frames.items()},
        }

