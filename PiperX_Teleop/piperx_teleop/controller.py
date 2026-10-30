"""Grip controls motion; B/A control recording. These lifecycles are independent."""
import json
from collections import deque
import logging
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .core import ARMS, History, PoseMapper, Synchronizer
from .recording import Recorder
from .feedback import Feedback
from .robot import validate_vector

LOG = logging.getLogger(__name__)


class Controller:
    def __init__(self, cfg, robots, cameras, mock=False, config_path=None):
        self.cfg, self.robots, self.cameras = cfg, robots, cameras
        self.mock, self.config_path = mock, Path(config_path) if config_path else None
        self.states = {s: History() for s in ARMS}
        self.actions = {s: History() for s in ARMS}
        self.mappers = {s: PoseMapper(cfg["arms"][s]) for s in ARMS}
        self.enabled = dict.fromkeys(ARMS, False)
        self.motion_active = dict.fromkeys(ARMS, False)
        self.cartesian_blocked = dict.fromkeys(ARMS, "")
        self.resume_since = None
        self.resume_hands = set()
        self.hand_rearm = dict.fromkeys(ARMS, False)
        self.gripper_rearm = dict.fromkeys(ARMS, False)
        self.paused_width = {}
        self.last_commands = {}
        self.enable_since = dict.fromkeys(ARMS, None)
        self.sync = Synchronizer(cameras.histories, self.states, self.actions, cfg["sync"])
        self.recorder = Recorder(cfg["output"], cfg, mock)
        self.record_end_t = None
        self.ready, self.feedback_missing = False, list(ARMS)
        self.mode, self.message = "waiting", "Waiting for three RGB cameras, both arms and Quest"
        self.packet = None
        self.input_frames = History()
        self.socket_connected = False
        self.button_counts = {s: {"middle": 0, "trigger": 0} for s in ARMS}
        self.input_lock = threading.Lock()
        self.input_events = deque()
        self.received_buttons = {}
        self.last_event = self.event_ack = 0
        self.feedback = Feedback(cfg.get("feedback"))
        self.stop_event = threading.Event()
        self.previous_hands = {}
        self.hold_since, self.hold_latched = {}, set()
        self.connected = False
        self.commands_started = False

    def accept(self, packet, received=None):
        received = time.perf_counter() if received is None else received
        with self.input_lock:
            events = packet.get("events")
            if events is None:  # Also support local callers sending button states.
                changes = [(n, down) for n, down in packet["buttons"].items()
                           if down != self.received_buttons.get(n, False)]
                events = [{"id": self.last_event+i+1, "name": n, "down": down, "age_ms": 0}
                          for i, (n, down) in enumerate(changes)]
            new = [event for event in events if event["id"] > self.last_event]
            if [event["id"] for event in new] != list(range(self.last_event+1, self.last_event+len(new)+1)):
                raise ValueError("Button event sequence gap")
            if len(self.input_events) + len(new) > 64:
                raise ValueError("Button event queue full")
            self.input_events.extend({**event, "t": received-event["age_ms"]/1000} for event in new)
            if new:
                self.last_event = new[-1]["id"]
            self.received_buttons = dict(packet["buttons"])
            previous = self.packet[1] if self.packet else {}
            for side in ARMS:
                old, hand = previous.get(side, {}), packet[side]
                self.button_counts[side]["middle"] += int(hand["grip"] and not old.get("grip", False))
                self.button_counts[side]["trigger"] += int(hand["trigger"] > .7 and old.get("trigger", 0) <= .7)
            self.packet = (received, packet)
            self.input_frames.add(received, None)

    def begin_input(self):
        with self.input_lock:
            self.packet = None
            self.socket_connected = True
            self.input_frames = History()
            self.button_counts = {s: {"middle": 0, "trigger": 0} for s in ARMS}
            self.input_events.clear()
            self.received_buttons.clear()
            self.last_event = self.event_ack = 0

    def disconnect(self):
        with self.input_lock:
            self.socket_connected = False
            self.packet = None
            self.input_events.clear()

    def start(self):
        self.feedback.start()
        self.thread = threading.Thread(target=self._run, daemon=True, name="robot-control")
        self.thread.start()

    def status(self):
        now = time.perf_counter()
        recording = self.recorder.active and self.record_end_t is None
        return {"mode": self.mode, "ready": self.ready, "recording": recording,
                "quest": self.input_status(),
                "grippers": self.gripper_status(),
                "event_ack": self.event_ack, "notifications": self.feedback.snapshot(),
                "haptics": self.feedback.haptics,
                "saving": self.recorder.busy and not recording,
                "frames": self.recorder.count, "dropped": self.recorder.dropped,
                "message": self.message, "camera_errors": dict(self.cameras.errors),
                "fps": {"cameras": {n: h.fps(now) for n, h in self.cameras.histories.items()},
                        "action": {n: h.fps(now) for n, h in self.actions.items()},
                        "state": {n: h.fps(now) for n, h in self.states.items()},
                        "recording": self.recorder.writes.fps(now) if self.recorder.busy else 0.}}

    def gripper_status(self):
        result = {}
        for side in ARMS:
            state, action = self.states[side].latest(), self.actions[side].latest()
            result[side] = {
                "actual_mm": round(state.value["vector"][7]*1000, 1) if state else None,
                "target_mm": round(action.value[7]*1000, 1) if action else None,
                "enabled": state.value.get("gripper_enabled") if state else None,
                "status_code": state.value.get("gripper_status") if state else None,
                "feedback_age_ms": round((time.perf_counter()-state.t)*1000, 1) if state else None,
            }
        return result

    def input_status(self):
        with self.input_lock:
            packet = self.packet
            now = time.perf_counter()
            data = packet[1] if packet else {}
            return {"connected": self.socket_connected,
                    "age_ms": round(max(0, now-packet[0])*1000, 1) if packet else None,
                    "hz": round(self.input_frames.fps(now), 1), "active": data.get("active", False),
                    "reason": data.get("inactive_reason", ""),
                      "buttons": dict(data.get("buttons", {})),
                    **{s: {"tracked": data.get(s, {}).get("tracked", False),
                           "middle": data.get(s, {}).get("grip", False),
                           "trigger": data.get(s, {}).get("trigger", 0),
                           "presses": dict(self.button_counts[s]),
                           "reason": data.get(s, {}).get("tracking_reason", "")} for s in ARMS}}

    def _notify(self, kind, message):
        self.message = message
        self.feedback.emit(kind, message)

    def _fault(self, message):
        LOG.info("Last commands before fault: %s", json.dumps(self.last_commands))
        LOG.info("Quest at fault: %s", json.dumps(self.input_status()))
        for robot in self.robots.values() if self.commands_started else ():
            try:
                robot.stop()
            except Exception:
                LOG.exception("Electronic stop failed")
        self.mode = "fault"
        self._notify("fault", message)
        self.ready = False
        self.recorder.stop(message, complete=False)
        with self.input_lock:
            self.input_events.clear()
            self.event_ack = self.last_event
        LOG.error(message)

    def _read(self, now):
        for side, robot in self.robots.items():
            try:
                result = robot.read()
            except Exception as error:
                raise RuntimeError(f"{side}: {error}") from error
            if result:
                self.states[side].add(*result)
        latest = {side: history.latest() for side, history in self.states.items()}
        self.feedback_missing = [side for side, state in latest.items()
            if state is None or any(now - stamp > self.cfg["feedback_timeout_s"] or stamp > now + .05
                                   for stamp in state.value["source_timestamps"].values())]
        if self.feedback_missing:
            return None
        return {side: sample.value for side, sample in latest.items()}

    def _pause_input(self, states, reason, required_hands=()):
        self.resume_since = None
        self.ready = False
        if self.mode != "paused":
            self.resume_hands = set(required_hands)
            self.recorder.stop(reason, complete=False)
            self.record_end_t = None
            for side, robot in self.robots.items():
                # Fresh robot feedback is required by the caller. Failure falls back to _fault.
                if self.enabled[side] or self.enable_since[side] is not None or self.mode == "resetting":
                    robot.hold(states[side]["joints"])
                self.paused_width[side] = states[side]["vector"][7]
                if self.commands_started:
                    robot.send_gripper(self.paused_width[side])
                    self.actions[side].add(time.perf_counter(), list(states[side]["vector"]))
                self.mappers[side].release(states[side]["vector"])
                self.motion_active[side] = self.enabled[side] = False
                self.enable_since[side] = None
                self.cartesian_blocked[side] = ""
                self.gripper_rearm[side] = True
                self.hand_rearm[side] = True
            self.reset_jobs, self.reset_active = [], None
            self.previous_hands.clear()
            self.hold_since.clear()
            self.hold_latched.clear()
            self.mode = "paused"
            LOG.warning("Input paused; holding robot positions: %s", reason)
            self._notify("paused", reason)
        self.message = reason + "; restore tracking, then release all buttons/triggers"

    def _waiting_for(self, now, input_missing):
        missing = [f"{side} feedback" for side in self.feedback_missing]
        for name, history in self.cameras.histories.items():
            frame = history.latest()
            error = self.cameras.errors.get(name)
            if error:
                missing.append(f"RGB {name}: {error}")
            elif frame is None or not -.05 <= now - frame.t <= self.cfg["camera_timeout_s"]:
                missing.append(f"RGB {name}")
        missing.extend(input_missing)
        return missing

    def _held(self, name, down, now, duration):
        if not down:
            self.hold_since.pop(name, None)
            self.hold_latched.discard(name)
            return False
        self.hold_since.setdefault(name, now)
        if now - self.hold_since[name] < duration or name in self.hold_latched:
            return False
        self.hold_latched.add(name)
        return True

    def _save_home(self, states):
        if self.recorder.busy or self.mode != "idle":
            self._notify("rejected", "Release both middle buttons and finish saving before saving home")
            return
        if self.mock or self.config_path is None:
            self._notify("rejected", "Home is saved only in real robot mode with a config file")
            return
        for side in ARMS:
            issue = self.robots[side].cartesian_start_issue(states[side]["joints"])
            if issue:
                self._notify("rejected", f"Home not saved: {side}: {issue}")
                return
        saved = json.loads(self.config_path.read_text(encoding="utf-8"))
        for side in ARMS:
            path = saved["arms"][side]["home_waypoints"]
            joints = list(states[side]["joints"])
            saved["arms"][side]["home_waypoints"] = [*path[:-1], joints] if path else [joints]
        temp = self.config_path.with_suffix(".tmp")
        temp.write_text(json.dumps(saved, indent=2, ensure_ascii=False), encoding="utf-8")
        temp.replace(self.config_path)
        for side in ARMS:
            self.cfg["arms"][side]["home_waypoints"] = saved["arms"][side]["home_waypoints"]
        self._notify("home_saved", "Current joint poses saved as home")

    def _begin_reset(self, now):
        if self.recorder.busy:
            self._notify("rejected", "Press A to end recording; wait for saving before resetting")
            return
        if any(not self.cfg["arms"][s]["home_waypoints"] for s in ARMS):
            self._notify("rejected", "Save home with X first, or configure home_waypoints")
            return
        for side in ARMS:
            issue = self.robots[side].cartesian_start_issue(self.cfg["arms"][side]["home_waypoints"][-1])
            if issue:
                self._notify("rejected", f"Home needs a working pose: {side}: {issue}")
                return
        self.mode = "resetting"
        self._notify("home_started", "Going home; release Y and press Y again to cancel")
        self.reset_jobs = [(s, joints) for s in ARMS for joints in self.cfg["arms"][s]["home_waypoints"]]
        self.reset_active = None
        self.reset_start = now

    def _cancel_reset(self, states):
        # Replace joint trajectories with measured joint targets; keep motors enabled.
        for side in ARMS:
            self.robots[side].hold(states[side]["joints"])
            self.mappers[side].release(states[side]["vector"])
            self.motion_active[side] = False
            self.enabled[side] = False  # Restore normal speed on the next grip.
        self.reset_jobs.clear()
        self.reset_active = None
        self.previous_hands.clear()
        self.hold_latched.add("reset")  # Holding the cancel press must not start another reset.
        self.mode = "idle"
        self._notify("home_cancelled", "Home cancelled")

    def _reset_tick(self, states, now):
        self.commands_started = True
        # Wait for enable acknowledgements without blocking input/feedback watchdogs.
        for side in ARMS:
            if not self.enabled[side]:
                self.enabled[side] = bool(self.robots[side].enable())
        if not all(self.enabled.values()):
            if now - self.reset_start > 5:
                raise TimeoutError("Home enable timeout")
            return
        if self.reset_active is not None:
            side, joints, sent, settled = self.reset_active
            if now - sent > self.cfg["reset_timeout_s"]:
                raise TimeoutError(f"Home waypoint timeout: {side}")
            reached = np.max(np.abs(np.array(states[side]["joints"]) - joints)) < .04
            settled = (settled or now) if reached else None
            self.reset_active = side, joints, sent, settled
            if settled is None or now - settled < .3:
                return
            self.reset_active = None
        if not self.reset_jobs:
            self.mode = "idle"
            self._notify("home_done", "Home complete")
            for side in ARMS:
                self.mappers[side].release(states[side]["vector"])
                self.motion_active[side] = False
                self.enabled[side] = False  # Restore normal speed on next grip press.
            return
        side, joints = self.reset_jobs.pop(0)
        self.robots[side].move_home(joints)
        self.reset_active = side, joints, now, None

    def _finish_record(self):
        if not self.recorder.busy and self.record_end_t is not None:
            self.record_end_t = None
            if self.recorder.result == "complete":
                self._notify("saved", f"Saved {self.recorder.path}")
            else:
                self._notify("rejected", "No synchronized frames; episode incomplete")

    def _record_tick(self, now):
        if self.recorder.error:
            raise RuntimeError(self.recorder.error)
        self._finish_record()
        if not self.recorder.active:
            return
        limit = now - self.cfg["sync"]["delay_ms"] / 1000
        if self.record_end_t is not None:
            limit = min(limit, self.record_end_t)
        while self.next_record_t <= limit:
            sample = self.sync.sample(self.next_record_t)
            if sample:
                self.recorder.add(sample, self.grid_index)
            else:
                self.recorder.dropped += 1
            self.grid_index += 1
            self.next_record_t += 1 / self.cfg["record_hz"]
        if self.record_end_t is not None and self.next_record_t > self.record_end_t:
            self.recorder.stop(end_time=self.record_end_t)

    def _reject_events(self, events, message):
        for event in events:
            if event["down"]:
                self._notify("rejected", message)
            self.event_ack = event["id"]

    def _button_event(self, event, states):
        name, down = event["name"], event["down"]
        if name in ("reset", "set_home"):
            if down:
                self.hold_since[name] = event["t"]
            else:
                self.hold_since.pop(name, None)
                self.hold_latched.discard(name)
        if not down:
            return
        if self.mode == "resetting":
            if name == "reset":
                self._cancel_reset(states)
            else:
                self._notify("rejected", "Going home; press Y again to cancel first")
            return
        if name == "record_start":
            if not all(self.input_status()[s]["tracked"] for s in ARMS):
                self._notify("rejected", "Recording needs both controllers tracked")
                return
            if self.recorder.busy:
                self._notify("rejected", "Already recording or saving; wait before starting another episode")
                return
            self.recorder.start(event["t"])
            self.record_end_t = None
            self.next_record_t, self.grid_index = event["t"], 0
            self.sync.last_frames = {}
            self._notify("record_started", "Recording; A: stop and save")
        elif name == "record_stop":
            if self.recorder.active and self.record_end_t is None:
                self.record_end_t = max(self.recorder.start_time, event["t"])
                self._notify("saving", "Saving; teleoperation remains available")
            else:
                self._notify("rejected", "Already saving" if self.recorder.busy else "Not recording")

    def tick(self, now, dt):
        with self.input_lock:
            events = list(self.input_events)
            self.input_events.clear()
        if self.mode == "fault":
            self._reject_events(events, "Fault; restart the program before operating")
            return
        if self.recorder.error:
            raise RuntimeError(self.recorder.error)
        states = self._read(now)
        with self.input_lock:
            packet = self.packet
            input_age = time.perf_counter() - packet[0] if packet else float("inf")
        data = packet[1] if packet and 0 <= input_age <= self.cfg["input_timeout_s"] else None
        required_hands = set()
        if self.mode == "paused":
            required_hands = self.resume_hands
        elif self.mode == "resetting" or self.recorder.active:
            required_hands = set(ARMS)
        else:
            for side in ARMS:
                previous = self.previous_hands.get(side, {})
                action = self.actions[side].latest()
                gripper_moving = states is not None and action is not None and abs(
                    action.value[7]-states[side]["vector"][7]) > .002
                if previous.get("tracked") and (previous.get("grip") or
                        previous.get("trigger", 0) > .05 or gripper_moving):
                    required_hands.add(side)
        input_missing = []
        if packet is None:
            input_missing.append("Quest no XR frames" if self.socket_connected else "Quest disconnected")
        elif data is None:
            input_missing.append(f"Quest input timeout ({input_age*1000:.0f} ms)")
        else:
            if not data["active"]:
                input_missing.append("Quest " + (data.get("inactive_reason") or "session inactive"))
            for side in ARMS:
                if side in required_hands and not data[side]["tracked"]:
                    input_missing.append(f"Quest {side}: " + (data[side].get("tracking_reason") or "tracking lost"))
        hardware_missing = self._waiting_for(now, [])
        missing = hardware_missing + input_missing
        self.ready = not missing
        if missing:
            if self.connected:
                if hardware_missing:
                    self._fault("Lost: " + "; ".join(missing))
                else:
                    self._pause_input(states, "Lost: " + "; ".join(input_missing), required_hands)
                    self._reject_events(events, self.message)
            else:
                self.mode, self.message = "waiting", "Waiting: " + "; ".join(missing)
                # A button held during startup must not become a queued command.
                self._reject_events(events, self.message)
            return
        if self.mode == "paused":
            self.ready = False
            neutral = (not any(data["buttons"].values()) and
                       all(not data[s]["grip"] and data[s]["trigger"] <= .05 for s in ARMS))
            self.resume_since = (self.resume_since if self.resume_since is not None else now) if neutral else None
            self.message = "Tracking restored; release all buttons/triggers for 0.3s, then press again"
            self._reject_events(events, self.message)
            if self.resume_since is not None and now-self.resume_since >= .3:
                self.mode, self.ready = "idle", True
                self.message = "Ready: press middle to take control; squeeze trigger to resume gripper"
            return
        if not self.connected:
            self.message = "Ready | Middle button: arm | Index trigger: gripper | B: record | A: save"
        self.connected = True
        buttons = data["buttons"]
        released = not any(data[s]["grip"] for s in ARMS)
        self._finish_record()
        was_resetting = self.mode == "resetting"
        if self.mode != "resetting":
            self.mode = "idle" if released else "teleop"
        for event in events:
            if now - event["t"] > self.cfg["input_timeout_s"]:
                self._fault("Button event timed out")
                return
            self._button_event(event, states)
            self.event_ack = event["id"]
        if was_resetting and self.mode != "resetting":
            return
        if self.mode == "resetting":
            self._reset_tick(states, now)
            return
        self.commands_started = True
        for side in ARMS:
            hand, previous = data[side], self.previous_hands.get(side)
            if not hand["tracked"]:
                self.hand_rearm[side] = True
                self.previous_hands.pop(side, None)
                continue  # An idle missing controller must never block the other hand or X/A/B.
            if self.hand_rearm[side]:
                if hand["grip"] or hand["trigger"] > .05:
                    self.message = f"{side}: release middle/trigger once before taking control"
                    continue
                self.hand_rearm[side] = False
            if previous and hand["grip"] and previous["grip"]:
                if np.linalg.norm(np.array(hand["position"]) - previous["position"]) > .15:
                    raise RuntimeError(f"{side}: controller tracking jumped")
            self.previous_hands[side] = {**hand, "position": list(hand["position"])}
            effective = dict(hand)
            pressed = hand["grip"] and not (previous and previous["grip"])
            released_hand = not hand["grip"] and previous and previous["grip"]
            if pressed:
                self.cartesian_blocked[side] = self.robots[side].cartesian_start_issue(states[side]["joints"])
                if self.cartesian_blocked[side]:
                    self._notify("rejected", f"{side}: {self.cartesian_blocked[side]}")
                else:
                    self.message = f"{side}: hold; move controller to follow"
            if (pressed and not self.cartesian_blocked[side]) or (released_hand and self.enabled[side]):
                # Replace any old trajectory BEFORE enabling. No Cartesian IK on the clutch edge.
                joints = list(states[side]["joints"])
                self.last_commands[side] = {"kind": "joint_hold", "joints": joints,
                                            "state": list(states[side]["vector"])}
                try:
                    self.robots[side].hold(joints)
                except Exception as error:
                    raise RuntimeError(f"{side}: {error}") from error
                self.motion_active[side] = False
                self.mappers[side].release(states[side]["vector"])
            if hand["grip"] and not self.cartesian_blocked[side] and not self.enabled[side]:
                self.enable_since[side] = self.enable_since[side] or now
                self.enabled[side] = bool(self.robots[side].enable())
                if not self.enabled[side]:
                    effective["grip"] = False
                    if now - self.enable_since[side] > 5:
                        raise TimeoutError(f"{side}: enable timeout")
            if not hand["grip"]:
                self.enable_since[side] = None
                self.motion_active[side] = False
                self.cartesian_blocked[side] = ""
            if self.cartesian_blocked[side]:
                effective["grip"] = False
            if self.gripper_rearm[side]:
                if hand["trigger"] > .05:
                    self.gripper_rearm[side] = False
                else:
                    effective["trigger"] = 1-self.paused_width[side]/self.cfg["arms"][side]["gripper_max_m"]
            target = self.mappers[side].update(effective, states[side]["vector"], min(dt, .05))
            validate_vector(target, self.cfg["arms"][side])
            if np.linalg.norm(np.array(target[:3]) - states[side]["vector"][:3]) > self.cfg["max_position_tracking_error_m"]:
                raise RuntimeError(f"{side}: robot cannot follow target")
            anchor = self.mappers[side].anchor
            if effective["grip"] and anchor is not None:
                origin_p, origin_r, _ = anchor
                # Require deliberate motion before switching from joint hold to Cartesian IK.
                hand_rotation = origin_r.inv() * Rotation.from_quat(hand["quaternion"])
                moved = (np.linalg.norm(np.array(hand["position"])-origin_p) > .005
                         or hand_rotation.magnitude() > np.deg2rad(3))
                self.motion_active[side] = self.motion_active[side] or moved
            if effective["grip"] and self.motion_active[side]:
                self.last_commands[side] = {"kind": "cartesian", "target": list(target),
                    "state": list(states[side]["vector"]), "joints": list(states[side]["joints"])}
                self.robots[side].send(target)
            else:
                # Gripper remains independent of Grip; no move_p while idle or awaiting enable.
                if anchor is not None:
                    target[:7] = anchor[2][:7]
                    self.mappers[side].target[:7] = target[:7]
                self.robots[side].send_gripper(target[7])
            self.actions[side].add(time.perf_counter(), target)
        self._record_tick(now)
        if self._held("set_home", buttons["set_home"] and released, now, 2.):
            self._save_home(states)
        if self._held("reset", buttons["reset"] and released, now, 1.5):
            self._begin_reset(now)

    def _run(self):
        last = time.perf_counter()
        while not self.stop_event.is_set():
            now = time.perf_counter()
            try:
                self.tick(now, now-last)
            except Exception as error:
                LOG.info("Control fault details", exc_info=True)
                self._fault(str(error))
            last = now
            time.sleep(max(0, 1/self.cfg["control_hz"] - (time.perf_counter()-now)))

    def close(self):
        self.stop_event.set()
        if hasattr(self, "thread"):
            self.thread.join(timeout=8)
        # Stop robots before waiting for any disk writes.
        for robot in self.robots.values():
            try:
                if self.commands_started:
                    robot.stop()
            except Exception:
                LOG.exception("Shutdown stop failed")
            try:
                robot.close()
            except Exception:
                LOG.exception("Robot disconnect failed")
        self.recorder.stop("shutdown", complete=False)
        if not self.recorder.wait(timeout=5):
            LOG.error("Episode writer still busy at shutdown; do not use an unfinished episode")
        self.feedback.close()
