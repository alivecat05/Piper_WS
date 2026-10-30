"""Input loss holds safely; only a deliberate release and new input can resume."""
import json
import time

import pytest

from test_pipeline import cfg, make_controller, packet, step
from piperx_teleop.core import ARMS


def neutral_packet():
    data = packet()
    for side in ARMS:
        data[side]["trigger"] = 0
    return data


def resume(control):
    step(control, neutral_packet())
    control.resume_since = time.perf_counter()-.31
    step(control, neutral_packet())
    assert control.mode == "idle"


def test_tracking_loss_holds_once_and_requires_new_arm_and_gripper_input(cfg, monkeypatch):
    control = make_controller(cfg)
    data = packet()
    data["right"].update(grip=True, trigger=.6)
    step(control, data)
    robot = control.robots["right"]
    robot.joints = [.2]*6
    robot.vector[7] = .015  # Actual width differs from the last commanded width.
    holds, sends = [], []
    monkeypatch.setattr(robot, "hold", lambda joints: holds.append(list(joints)))
    monkeypatch.setattr(robot, "send", lambda target: sends.append(target))
    for arm in control.robots.values():
        monkeypatch.setattr(arm, "stop", lambda: pytest.fail("Input pause issued emergency stop"))
    data["left"].update(tracked=False, tracking_reason="position_emulated")
    step(control, data)
    assert control.mode == "paused" and holds == [[.2]*6]
    assert robot.vector[7] == .015
    assert control.gripper_status()["right"]["target_mm"] == 15
    step(control, data)
    assert len(holds) == 1
    data["left"]["tracked"] = True
    for _ in range(3):
        step(control, data)  # Grip and trigger held through tracking recovery.
    assert control.mode == "paused" and control.resume_since is None and not sends
    resume(control)
    step(control, neutral_packet())
    assert robot.vector[7] == pytest.approx(.015)  # Releasing to rearm must not drop a held object.
    assert not control.enabled["right"] and not sends
    data = neutral_packet()
    data["right"].update(grip=True, position=[1., .8, -.5])
    step(control, data)
    assert len(holds) == 2 and not sends  # Fresh anchor, no replay of the pre-loss hand displacement.
    assert robot.vector[7] == pytest.approx(.015)
    data["right"]["position"][0] += .02
    step(control, data)
    assert len(sends) == 1
    data["right"].update(grip=False, trigger=1)
    step(control, data)
    assert robot.vector[7] == 0


def test_tracking_must_remain_valid_during_release_window(cfg):
    control = make_controller(cfg)
    active = neutral_packet()
    active["right"]["grip"] = True
    step(control, active)
    control.disconnect()
    control.tick(time.perf_counter(), .02)
    step(control, neutral_packet())
    control.resume_since = time.perf_counter()-.31
    bad = neutral_packet()
    bad["right"]["tracked"] = False
    step(control, bad)
    assert control.resume_since is None
    step(control, neutral_packet())
    assert control.mode == "paused" and not control.ready


def test_idle_controller_can_disappear_while_other_arm_keeps_following(cfg, monkeypatch):
    control = make_controller(cfg)
    sent = []
    monkeypatch.setattr(control.robots["right"], "send", lambda target: sent.append(target))
    step(control, neutral_packet())
    data = neutral_packet()
    data["right"]["grip"] = True
    step(control, data)
    data["left"].update(tracked=False, tracking_reason="controller_missing")
    data["right"]["position"][0] += .02
    step(control, data)
    assert control.mode == "teleop" and control.ready and len(sent) == 1


def test_idle_pickup_never_pauses_and_requires_release_before_new_control(cfg, monkeypatch):
    control = make_controller(cfg)
    step(control, neutral_packet())
    data = neutral_packet()
    data["right"].update(tracked=False, grip=True, tracking_reason="pose_missing")
    step(control, data)
    assert control.mode != "paused" and not control.enabled["right"]
    data["right"]["tracked"] = True
    step(control, data)
    assert not control.enabled["right"]
    data["right"]["grip"] = False
    step(control, data)
    data["right"]["grip"] = True
    step(control, data)
    assert control.enabled["right"]


def test_x_saves_home_when_unused_right_controller_is_missing(cfg, tmp_path):
    control = make_controller(cfg)
    path = tmp_path/"config.json"
    path.write_text(json.dumps(cfg), encoding="utf-8")
    control.mock, control.config_path = False, path
    data = neutral_packet()
    data["right"].update(tracked=False, tracking_reason="controller_missing")
    data["buttons"]["set_home"] = True
    step(control, data)
    control.hold_since["set_home"] = time.perf_counter()-2.1
    step(control, data)
    assert control.mode == "idle" and "saved as home" in control.message
    assert json.loads(path.read_text())["arms"]["right"]["home_waypoints"] == [[0.]*6]


def test_missing_controller_still_blocks_record_start(cfg):
    control = make_controller(cfg)
    data = neutral_packet()
    data["right"].update(tracked=False, tracking_reason="controller_missing")
    data["buttons"]["record_start"] = True
    step(control, data)
    assert control.recorder.path is None and "both controllers" in control.message


def test_active_hand_pause_can_resume_with_unused_other_hand_missing(cfg):
    control = make_controller(cfg)
    data = neutral_packet()
    data["left"]["tracked"] = False
    data["right"]["grip"] = True
    step(control, data)
    data["right"]["tracked"] = False
    step(control, data)
    assert control.mode == "paused" and control.resume_hands == {"right"}
    data["right"].update(tracked=True, grip=False)
    step(control, data)
    control.resume_since = time.perf_counter()-.31
    step(control, data)
    assert control.mode == "idle"


def test_camera_failure_during_input_pause_still_latches_fault(cfg, monkeypatch):
    control = make_controller(cfg)
    step(control, neutral_packet())
    control.disconnect()
    control.tick(time.perf_counter(), .02)
    stopped = []
    for side, robot in control.robots.items():
        monkeypatch.setattr(robot, "stop", lambda side=side: stopped.append(side))
    control.cameras.errors["head"] = "camera unplugged"
    step(control, neutral_packet())
    assert control.mode == "fault" and set(stopped) == set(ARMS)
    control.cameras.errors.clear()
    step(control, neutral_packet())
    assert control.mode == "fault"


@pytest.mark.parametrize("failure", ["hold", "read"])
def test_pause_command_or_robot_feedback_error_falls_back_to_fault(cfg, monkeypatch, failure):
    control = make_controller(cfg)
    data = neutral_packet()
    data["right"]["grip"] = True
    step(control, data)
    stopped = []
    for side, robot in control.robots.items():
        monkeypatch.setattr(robot, "stop", lambda side=side: stopped.append(side))
    def fail(*_):
        control.stop_event.set()
        raise RuntimeError("robot failure")
    monkeypatch.setattr(control.robots["right"], failure, fail)
    control.disconnect()
    control._run()
    assert control.mode == "fault" and set(stopped) == set(ARMS)


def test_pause_cancels_home_and_discards_recording_buttons(cfg):
    control = make_controller(cfg)
    for side in ARMS:
        cfg["arms"][side]["home_waypoints"] = [[.2]*6]
    step(control, neutral_packet())
    control._begin_reset(time.perf_counter())
    control.disconnect()
    control.tick(time.perf_counter(), .02)
    assert control.mode == "paused" and control.reset_jobs == [] and control.reset_active is None
    data = neutral_packet()
    data["buttons"]["record_start"] = True
    step(control, data)
    assert control.recorder.path is None and control.resume_since is None
    resume(control)
    step(control, neutral_packet())
    assert control.recorder.path is None


def test_zero_pose_cannot_be_saved_or_used_as_working_home(cfg, tmp_path, monkeypatch):
    control = make_controller(cfg)
    path = tmp_path/"config.json"
    for side in ARMS:
        cfg["arms"][side]["home_waypoints"] = [[0]*6]
    path.write_text(json.dumps(cfg), encoding="utf-8")
    control.mock, control.config_path = False, path
    monkeypatch.setattr(control.robots["right"], "cartesian_start_issue", lambda _: "J2 at limit")
    step(control, neutral_packet())
    before = path.read_bytes()
    control._save_home({side: h.latest().value for side, h in control.states.items()})
    assert path.read_bytes() == before and "Home not saved" in control.message
    control._begin_reset(time.perf_counter())
    assert control.mode == "idle" and "Home needs a working pose" in control.message
