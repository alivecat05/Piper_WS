"""Behavior checks for timing, controller state transitions, and dataset integrity."""
import copy
import json
from pathlib import Path
import time
from threading import Event

import numpy as np
import pytest

from piperx_teleop.app import load_config, validate_packet
from piperx_teleop.cameras import Cameras
from piperx_teleop.controller import Controller
from piperx_teleop.core import ARMS, CAMERAS, History, PoseMapper, Synchronizer
from piperx_teleop.export import read_segments
from piperx_teleop.recording import Recorder
from piperx_teleop.robot import MockPiperX

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def cfg(tmp_path):
    result = load_config(ROOT / "config.example.json")
    result["feedback"] = {"sound": False, "haptics": False}
    result["output"] = str(tmp_path / "episodes")
    for arm in result["arms"].values():
        arm["home_waypoints"] = []  # Tests must not depend on the operator's saved working poses.
    return result


def packet(**buttons):
    hand = {"tracked": True, "grip": False, "trigger": .4,
            "position": [0., 1., 0.], "quaternion": [0., 0., 0., 1.]}
    return {"type": "input", "active": True, "left": copy.deepcopy(hand), "right": copy.deepcopy(hand),
            "buttons": {key: buttons.get(key, False) for key in ("record_start", "record_stop", "reset", "set_home")}}


def synced_sample(cfg, t=10.):
    cameras = {name: History() for name in CAMERAS}
    states, actions = {side: History() for side in ARMS}, {side: History() for side in ARMS}
    vector = [.3, 0., .3, 0., 0., 0., 1., .04]
    for name, h in cameras.items():
        c = cfg["cameras"][name]
        h.add(t, {"bgr": np.zeros((c["height"], c["width"], 3), np.uint8), "frame_number": 1})
    for side in ARMS:
        states[side].add(t, {"vector": vector, "joints": [0.]*6,
                             "source_timestamps": {"tcp": t, "gripper": t}})
        actions[side].add(t-.01, vector)
    return Synchronizer(cameras, states, actions, cfg["sync"]), cameras, states, actions


def test_causal_commands_and_no_duplicate_images(cfg):
    sync, _, _, actions = synced_sample(cfg)
    future = [.5, 0., .3, 0., 0., 0., 1., .01]
    actions["left"].add(10.001, future)
    sample = sync.sample(10.)
    assert sample["action"][0] == .3  # Future command must not leak into the label.
    assert sync.sample(10.) is None


def test_missing_or_skewed_sources_drop_sample(cfg):
    sync, cameras, states, _ = synced_sample(cfg)
    cameras["right_wrist"] = History()
    assert sync.sample(10.) is None
    sync, cameras, _, _ = synced_sample(cfg)
    # Both within per-camera tolerance, but their mutual span exceeds 25 ms.
    for name, t in (("head", 9.981), ("right_wrist", 10.019)):
        old = cameras[name].latest().value
        cameras[name] = History()
        cameras[name].add(t, old)
    assert sync.sample(10.) is None
    sync, _, states, _ = synced_sample(cfg)
    states["left"].latest().value["source_timestamps"]["gripper"] = 9.
    assert sync.sample(10.) is None


def test_clutch_reanchors_and_bounds_motion(cfg):
    mapper = PoseMapper(cfg["arms"]["left"])
    state = [.3, 0., .3, 0., 0., 0., 1., .04]
    hand = packet()["left"]
    hand["grip"] = True
    assert mapper.update(hand, state, .02)[:7] == state[:7]
    hand["position"][2] = -.5  # Forward in XR maps to +X in robot base.
    moved = mapper.update(hand, state, .02)
    assert 0 < moved[0] - state[0] <= .12*.02+1e-9
    assert np.isclose(np.linalg.norm(moved[3:7]), 1)
    hand["grip"] = False
    assert mapper.update(hand, state, .02)[:7] == state[:7]
    hand["position"][2] = -4
    hand["grip"] = True
    assert mapper.update(hand, state, .02)[:7] == state[:7]


def test_clutch_ignores_absolute_head_or_chest_controller_pose(cfg):
    state = [.3, 0., .3, 0., 0., 0., 1., .04]
    targets = []
    for position, quaternion in (([0., 1.6, -.3], [0., 0., 0., 1.]),
                                 ([.2, .8, -.6], [1., 0., 0., 0.])):
        mapper = PoseMapper(cfg["arms"]["right"])
        hand = packet()["right"]
        hand.update(grip=True, position=position, quaternion=quaternion)
        np.testing.assert_allclose(mapper.update(hand, state, .02)[:7], state[:7])
        hand["position"][2] -= .01
        targets.append(mapper.update(hand, state, .02))
    np.testing.assert_allclose(targets[0], targets[1])


@pytest.mark.parametrize("grip", [False, True])
@pytest.mark.parametrize("dt", [.001, .05])
def test_gripper_follows_trigger_proportionally_without_time_lag(cfg, grip, dt):
    mapper = PoseMapper(cfg["arms"]["right"])
    state = [.3, 0., .3, 0., 0., 0., 1., .04]
    hand = packet()["right"]
    hand["grip"] = grip
    # Slow squeeze, fast squeeze, hold, and immediate reversal all use the current trigger.
    for trigger in [0., .1, .12, .5, 1., 1., .25, 0.]:
        hand["trigger"] = trigger
        target = mapper.update(hand, state, dt)
        assert target[7] == pytest.approx((1-trigger)*.07)
        np.testing.assert_allclose(target[:7], state[:7])


def test_frequency_is_measured_and_expires():
    h = History()
    for i in range(31):
        h.add(10+i/30, i)
    assert h.fps(11.) == pytest.approx(30.)
    assert h.fps(12.) == 0
    h.add(9., 4)
    assert len(h.snapshot()) == 31


def test_invalid_packet_rejected():
    data = packet()
    data["right"]["quaternion"] = [0, 0, 0, 0]
    with pytest.raises(ValueError):
        validate_packet(data)


def make_controller(cfg):
    cameras = Cameras(cfg, mock=True)
    robots = {side: MockPiperX(cfg["arms"][side]) for side in ARMS}
    return Controller(cfg, robots, cameras, mock=True)


def feed_cameras(controller, now):
    """Inject fresh camera frames for unit tests that don't run camera workers."""
    for name, history in controller.cameras.histories.items():
        cfg = controller.cfg["cameras"][name]
        previous = history.latest()
        history.add(now, {"bgr": np.zeros((cfg["height"], cfg["width"], 3), np.uint8),
                          "frame_number": previous.value["frame_number"] + 1 if previous else 0})


def step(controller, data, delta=.02, camera_frames=True):
    now = time.perf_counter()
    if camera_frames and not controller.cameras.threads:
        feed_cameras(controller, now)
    controller.accept(data, now)
    controller.tick(now, delta)


def test_grip_starts_motion_without_enable_button_and_tracking_loss_stops(cfg):
    control = make_controller(cfg)
    step(control, packet())
    assert control.mode == "idle"
    data = packet()
    data["right"]["grip"] = True
    step(control, data)
    assert control.mode == "teleop"
    assert control.enabled == {"left": False, "right": True}
    data = packet()
    data["left"]["tracked"] = False
    step(control, data)
    assert control.mode == "paused"
    step(control, packet())
    assert control.mode == "paused"


def test_timeout_stops_both_arms(cfg):
    control = make_controller(cfg)
    step(control, packet())
    control.accept(packet(), time.perf_counter()-1)
    control.tick(time.perf_counter(), .02)
    assert control.mode == "paused"


def test_released_grips_required_for_home(cfg):
    control = make_controller(cfg)
    data = packet()
    data["left"]["grip"] = True
    step(control, data)
    data = packet(reset=True)
    data["left"]["grip"] = True
    control.hold_since["reset"] = time.perf_counter()-2
    step(control, data)
    assert control.mode == "teleop"
    step(control, packet(reset=True))
    control.hold_since["reset"] = time.perf_counter()-2
    step(control, packet(reset=True))
    assert "home_waypoints" in control.message


def test_writer_and_export_preserve_gaps(cfg):
    sync, _, _, _ = synced_sample(cfg)
    row = sync.sample(10.)
    recorder = Recorder(cfg["output"], cfg, mock=True)
    recorder.start(10.)
    for i in (0, 1, 4, 5):
        item = dict(row, timestamp_monotonic_s=10+i/30)
        recorder.add(item, i)
    recorder.stop()
    assert recorder.wait(timeout=3)
    meta = json.loads((recorder.path/"manifest.json").read_text(encoding="utf-8"))
    assert meta["status"] == "complete" and meta["frames"] == 4
    with pytest.raises(ValueError, match="Mock data"):
        read_segments(cfg["output"])
    segments = read_segments(cfg["output"], allow_mock=True)
    assert [len(rows) for _, _, rows in segments] == [2, 2]
    (recorder.path/"images"/"head"/"00000000.jpg").unlink()
    with pytest.raises(ValueError, match="Missing or invalid image"):
        read_segments(cfg["output"], allow_mock=True)


def test_failed_episode_is_not_exported(cfg):
    sync, _, _, _ = synced_sample(cfg)
    recorder = Recorder(cfg["output"], cfg, mock=True)
    recorder.start(10.)
    recorder.add(sync.sample(10.), 0)
    recorder.stop("tracking_lost", complete=False)
    assert recorder.wait(timeout=3)
    with pytest.raises(ValueError, match="No complete"):
        read_segments(cfg["output"], allow_mock=True)


def test_three_camera_mock_pipeline(cfg):
    cameras = Cameras(cfg, mock=True)
    cameras.start()
    try:
        time.sleep(.25)
        assert not cameras.errors
        assert all(history.latest() for history in cameras.histories.values())
        assert all(15 < history.fps(time.perf_counter()) < 40 for history in cameras.histories.values())
    finally:
        cameras.close()


def test_complete_teleop_record_and_home(cfg):
    control = make_controller(cfg)
    for side in ARMS:
        cfg["arms"][side]["home_waypoints"] = [[0.]*6]
    control.cameras.start()
    try:
        time.sleep(.15)
        step(control, packet())
        step(control, packet(record_start=True))
        until = time.perf_counter()+1.3
        while time.perf_counter() < until:
            step(control, packet())
            time.sleep(1/60)
        step(control, packet(record_stop=True))
        assert not control.status()["recording"] and control.status()["saving"]
        until = time.perf_counter()+2
        while control.recorder.busy and time.perf_counter() < until:
            step(control, packet())
            time.sleep(.01)
        assert not control.recorder.busy
        assert control.recorder.count >= 20
        assert read_segments(cfg["output"], allow_mock=True)
        step(control, packet())
        step(control, packet(reset=True))
        control.hold_since["reset"] = time.perf_counter()-2
        step(control, packet(reset=True))
        assert control.mode == "resetting"
        until = time.perf_counter()+1.
        while time.perf_counter() < until and control.mode == "resetting":
            step(control, packet())
            time.sleep(.02)
        assert control.mode == "idle"
    finally:
        control.close()
        control.cameras.close()


def test_adapter_reads_real_sdk_status_types(cfg):
    from types import SimpleNamespace as NS
    from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
    from pyAgxArm.protocols.can_protocol.msgs.effector.agx_gripper.default.feedback.arm_feedback_gripper import ArmMsgFeedbackGripper
    from piperx_teleop.robot import PiperX
    adapter = PiperX.__new__(PiperX)
    adapter.cfg = cfg["arms"]["left"]
    adapter.clock_offset = time.perf_counter()-time.time()
    stamp = time.time()
    msg = lambda value: NS(timestamp=stamp, msg=value)
    parser = NS(**{key: msg(None) for key in ("end_pose_xy", "end_pose_zrx", "end_pose_ryrz",
                                           "joint_12", "joint_34", "joint_56")})
    adapter.robot = NS(get_tcp_pose=lambda: msg([.3, 0, .3, 0, 0, 0]),
        get_joint_angles=lambda: msg([0.]*6), get_arm_status=lambda: msg(ArmMsgFeedbackStatus()), _parser=parser)
    adapter.gripper = NS(get_gripper_status=lambda: msg(ArmMsgFeedbackGripper(value=.04)))
    t, state = adapter.read()
    assert state["vector"][7] == .04 and abs(t-time.perf_counter()) < .1


def test_home_can_be_taught_and_persisted(cfg, tmp_path):
    control = make_controller(cfg)
    config_path = tmp_path/"config.json"
    config_path.write_text(json.dumps(cfg), encoding="utf-8")
    control.mock, control.config_path = False, config_path
    control.robots["left"].joints = [.1, 0, .3, 0, 0, 0]
    step(control, packet(set_home=True))
    control.hold_since["set_home"] = time.perf_counter()-3
    step(control, packet(set_home=True))
    saved = json.loads(config_path.read_text(encoding="utf-8"))
    assert saved["arms"]["left"]["home_waypoints"] == [[.1, 0, .3, 0, 0, 0]]
    data = packet()
    data["left"]["grip"] = True
    step(control, data)
    before = config_path.read_bytes()
    control._save_home({side: history.latest().value for side, history in control.states.items()})
    assert config_path.read_bytes() == before  # Teaching is forbidden during active control.


def test_record_start_and_stop_are_independent_of_grips(cfg):
    control = make_controller(cfg)
    step(control, packet(record_start=True))
    assert control.recorder.active
    step(control, packet())
    step(control, packet(record_start=True))
    assert control.recorder.active  # B cannot toggle recording off.
    data = packet()
    data["left"]["grip"] = True
    step(control, data)
    assert control.mode == "teleop" and control.recorder.active
    step(control, packet())
    assert control.mode == "idle" and control.recorder.active  # Release never ends recording.
    data["buttons"]["record_stop"] = True
    step(control, data)
    assert control.mode == "teleop" and not control.status()["recording"]  # A doesn't stop teleop.
    assert control.status()["saving"]
    control.close()


def test_trigger_operates_gripper_with_grip_released(cfg):
    control = make_controller(cfg)
    data = packet()
    data["left"]["trigger"] = 1.
    for _ in range(60):
        step(control, data)
    assert control.robots["left"].vector[7] == pytest.approx(0.)
    assert control.mode == "idle"
    data["left"]["trigger"] = 0.
    for _ in range(100):
        step(control, data)
    assert control.robots["left"].vector[7] == pytest.approx(cfg["arms"]["left"]["gripper_max_m"])


def test_startup_requires_all_cameras_and_both_feedbacks(cfg, monkeypatch):
    control = make_controller(cfg)
    read_right = control.robots["right"].read
    monkeypatch.setattr(control.robots["right"], "read", lambda: None)
    data = packet(record_start=True)
    data["left"]["grip"] = True
    step(control, data, camera_frames=False)
    assert control.mode == "waiting" and not control.status()["ready"]
    assert "right feedback" in control.message and "RGB head" in control.message
    assert all(not history.snapshot() for history in control.actions.values())
    assert not any(control.enabled.values()) and control.recorder.path is None
    feed_cameras(control, time.perf_counter())
    step(control, data, camera_frames=False)
    assert control.mode == "waiting" and "RGB" not in control.message
    monkeypatch.setattr(control.robots["right"], "read", read_right)
    step(control, data)
    assert control.status()["ready"] and control.mode == "teleop"
    assert not control.status()["recording"]  # B held during startup isn't queued.
    step(control, packet())
    step(control, packet(record_start=True))
    assert control.status()["recording"]
    control.close()


def test_stalled_camera_stops_control_before_driver_timeout(cfg, monkeypatch):
    control = make_controller(cfg)
    stopped = []
    for side, robot in control.robots.items():
        monkeypatch.setattr(robot, "stop", lambda side=side: stopped.append(side))
    step(control, packet())
    image = control.cameras.histories["head"].latest().value
    history = History()
    history.add(time.perf_counter() - cfg["camera_timeout_s"] - .1, image)
    control.cameras.histories["head"] = history
    step(control, packet(), camera_frames=False)
    assert not control.cameras.errors  # The camera worker hasn't raised an error yet.
    assert control.mode == "fault" and "RGB head" in control.message
    assert set(stopped) == set(ARMS)


def test_input_arriving_during_feedback_read_is_not_rejected(cfg, monkeypatch):
    control = make_controller(cfg)
    step(control, packet())
    original_read = control.robots["left"].read
    data = packet()
    data["right"]["grip"] = True
    def read_with_new_input():
        control.accept(data)  # Arrives after tick's initial timestamp.
        return original_read()
    monkeypatch.setattr(control.robots["left"], "read", read_with_new_input)
    control.tick(time.perf_counter(), .02)
    assert control.mode == "teleop" and control.enabled["right"]


def test_stop_flushes_tail_up_to_button_time_only(cfg):
    control = make_controller(cfg)
    sync, cameras, states, actions = synced_sample(cfg)
    for i in range(1, 6):
        t = 10 + i/30
        for history in cameras.values():
            history.add(t, dict(history.latest().value, frame_number=i+1))
        for history in states.values():
            history.add(t, dict(history.latest().value, source_timestamps={"tcp": t, "gripper": t}))
        for history in actions.values():
            history.add(t-.01, history.latest().value)
    control.sync = sync
    control.recorder.start(10.)
    control.next_record_t, control.grid_index = 10., 0
    control.record_end_t = 10.075
    control._record_tick(10.07)
    assert control.grid_index == 0 and control.status()["saving"]
    control._record_tick(10.12)
    assert control.grid_index == 2 and control.recorder.active
    control._record_tick(10.16)
    assert control.recorder.wait(timeout=3)
    rows = [json.loads(line) for line in (control.recorder.path/"samples.jsonl").read_text().splitlines()]
    assert [row["grid_index"] for row in rows] == [0, 1, 2]
    assert rows[-1]["timestamp_monotonic_s"] == pytest.approx(10+2/30)
    assert control.recorder.dropped == 0
    meta = json.loads((control.recorder.path/"manifest.json").read_text())
    assert meta["status"] == "complete" and meta["end_monotonic_s"] == 10.075
    assert len(read_segments(cfg["output"], allow_mock=True)[0][2]) == 3


def test_slow_save_keeps_motion_and_watchdog_responsive(cfg, monkeypatch):
    control = make_controller(cfg)
    blocked, release = Event(), Event()
    original_manifest = control.recorder._manifest
    def slow_manifest(status, reason):
        if status != "recording":
            blocked.set()
            if not release.wait(3):
                raise TimeoutError("Test failed to release the disk writer")
        original_manifest(status, reason)
    monkeypatch.setattr(control.recorder, "_manifest", slow_manifest)
    stopped = []
    for side, robot in control.robots.items():
        monkeypatch.setattr(robot, "stop", lambda side=side: stopped.append(side))
    try:
        step(control, packet())
        step(control, packet(record_start=True))
        data = packet(record_stop=True)
        data["left"]["grip"] = True
        step(control, data)
        assert control.record_end_t is not None and control.status()["saving"]
        time.sleep(.09)
        data["buttons"]["record_stop"] = False
        step(control, data)
        assert blocked.wait(1)  # Disk is still busy finalizing; control.tick has returned.
        before = control.robots["left"].vector[0]
        data["left"]["position"][2] -= .05
        step(control, data)
        assert control.robots["left"].vector[0] > before
        path = control.recorder.path
        data["buttons"]["record_start"] = True
        step(control, data)
        assert control.recorder.path == path and not control.status()["recording"]
        control.accept(data, time.perf_counter()-1)
        control.tick(time.perf_counter(), .02)
        assert control.mode == "paused" and stopped == []
    finally:
        release.set()
        control.close()


def test_full_writer_queue_can_be_closed_without_waiting(cfg, monkeypatch):
    recorder = Recorder(cfg["output"], cfg, mock=True)
    blocked, release = Event(), Event()
    original = recorder._write
    def delayed_writer():
        blocked.set()
        release.wait(3)
        original()
    monkeypatch.setattr(recorder, "_write", delayed_writer)
    try:
        recorder.start(10.)
        assert blocked.wait(1)
        for i in range(60):
            recorder.add({"images": {}, "timestamp_monotonic_s": 10+i/30}, i)
        start = time.perf_counter()
        recorder.stop(end_time=12.)
        assert time.perf_counter()-start < .2
        assert recorder.busy and not recorder.active
    finally:
        release.set()
        assert recorder.wait(timeout=3)
    assert recorder.result == "complete" and recorder.count == 60


def test_input_loss_during_tail_marks_episode_incomplete(cfg):
    control = make_controller(cfg)
    step(control, packet(record_start=True))
    step(control, packet(record_stop=True))
    assert control.recorder.active and control.status()["saving"]
    control.disconnect()
    control.tick(time.perf_counter(), .02)
    assert control.mode == "paused"
    assert control.recorder.wait(timeout=3)
    meta = json.loads((control.recorder.path/"manifest.json").read_text())
    assert meta["status"] == "incomplete" and "Lost" in meta["reason"]


def test_y_cancels_home_at_measured_joints_and_allows_teleop(cfg, monkeypatch):
    control = make_controller(cfg)
    sent_home, held = [], {}
    for side, robot in control.robots.items():
        cfg["arms"][side]["home_waypoints"] = [[.1]*6, [.2]*6]
        move_home, hold = robot.move_home, robot.hold
        def tracked_home(joints, side=side, move=move_home):
            sent_home.append(side)
            move(joints)
        def tracked_hold(joints, side=side, move=hold):
            held[side] = list(joints)
            move(joints)
        monkeypatch.setattr(robot, "move_home", tracked_home)
        monkeypatch.setattr(robot, "hold", tracked_hold)
    step(control, packet(reset=True))
    control.hold_since["reset"] = time.perf_counter()-2
    step(control, packet(reset=True))
    step(control, packet(reset=True))
    assert control.mode == "resetting" and sent_home == ["left"]
    step(control, packet())  # Release Y before pressing again.
    control.robots["left"].joints = [.05]*6
    control.robots["right"].joints = [-.03]*6
    step(control, packet(reset=True))
    assert held == {"left": [.05]*6, "right": [-.03]*6}
    assert control.mode == "idle" and not control.reset_jobs
    for _ in range(4):
        step(control, packet(reset=True))
    assert sent_home == ["left"] and control.mode == "idle"
    data = packet()
    data["left"]["grip"] = True
    step(control, data)
    assert control.mode == "teleop" and control.enabled["left"]


def test_adapter_hold_overrides_joint_target_without_disabling():
    from types import SimpleNamespace
    from piperx_teleop.robot import PiperX
    commands = []
    adapter = PiperX.__new__(PiperX)
    adapter.robot = SimpleNamespace(move_j=lambda joints: commands.append(joints),
        get_joint_limits_enabled=lambda: True,
        get_config=lambda: {"joint_limits": {f"joint{i}": [-1, 1] for i in range(1, 7)}})
    adapter.hold([.05]*6)
    assert commands == [[.05]*6]


def test_adapter_hold_does_not_silently_clamp_current_joints():
    from types import SimpleNamespace
    from piperx_teleop.robot import PiperX
    adapter = PiperX.__new__(PiperX)
    adapter.robot = SimpleNamespace(move_j=lambda *_: pytest.fail("Invalid hold was sent"),
        get_joint_limits_enabled=lambda: True,
        get_config=lambda: {"joint_limits": {f"joint{i}": [0, 1] for i in range(1, 7)}})
    with pytest.raises(ValueError, match="J2"):
        adapter.hold([.1, -.01, .1, .1, .1, .1])


def test_adapter_hold_preserves_measured_angles_with_sdk_limits_inactive():
    from types import SimpleNamespace
    from piperx_teleop.robot import PiperX
    commands = []
    adapter = PiperX.__new__(PiperX)
    adapter.robot = SimpleNamespace(move_j=lambda joints: commands.append(joints),
        get_joint_limits_enabled=lambda: False,
        get_config=lambda: {"joint_limits": {f"joint{i}": [0, 1] for i in range(1, 7)}})
    measured = [.1, -.0332, .014, .1, .1, .1]
    adapter.hold(measured)
    assert commands == [measured]


@pytest.mark.parametrize("joints", [[0]*5, [0]*7, [0, float("nan"), 0, 0, 0, 0],
                                    [0, float("inf"), 0, 0, 0, 0]])
def test_adapter_hold_rejects_invalid_feedback_even_without_sdk_limits(joints):
    from types import SimpleNamespace
    from piperx_teleop.robot import PiperX
    adapter = PiperX.__new__(PiperX)
    adapter.robot = SimpleNamespace(move_j=lambda *_: pytest.fail("Invalid hold was sent"),
                                   get_joint_limits_enabled=lambda: False)
    with pytest.raises(ValueError):
        adapter.hold(joints)


def test_middle_button_holds_before_enable_then_follows_only_after_motion(cfg, monkeypatch):
    control = make_controller(cfg)
    robot = control.robots["right"]
    calls = []
    hold, enable, send = robot.hold, robot.enable, robot.send
    monkeypatch.setattr(robot, "hold", lambda joints: (calls.append("hold"), hold(joints))[-1])
    monkeypatch.setattr(robot, "enable", lambda: (calls.append("enable"), enable())[-1])
    monkeypatch.setattr(robot, "send", lambda target: (calls.append("cartesian"), send(target))[-1])
    monkeypatch.setattr(control.robots["left"], "send", lambda *_: pytest.fail("Idle left arm got move_p"))
    step(control, packet())
    assert calls == []
    data = packet()
    data["right"]["grip"] = True
    step(control, data)
    assert calls == ["hold", "enable"]
    data["right"]["position"][0] += .0005
    step(control, data)
    assert calls == ["hold", "enable"]  # Small initial XR jitter must not trigger IK.
    data["right"]["position"][0] += .01
    step(control, data)
    assert calls[-1] == "cartesian"
    robot.vector[:3] = [.31, .02, .32]  # Release captures measured position, not old target.
    step(control, packet())
    assert calls[-1] == "hold"
    np.testing.assert_allclose(control.actions["right"].latest().value[:3], [.31, .02, .32])
    count = len(calls)
    step(control, packet())
    assert len(calls) == count
    step(control, data)  # New press starts at a new anchor, without stale delta replay.
    assert calls[-1] == "hold"
    assert len(calls) == count+1


def test_no_cartesian_command_while_enable_is_pending(cfg, monkeypatch):
    control = make_controller(cfg)
    robot = control.robots["right"]
    monkeypatch.setattr(robot, "enable", lambda: False)
    monkeypatch.setattr(robot, "send", lambda *_: pytest.fail("move_p before enable acknowledgement"))
    data = packet()
    data["right"]["grip"] = True
    for _ in range(5):
        step(control, data)
        data["right"]["position"][0] += .01
    assert not control.enabled["right"] and control.mappers["right"].anchor is None


def test_folded_start_blocks_cartesian_but_gripper_still_works(cfg, monkeypatch):
    control = make_controller(cfg)
    robot = control.robots["right"]
    monkeypatch.setattr(robot, "cartesian_start_issue", lambda _: "J2 outside start range")
    monkeypatch.setattr(robot, "send", lambda *_: pytest.fail("Blocked pose reached move_p"))
    monkeypatch.setattr(robot, "hold", lambda *_: pytest.fail("Blocked press sent a joint target"))
    monkeypatch.setattr(robot, "enable", lambda: pytest.fail("Blocked press enabled arm"))
    data = packet()
    data["right"].update(grip=True, trigger=1)
    step(control, data)
    data["right"]["position"][0] += .03
    for _ in range(60):
        step(control, data)
    assert control.mode != "fault" and not control.enabled["right"]
    assert robot.vector[7] == pytest.approx(0)
    assert control.cartesian_blocked["right"]
    assert not control.motion_active["right"]
    assert control.gripper_status()["right"]["target_mm"] == 0
    data["right"].update(grip=False, trigger=0)
    for _ in range(100):
        step(control, data)
    assert robot.vector[7] == pytest.approx(.07)
    assert not control.cartesian_blocked["right"]


def test_press_jitter_does_not_start_cartesian_following(cfg, monkeypatch):
    from scipy.spatial.transform import Rotation
    control = make_controller(cfg)
    calls = []
    monkeypatch.setattr(control.robots["right"], "send", lambda target: calls.append(target))
    data = packet()
    data["right"]["grip"] = True
    step(control, data)
    data["right"]["position"][0] += .003
    data["right"]["quaternion"] = Rotation.from_euler("z", 2, degrees=True).as_quat().tolist()
    for _ in range(10):
        step(control, data)
    assert not calls
    data["right"]["position"][0] += .003
    step(control, data)
    assert len(calls) == 1


@pytest.mark.parametrize("joints,blocked", [([0, -.0332, .0419, 0, 0, 0], True),
                                         ([0, 0, 0, .6265, .0273, .0673], True),
                                         ([0, .01, -.01, 0, 0, 0], True),
                                         ([3.13, .5, -.5, 0, 0, 0], True),
                                         ([0, .5, -.5, 0, 0, 0], False)])
def test_cartesian_start_checks_pose_even_when_joint_hold_is_allowed(joints, blocked):
    from types import SimpleNamespace
    from piperx_teleop.robot import PiperX
    adapter = PiperX.__new__(PiperX)
    limits = {f"joint{i}": [-3.14, 3.14] for i in range(1, 7)}
    limits.update(joint2=[0, 3.14], joint3=[-2.97, 0])
    adapter.robot = SimpleNamespace(get_config=lambda: {"joint_limits": limits})
    assert bool(adapter.cartesian_start_issue(joints)) == blocked


def test_gripper_command_includes_enable_without_enabling_arm():
    from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.driver import Driver
    from piperx_teleop.robot import PiperX
    sent = []
    gripper = Driver.__new__(Driver)
    gripper._send_msg = sent.append
    adapter = PiperX.__new__(PiperX)
    adapter.cfg = {"gripper_force_n": 1.0}
    adapter.gripper = gripper
    adapter.send_gripper(.012)
    assert len(sent) == 1
    assert sent[0].value == 12000 and sent[0].force == 1000
    assert sent[0].status_code == 1


def test_index_trigger_only_sends_gripper_commands(cfg, monkeypatch):
    control = make_controller(cfg)
    for robot in control.robots.values():
        monkeypatch.setattr(robot, "send", lambda *_: pytest.fail("Index trigger sent arm motion"))
        monkeypatch.setattr(robot, "enable", lambda: pytest.fail("Index trigger enabled arm"))
        monkeypatch.setattr(robot, "hold", lambda *_: pytest.fail("Index trigger sent joint target"))
    data = packet()
    data["right"]["trigger"] = 1
    for _ in range(60):
        step(control, data)
    assert control.robots["right"].vector[7] == pytest.approx(0)


@pytest.mark.parametrize("reason,side,expected", [
    ("session_blurred", None, "Quest session_blurred"),
    ("server_ack_stale", None, "Quest server_ack_stale"),
    ("pose_missing", "left", "Quest left: pose_missing"),
    ("position_emulated", "right", "Quest right: position_emulated"),
])
def test_tracking_pause_names_exact_reason(cfg, reason, side, expected):
    control = make_controller(cfg)
    step(control, packet())
    data = packet()
    if side:
        data[side]["tracked"] = False
        data[side]["tracking_reason"] = reason
    else:
        data["active"] = False
        data["inactive_reason"] = reason
    step(control, data)
    assert control.mode == "paused" and expected in control.message
    step(control, packet())
    assert control.mode == "paused"  # Held triggers do not rearm control.


def test_input_status_shows_middle_and_index_separately(cfg):
    control = make_controller(cfg)
    control.begin_input()
    data = packet()
    data["right"]["grip"] = True
    data["right"]["trigger"] = .8
    control.accept(data)
    quest = control.status()["quest"]
    assert quest["connected"] and quest["active"] and quest["age_ms"] is not None
    assert quest["right"]["middle"] and quest["right"]["trigger"] == .8
    assert not quest["left"]["middle"]


def test_input_timeout_is_distinct_from_tracking_loss(cfg):
    control = make_controller(cfg)
    step(control, packet())
    control.accept(packet(), time.perf_counter()-1)
    control.tick(time.perf_counter(), .02)
    assert "Quest input timeout" in control.message


def test_diagnostic_counts_capture_button_edges_without_enabling_motion(cfg):
    control = make_controller(cfg)
    control.begin_input()
    data = packet()
    data["right"].update(tracked=False, grip=True, trigger=1., tracking_reason="pose_missing")
    control.accept(data)
    control.accept(data)
    control.tick(time.perf_counter(), .02)
    hand = control.status()["quest"]["right"]
    assert hand["presses"] == {"middle": 1, "trigger": 1}
    assert control.mode == "waiting" and not any(control.enabled.values())
    assert not control.actions["right"].snapshot()
    control.begin_input()
    assert control.status()["quest"]["right"]["presses"] == {"middle": 0, "trigger": 0}


def test_short_button_taps_survive_pose_replacement_and_retries(cfg):
    control = make_controller(cfg)
    step(control, packet())
    data = packet()
    data["events"] = [{"id": 1, "name": "record_start", "down": True, "age_ms": 0},
                      {"id": 2, "name": "record_start", "down": False, "age_ms": 0}]
    control.accept(data)
    control.accept(data)  # Retransmit while waiting for the execution acknowledgement.
    latest = packet()
    latest["events"] = []
    control.accept(latest)  # Latest pose replaces the packet, not the button events.
    feed_cameras(control, time.perf_counter())
    control.tick(time.perf_counter(), .02)
    assert control.status()["recording"] and control.event_ack == 2
    assert [n["kind"] for n in control.feedback.snapshot()] == ["record_started"]
    path = control.recorder.path
    control.accept(data)
    control.tick(time.perf_counter(), .02)
    assert control.recorder.path == path
    assert len(control.feedback.snapshot()) == 1
    control.close()


def test_ordered_start_stop_events_in_one_control_tick(cfg):
    control = make_controller(cfg)
    step(control, packet())
    data = packet()
    data["events"] = [{"id": i+1, "name": name, "down": down, "age_ms": age}
                      for i, (name, down, age) in enumerate([
                          ("record_start", True, 30), ("record_start", False, 20),
                          ("record_stop", True, 10), ("record_stop", False, 0)])]
    step(control, data)
    assert control.status()["saving"] and control.event_ack == 4
    assert control.record_end_t-control.recorder.start_time == pytest.approx(.02)
    assert [n["kind"] for n in control.feedback.snapshot()] == ["record_started", "saving"]
    control.close()


def test_events_rejected_before_ready_are_acknowledged_not_replayed(cfg):
    control = make_controller(cfg)
    data = packet()
    data["events"] = [{"id": 1, "name": "record_start", "down": True, "age_ms": 0}]
    step(control, data, camera_frames=False)
    assert control.event_ack == 1 and control.recorder.path is None
    assert control.feedback.snapshot()[-1]["kind"] == "rejected"
    step(control, data)
    assert control.ready and control.recorder.path is None


def test_stale_or_out_of_sequence_events_cannot_execute(cfg):
    control = make_controller(cfg)
    step(control, packet())
    data = packet()
    data["events"] = [{"id": 2, "name": "record_start", "down": True, "age_ms": 0}]
    with pytest.raises(ValueError, match="sequence gap"):
        control.accept(data)
    data["events"][0]["id"] = 1
    control.accept(data, time.perf_counter()-1)
    control.accept(dict(packet(), events=[]))  # Fresh pose must not renew an old button event.
    control.tick(time.perf_counter(), .02)
    assert control.mode == "fault" and control.recorder.path is None


def test_saved_confirmation_waits_for_disk_commit(cfg, monkeypatch):
    control = make_controller(cfg)
    blocked, release = Event(), Event()
    original = control.recorder._manifest
    def delayed_commit(status, reason):
        if status == "complete":
            blocked.set()
            release.wait(3)
        original(status, reason)
    monkeypatch.setattr(control.recorder, "_manifest", delayed_commit)
    try:
        step(control, packet())
        step(control, packet(record_start=True))
        sync, _, _, _ = synced_sample(cfg)
        control.recorder.add(sync.sample(10.), 0)
        step(control, packet(record_stop=True))
        time.sleep(.09)
        step(control, packet())
        assert blocked.wait(1)
        assert "saved" not in [n["kind"] for n in control.feedback.snapshot()]
        release.set()
        assert control.recorder.wait(2)
        step(control, packet())
        step(control, packet())
        assert [n["kind"] for n in control.feedback.snapshot()].count("saved") == 1
        meta = json.loads((control.recorder.path/"manifest.json").read_text())
        assert meta["status"] == "complete"
    finally:
        release.set()
        control.close()


def test_invalid_event_payload_rejected():
    data = packet()
    data["events"] = [{"id": 1, "name": "record_start", "down": True, "age_ms": 201}]
    with pytest.raises(ValueError, match="button event"):
        validate_packet(data)
