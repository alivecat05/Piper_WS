import json
from pathlib import Path
import time

import numpy as np
import pytest

from piper_capture.capture import Capture, load_config
from piper_capture.core import ARMS, CAMERAS, History, Synchronizer
from piper_capture.export import read_segments
from piper_capture.robot import MockPiperX


@pytest.fixture
def cfg(tmp_path):
    cfg = load_config(Path(__file__).resolve().parents[1] / "config.example.json")
    cfg["output"] = str(tmp_path / "episodes")
    for camera in cfg["cameras"].values():
        camera.update(width=64, height=48)
    return cfg


def histories():
    return {n: History(500) for n in CAMERAS}, {s: History(500) for s in ARMS}


def add_frames(frames, t, number):
    for n in CAMERAS:
        frames[n].add(t, {"frame_number": number, "bgr": np.zeros((4, 4, 3))})


def add_state(states, t, x=0, *, gripper=None, pose_stamp=None, qw=1):
    for side in ARMS:
        stamps = {"tcp": t, "gripper": t if gripper is None else gripper}
        if pose_stamp is not None:
            stamps["end_pose_xy"] = pose_stamp
        states[side].add(t, {"vector": [x, 0, .3, 0, 0, 0, qw, .04],
                             "joints": [0]*6, "source_timestamps": stamps})


def test_action_uses_next_robot_measurement_without_next_rgb(cfg):
    frames, states = histories()
    add_frames(frames, 1, 1)
    add_state(states, 1, .1)
    add_state(states, 1+1/30, .2)
    row = Synchronizer(frames, states, cfg["sync"]).sample(1)
    assert row["observation.state"][0] == .1
    assert row["action"][0] == .2
    assert row["action_timestamp_monotonic_s"] == 1+1/30


def test_no_action_without_fresh_next_robot_feedback(cfg):
    frames, states = histories()
    add_frames(frames, 1, 1)
    add_state(states, 1)
    add_state(states, 1+1/30, gripper=1)
    sync = Synchronizer(frames, states, cfg["sync"])
    assert sync.sample(1) is None
    assert sync.rejection == "action_missing_or_stale"
    assert not sync.last_frames  # Failed actions must not consume an RGB frame.


def test_one_stale_pose_fragment_is_not_a_future_action(cfg):
    frames, states = histories()
    add_frames(frames, 1, 1)
    add_state(states, 1)
    add_state(states, 1+1/30, pose_stamp=.99)
    assert Synchronizer(frames, states, cfg["sync"]).sample(1) is None


def test_equivalent_quaternion_signs_are_continuous(cfg):
    frames, states = histories()
    add_frames(frames, 1, 1)
    add_state(states, 1)
    add_state(states, 1+1/30, qw=-1)
    assert Synchronizer(frames, states, cfg["sync"]).sample(1)["action"][6] == 1


def test_synchronizer_rejects_stale_fields_span_and_duplicate(cfg):
    frames, states = histories()
    add_frames(frames, 1, 1)
    add_state(states, 1, gripper=.8)
    sync = Synchronizer(frames, states, cfg["sync"])
    assert sync.sample(1) is None
    assert sync.rejection == "state_missing_or_stale"
    add_state(states, 1.001)
    add_state(states, 1+1/30)
    assert sync.sample(1.001) is not None
    assert sync.sample(1.002) is None
    assert sync.rejection.startswith("camera_reused:")
    for n in CAMERAS:
        frames[n].add(2.017 if n == "head" else 1.983,
                      {"frame_number": 2, "bgr": np.zeros((4, 4, 3))})
    add_state(states, 2)
    add_state(states, 2+1/30)
    assert sync.sample(2) is None
    assert sync.rejection == "camera_span"


def test_joint_matching_avoids_long_gap_from_camera_phase(cfg):
    # Independent nearest selection picks +16.5 ms and -12.5 ms: every tick
    # fails the 25 ms span. Previous head frames are valid at -16.83 ms.
    frames, states = histories()
    for i in range(202):
        for n in CAMERAS:
            t = 1+i/30 + (.0165 if n == "head" else -.0125)
            frames[n].add(t, {"frame_number": i, "bgr": None})
        add_state(states, 1+i/30)
    sync = Synchronizer(frames, states, cfg["sync"])
    ids = {n: [] for n in CAMERAS}
    for i in range(1, 201):
        row = sync.sample(1+i/30)
        assert row is not None, sync.rejection
        stamps = list(row["source_timestamps"]["cameras"].values())
        assert max(stamps)-min(stamps) <= .025
        assert all(abs(t-row["timestamp_monotonic_s"]) <= .020 for t in stamps)
        for n in CAMERAS:
            ids[n].append(row["camera_metadata"][n]["frame_number"])
    assert all(len(set(v)) == 200 for v in ids.values())


def wait_mode(cap, mode, timeout=4):
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        status = cap.status()
        if status["mode"] == mode:
            return
        if status["mode"] == "fault":
            pytest.fail(status["message"])
        time.sleep(.01)
    pytest.fail(f"Did not reach {mode}: {cap.status()}")


def test_recording_is_read_only_and_native_data_is_exportable(cfg, tmp_path):
    robots = {s: MockPiperX(cfg["arms"][s]) for s in ARMS}
    with Capture(cfg, mock=True, robots=robots, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.set_home()
        assert not any(r.commands for r in robots.values())
        cap.record(task="  把杯子放入托盘  ", capture_index=12)
        with pytest.raises(RuntimeError):
            cap.go_home()
        time.sleep(.65)
        cap.stop()
        cap.stop()  # Idempotent, never starts a new episode.
        wait_mode(cap, "idle")
        assert cap.status()["frames"] > 4
        episode = cap.status()["episode"]
        meta = json.loads((Path(episode)/"manifest.json").read_text(encoding="utf-8"))
        assert Path(episode).parts[-2:] == ("把杯子放入托盘", "episode_012")
        assert "task" not in meta and "capture_index" not in meta
        rows = [json.loads(line) for line in (Path(episode)/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
        assert all("task" not in r and "capture_index" not in r and "task_id" not in r for r in rows)
        with pytest.raises(ValueError, match="已存在"):
            cap.record(task="把杯子放入托盘", capture_index=12)
        assert cap.recorder.next_index("把杯子放入托盘") == 13
        expected = int((meta["end_unix_s"]-meta["start_unix_s"])*cfg["record_hz"]) + 1
        assert meta["frames"] + meta["dropped_grid_ticks"] == expected
        assert sum(meta["drop_reasons"].values()) == meta["dropped_grid_ticks"]
        assert meta["drop_reasons"]["episode_end"] == 1
        segments = read_segments(episode, allow_mock=True)
        assert segments
        for _, meta, rows in segments:
            assert meta["status"] == "complete"
            assert rows[-1]["action_timestamp_unix_s"] <= meta["end_unix_s"]
        with pytest.raises(ValueError, match="Mock"):
            read_segments(episode)
    assert not any(r.commands for r in robots.values())


@pytest.mark.parametrize("task,capture_index", [(" ", 1), ("任务", -1), ("任务", 1.5), ("任务", True), ("../逃逸", 1), ("CON", 1), ("任务", 0)])
def test_invalid_task_never_starts_recording(cfg, tmp_path, task, capture_index):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        with pytest.raises(ValueError):
            cap.record(task=task, capture_index=capture_index)
        assert cap.status()["mode"] == "idle"
        assert cap.recorder.path is None


def test_home_moves_only_on_explicit_request_and_can_cancel(cfg, tmp_path):
    robots = {s: MockPiperX(cfg["arms"][s]) for s in ARMS}
    with Capture(cfg, mock=True, robots=robots, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.set_home()
        robots["left"].joints[1] = .9
        cap.go_home()
        wait_mode(cap, "idle")
        assert robots["left"].joints[1] == .6
        assert robots["left"].commands[:3] == ["hold", "enable", "home"]
        cap.go_home()
        time.sleep(.1)
        assert cap.cancel_home()["mode"] == "idle"
        assert "stop" not in robots["left"].commands


def test_close_without_save_leaves_incomplete_episode(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.record()
        time.sleep(.3)
        episode = Path(cap.status()["episode"])
    assert json.loads((episode/"manifest.json").read_text())["status"] == "incomplete"


def test_simulated_home_cannot_drive_real_robot(cfg, tmp_path):
    cfg["topology"] = "independent"
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.set_home()
        cap.mock = False
        with pytest.raises(ValueError, match="different"):
            cap.go_home()
        assert not any(r.commands for r in cap.robots.values())


def test_fault_during_return_stops_commanded_arm(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.set_home()
        cap.go_home()
        time.sleep(.06)
        cap.cameras.errors["head"] = "disconnected"
        deadline = time.perf_counter() + 1
        while cap.status()["mode"] != "fault" and time.perf_counter() < deadline:
            time.sleep(.01)
        assert cap.status()["mode"] == "fault"
        assert "stop" in cap.robots["left"].commands


def test_fault_in_passive_capture_does_not_issue_motor_commands(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.record()
        time.sleep(.2)
        cap.cameras.errors["head"] = "disconnected"
        deadline = time.perf_counter() + 1
        while cap.status()["mode"] != "fault" and time.perf_counter() < deadline:
            time.sleep(.01)
        assert cap.status()["mode"] == "fault"
        assert not any(r.commands for r in cap.robots.values())
        episode = Path(cap.status()["episode"])
    assert json.loads((episode/"manifest.json").read_text())["status"] == "incomplete"


def test_invalid_home_is_rejected_before_any_command(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.set_home()
        saved = json.loads(cap.home_path.read_text())
        saved["arms"]["right"]["gripper_m"] = 10
        cap.home_path.write_text(json.dumps(saved))
        with pytest.raises(ValueError):
            cap.go_home()
        assert not any(r.commands for r in cap.robots.values())


def test_feedback_timeout_does_not_record_cached_pose(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cached = cap.robots["right"].read()
        cap.robots["right"].read = lambda: cached
        cap.record()
        time.sleep(.35)
        assert cap.status()["mode"] == "fault"
        assert "Robot feedback" in cap.status()["message"]
        assert not any(r.commands for r in cap.robots.values())


def test_shared_bus_never_sends_independent_home_commands(cfg, tmp_path):
    cfg["topology"] = "leader_follower_shared_can"
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.mock = False  # Injected mock devices exercise the real-mode guard without hardware.
        cap.set_home()
        assert json.loads(cap.home_path.read_text())["scope"] == "followers_only"
        assert not cap.status()["home_available"]
        with pytest.raises(RuntimeError, match="主从共用 CAN"):
            cap.go_home()
        assert cap.status()["mode"] == "idle"
        assert not any(r.commands for r in cap.robots.values())
