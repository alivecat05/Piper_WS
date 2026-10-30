import csv
import json
import time

import cv2
import numpy as np
import pytest

from piper_capture.core import ARMS, CAMERAS, VECTOR_NAMES
from piper_capture.export import EpisodeFrames, read_segments
from piper_capture.recording import Recorder
from test_capture import cfg


def sample(cfg, stamp, number):
    state = [.3, 0., .3, 0., 0., 0., 1., .04] * 2
    action = list(state)
    action[0] += .001 * number
    future = stamp + 1 / cfg["record_hz"]
    image = np.full((48, 64, 3), (20 + number*40, 70, 210), np.uint8)
    return {"timestamp_monotonic_s": stamp, "action_timestamp_monotonic_s": future,
            "observation.state": state, "action": action, "joints": {s: [0.]*6 for s in ARMS},
            "source_timestamps": {"cameras": dict.fromkeys(CAMERAS, stamp),
                                  "states": {s: {"tcp": stamp, "gripper": stamp} for s in ARMS},
                                  "actions": {s: {"tcp": future, "gripper": future} for s in ARMS}},
            "camera_metadata": {n: {"device_ms": 100, "receive_s": stamp, "frame_number": number}
                                for n in CAMERAS},
            "images": {n: image.copy() for n in CAMERAS}}


def record_episode(cfg, grids=(0, 1, 4, 5)):
    start = time.perf_counter()
    recorder = Recorder(cfg["output"], cfg, mock=True)
    recorder.start(start)
    for number, grid in enumerate(grids):
        recorder.add(sample(cfg, start+grid/cfg["record_hz"], number), grid)
    recorder.stop(end_time=start+(grids[-1]+2)/cfg["record_hz"])
    assert recorder.wait(5)
    assert recorder.result == "complete", recorder.error
    path = recorder.path
    rows = [json.loads(line) for line in (path/"samples.jsonl").read_text().splitlines()]
    meta = json.loads((path/"manifest.json").read_text())
    return path, rows, meta


def test_mp4_csv_jsonl_have_identical_frame_order_and_unix_times(cfg):
    before = time.time()
    path, rows, meta = record_episode(cfg)
    assert not list(path.rglob("*.jpg"))
    assert not (path/"images").exists()
    assert meta["schema"] == "pipercapture.rgb_action.v2"
    assert before-.1 < meta["start_unix_s"] < before+1
    with (path/"action.csv").open(newline="") as stream:
        csv_rows = list(csv.reader(stream))
    assert csv_rows[0] == ["timestamp_unix_s", *VECTOR_NAMES]
    assert len(csv_rows) == len(rows)+1 == 5
    assert all(len(row) == 17 for row in csv_rows)
    with EpisodeFrames(path, meta) as frames:
        for index, (row, csv_row) in enumerate(zip(rows, csv_rows[1:])):
            assert row["frame_index"] == index
            assert "timestamp_monotonic_s" not in row
            assert "action_timestamp_monotonic_s" not in row
            assert "source_timestamps" not in row
            assert abs(float(csv_row[0]) - row["timestamp_unix_s"]) < 1e-6
            assert [float(v) for v in csv_row[1:]] == row["action"]
            assert row["source_timestamps_unix_s"]["cameras"]["head"] > 1_000_000_000
            assert "device_ms" not in row["camera_metadata"]["head"]
            images = frames.read(row)
            for image in images.values():
                assert image.shape == (48, 64, 3)
                assert np.max(np.abs(image.mean(axis=(0, 1)) - (20+index*40, 70, 210))) < 10
    segments = read_segments(path, allow_mock=True)
    assert [[r["grid_index"] for r in segment] for _, _, segment in segments] == [[0, 1], [4, 5]]


def test_mismatched_csv_is_rejected(cfg):
    path, _, _ = record_episode(cfg, (0, 1))
    with (path/"action.csv").open("a") as stream:
        stream.write("0," + ",".join(["0"]*16) + "\n")
    with pytest.raises(ValueError, match="extra rows"):
        read_segments(path, allow_mock=True)


def test_task_folders_increment_and_export_recursively(cfg):
    cfg["task"] = "把杯子放入托盘"
    first, _, _ = record_episode(cfg, (0, 1))
    second, _, _ = record_episode(cfg, (0, 1))
    assert first.parts[-2:] == (cfg["task"], "episode_001")
    assert second.parts[-2:] == (cfg["task"], "episode_002")
    segments = read_segments(cfg["output"], allow_mock=True)
    assert len(segments) == 2
    assert all(meta["task"] == cfg["task"] for _, meta, _ in segments)


def test_truncated_mp4_is_rejected(cfg):
    path, _, _ = record_episode(cfg, (0, 1))
    (path/"head.mp4").write_bytes(b"broken video")
    with pytest.raises(ValueError, match="Missing/truncated"):
        read_segments(path, allow_mock=True)


def test_encoder_failure_never_marks_episode_complete(cfg, monkeypatch):
    class FailedWriter:
        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setattr(cv2, "VideoWriter", lambda *args: FailedWriter())
    recorder = Recorder(cfg["output"], cfg, mock=True)
    recorder.start(time.perf_counter())
    assert recorder.wait(5)
    assert recorder.result == "incomplete"
    assert "Could not open" in recorder.error
    assert json.loads((recorder.path/"manifest.json").read_text())["status"] == "incomplete"


def test_existing_jpeg_episodes_remain_readable(cfg):
    path, rows, meta = record_episode(cfg, (0, 1))
    with EpisodeFrames(path, meta) as frames:
        for row in rows:
            row["images"] = {}
            for name, image in frames.read(row).items():
                relative = f"images/{name}/{row['frame_index']:08d}.jpg"
                (path/relative).parent.mkdir(parents=True, exist_ok=True)
                ok, encoded = cv2.imencode(".jpg", image)
                assert ok
                encoded.tofile(path/relative)
                row["images"][name] = relative
            row["timestamp_monotonic_s"] = row.pop("timestamp_unix_s")
            row["action_timestamp_monotonic_s"] = row.pop("action_timestamp_unix_s")
            row["source_timestamps"] = row.pop("source_timestamps_unix_s")
            row.pop("videos")
    meta["schema"] = "pipercapture.rgb_action.v1"
    (path/"manifest.json").write_text(json.dumps(meta))
    (path/"samples.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
    assert len(read_segments(path, allow_mock=True)) == 1
