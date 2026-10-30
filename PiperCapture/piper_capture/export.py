"""Native episodes -> official LeRobotDataset v3, using LeRobot 0.4.2."""
import argparse
import csv
from importlib.metadata import version
import json
from pathlib import Path

import cv2
import numpy as np

from .core import ACTION_SEMANTICS, ARMS, CAMERAS, VECTOR_NAMES


class EpisodeFrames:
    """Sequential MP4 decoding, with compatibility for existing JPEG episodes."""
    def __init__(self, path, meta):
        self.path, self.meta = Path(path).resolve(), meta
        self.videos, self.next_index = {}, 0

    def _path(self, relative):
        path = (self.path / relative).resolve()
        if not path.is_relative_to(self.path):
            raise ValueError("Image/video path outside episode")
        return path

    def __enter__(self):
        try:
            if self.meta["schema"] == "pipercapture.rgb_action.v2":
                for name in CAMERAS:
                    video = cv2.VideoCapture(str(self._path(self.meta["videos"][name])))
                    self.videos[name] = video
                    if not video.isOpened() or int(video.get(cv2.CAP_PROP_FRAME_COUNT)) != self.meta["frames"]:
                        raise ValueError(f"Missing/truncated video: {name}")
                    if abs(video.get(cv2.CAP_PROP_FPS) - self.meta["fps"]) > .01:
                        raise ValueError(f"Video FPS mismatch: {name}")
            return self
        except Exception:
            self.close()
            raise

    def read(self, row):
        if self.videos:
            if row["videos"] != self.meta["videos"] or row["frame_index"] < self.next_index:
                raise ValueError("Invalid video reference/frame order")
            while self.next_index <= row["frame_index"]:
                images = {}
                for name, video in self.videos.items():
                    ok, frame = video.read()
                    if not ok:
                        raise ValueError(f"Could not decode {name} frame {self.next_index}")
                    images[name] = frame
                self.next_index += 1
        else:
            images = {name: cv2.imdecode(np.fromfile(self._path(row["images"][name]), dtype=np.uint8),
                                         cv2.IMREAD_COLOR) for name in CAMERAS}
        for name, image in images.items():
            camera = self.meta["config"]["cameras"][name]
            if image is None or image.shape != (camera["height"], camera["width"], 3):
                raise ValueError(f"Invalid image dimensions: {name}")
        return images

    def close(self):
        for video in self.videos.values():
            video.release()
        self.videos.clear()

    def __exit__(self, *args):
        self.close()


def check_action_csv(path, rows):
    with (path / "action.csv").open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["timestamp_unix_s", *VECTOR_NAMES]:
            raise ValueError("Unexpected action.csv columns")
        for row in rows:
            record = next(reader, None)
            if record is None:
                raise ValueError("action.csv has fewer rows than samples.jsonl")
            values = np.array([float(record[name]) for name in VECTOR_NAMES])
            stamp = float(record["timestamp_unix_s"])
            if (not np.isfinite(stamp) or abs(stamp - row["timestamp_unix_s"]) > 1e-6
                    or not np.array_equal(values, row["action"])):
                raise ValueError("action.csv does not match samples.jsonl")
        if next(reader, None) is not None:
            raise ValueError("action.csv has extra rows")


def read_segments(source, allow_mock=False):
    """Reject unfinished episodes; split gaps instead of hiding missing real time."""
    source = Path(source)
    paths = [source] if (source / "manifest.json").exists() else sorted(
        p.parent for p in source.rglob("manifest.json"))
    result, fingerprint = [], None
    for path in paths:
        manifest_path = path / "manifest.json"
        if not manifest_path.exists():
            continue
        meta = json.loads(manifest_path.read_text(encoding="utf-8"))
        if meta.get("storage_layout") == "task/capture_index":
            meta["task"] = path.resolve().parent.name
        if meta["status"] != "complete":
            print(f"Skip unfinished episode: {path}")
            continue
        if meta["simulated"] and not allow_mock:
            raise ValueError(f"Mock data excluded by default: {path}; use --allow-mock only for validation")
        if (meta["schema"] not in ("pipercapture.rgb_action.v1", "pipercapture.rgb_action.v2")
                or meta["vector_names"] != VECTOR_NAMES
                or meta["action_semantics"] != ACTION_SEMANTICS):
            raise ValueError(f"Unsupported schema: {path}")
        cfg = meta["config"]
        signature = json.dumps({"fps": meta["fps"], "simulated": meta["simulated"],
            "tcp": {s: c["tcp_offset"] for s, c in cfg["arms"].items()},
            "cameras": {n: (c["width"], c["height"], c["serial"]) for n, c in cfg["cameras"].items()}}, sort_keys=True)
        if fingerprint is not None and fingerprint != signature:
            raise ValueError("Do not combine different camera/TCP/fps configurations or mock/real data")
        fingerprint = signature
        rows = [json.loads(line) for line in (path / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
        if not rows or len(rows) != meta["frames"]:
            raise ValueError(f"Manifest/frame count mismatch: {path}")
        modern = meta["schema"] == "pipercapture.rgb_action.v2"
        stamp_key = "timestamp_unix_s" if modern else "timestamp_monotonic_s"
        action_key = "action_timestamp_unix_s" if modern else "action_timestamp_monotonic_s"
        source_key = "source_timestamps_unix_s" if modern else "source_timestamps"
        if modern:
            check_action_csv(path, rows)
        with EpisodeFrames(path, meta) as frames:
            for index, row in enumerate(rows):
                if row["frame_index"] != index:
                    raise ValueError("Frame indexes must match row order")
                frames.read(row)
        segments, current = [], []
        for row in rows:
            if not np.isfinite([row[stamp_key], row[action_key]]).all():
                raise ValueError("Invalid timestamp")
            if abs(row[action_key] - row[stamp_key] - 1 / meta["fps"]) > 1e-6:
                raise ValueError("Action must refer to the next grid tick")
            if any(not np.isfinite(stamp) or stamp <= row[stamp_key] for s in ARMS
                   for field, stamp in row[source_key]["actions"][s].items()
                   if field in ("tcp", "gripper") or field.startswith("end_pose_")):
                raise ValueError("Action is not a future measurement")
            if current and row["grid_index"] != current[-1]["grid_index"] + 1:
                segments.append(current)
                current = []
            for key in ("action", "observation.state"):
                value = np.asarray(row[key])
                if value.shape != (16,) or not np.isfinite(value).all():
                    raise ValueError(f"Invalid {key} in {path}")
            if current and abs((row[stamp_key] - current[-1][stamp_key])
                               - 1 / meta["fps"]) > .002:
                raise ValueError("Timestamp/grid mismatch")
            current.append(row)
        if current:
            segments.append(current)
        result.extend((path, meta, segment) for segment in segments if len(segment) >= 2)
    if not result:
        raise ValueError("No complete episodes with at least two contiguous synchronized frames")
    return result


def convert(source, target, repo_id, allow_mock=False, image_mode=False, validate_only=False):
    target = Path(target).resolve()
    segments = read_segments(source, allow_mock)
    print(f"Validated {len(segments)} segments, {sum(len(rows) for _, _, rows in segments)} frames")
    if validate_only:
        return
    if Path(target).exists():
        raise FileExistsError(f"Output already exists: {target}")
    if version("lerobot") != "0.4.2":
        raise RuntimeError("Use the export environment with lerobot==0.4.2 for this adapter")
    import av  # Initialize bundled FFmpeg before Torch on Windows.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    meta = segments[0][1]
    features = {key: {"dtype": "float32", "shape": (16,), "names": VECTOR_NAMES}
                for key in ("observation.state", "action")}
    for name in CAMERAS:
        cfg = meta["config"]["cameras"][name]
        features[f"observation.images.{name}"] = {
            "dtype": "image" if image_mode else "video",
            "shape": (cfg["height"], cfg["width"], 3), "names": ["height", "width", "channels"]}
    dataset = LeRobotDataset.create(repo_id=repo_id, root=Path(target), fps=meta["fps"],
        robot_type="piperx_dual", features=features, use_videos=not image_mode,
        image_writer_threads=4, image_writer_processes=0)
    frames, current_path = None, None
    try:
        for path, manifest, rows in segments:
            if path != current_path:
                if frames is not None:
                    frames.close()
                frames = EpisodeFrames(path, manifest).__enter__()
                current_path = path
            for row in rows:
                frame = {key: np.asarray(row[key], dtype=np.float32) for key in ("observation.state", "action")}
                images = frames.read(row)
                for name in CAMERAS:
                    bgr = images[name]
                    frame[f"observation.images.{name}"] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                frame["task"] = manifest["task"]
                dataset.add_frame(frame)
            dataset.save_episode()
    finally:
        if frames is not None:
            frames.close()
        dataset.finalize()
    # Preserve source timing/configuration for audit; LeRobot time is a fresh uniform grid per segment.
    provenance = [{"episode_index": index, "source": str(path.resolve()), "first_grid": rows[0]["grid_index"],
                   "last_grid": rows[-1]["grid_index"], "manifest": meta}
                  for index, (path, meta, rows) in enumerate(segments)]
    (Path(target) / "meta" / "pipercapture_sources.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    print(f"Saved local LeRobot dataset: {target}; no upload performed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="outputs")
    parser.add_argument("--output", default="datasets/pipercapture")
    parser.add_argument("--repo-id", default="local/pipercapture")
    parser.add_argument("--allow-mock", action="store_true")
    parser.add_argument("--image-mode", action="store_true", help="Use embedded images instead of MP4")
    parser.add_argument("--validate-only", action="store_true", help="Check native data without installing LeRobot")
    args = parser.parse_args()
    convert(args.source, args.output, args.repo_id, args.allow_mock, args.image_mode, args.validate_only)


if __name__ == "__main__":
    main()
