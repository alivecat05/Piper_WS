"""Native episodes -> official LeRobotDataset v3, using LeRobot 0.4.2."""
import argparse
from importlib.metadata import version
import json
from pathlib import Path

import cv2
import numpy as np

from .core import CAMERAS, VECTOR_NAMES


def read_segments(source, allow_mock=False):
    """Reject unfinished episodes; split gaps instead of hiding missing real time."""
    source = Path(source)
    paths = [source] if (source / "manifest.json").exists() else sorted(source.glob("episode_*"))
    result, fingerprint = [], None
    for path in paths:
        manifest_path = path / "manifest.json"
        if not manifest_path.exists():
            continue
        meta = json.loads(manifest_path.read_text(encoding="utf-8"))
        if meta["status"] != "complete":
            print(f"Skip unfinished episode: {path}")
            continue
        if meta["simulated"] and not allow_mock:
            raise ValueError(f"Mock data excluded by default: {path}; use --allow-mock only for validation")
        if meta["schema"] != "piperx.rgb_action.v1" or meta["vector_names"] != VECTOR_NAMES:
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
        segments, current = [], []
        for row in rows:
            if current and row["grid_index"] != current[-1]["grid_index"] + 1:
                segments.append(current)
                current = []
            for key in ("action", "observation.state"):
                value = np.asarray(row[key])
                if value.shape != (16,) or not np.isfinite(value).all():
                    raise ValueError(f"Invalid {key} in {path}")
            for name in CAMERAS:
                relative = Path(row["images"][name])
                image_path = (path / relative).resolve()
                if not image_path.is_relative_to(path.resolve()):
                    raise ValueError("Image path outside episode")
                image = cv2.imread(str(image_path))
                c = cfg["cameras"][name]
                if image is None or image.shape != (c["height"], c["width"], 3):
                    raise ValueError(f"Missing or invalid image: {image_path}")
            if current and abs((row["timestamp_monotonic_s"] - current[-1]["timestamp_monotonic_s"])
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
    try:
        for path, manifest, rows in segments:
            for row in rows:
                frame = {key: np.asarray(row[key], dtype=np.float32) for key in ("observation.state", "action")}
                for name in CAMERAS:
                    bgr = cv2.imread(str(path / row["images"][name]))
                    frame[f"observation.images.{name}"] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                frame["task"] = manifest["task"]
                dataset.add_frame(frame)
            dataset.save_episode()
    finally:
        dataset.finalize()
    # Preserve source timing/configuration for audit; LeRobot time is a fresh uniform grid per segment.
    provenance = [{"source": str(path.resolve()), "first_grid": rows[0]["grid_index"],
                   "last_grid": rows[-1]["grid_index"], "manifest": meta} for path, meta, rows in segments]
    (Path(target) / "meta" / "piperx_sources.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    print(f"Saved local LeRobot dataset: {target}; no upload performed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="outputs/episodes")
    parser.add_argument("--output", default="datasets/piperx")
    parser.add_argument("--repo-id", default="local/piperx")
    parser.add_argument("--allow-mock", action="store_true")
    parser.add_argument("--image-mode", action="store_true", help="Use embedded images instead of MP4")
    parser.add_argument("--validate-only", action="store_true", help="Check native data without installing LeRobot")
    args = parser.parse_args()
    convert(args.source, args.output, args.repo_id, args.allow_mock, args.image_mode, args.validate_only)


if __name__ == "__main__":
    main()
