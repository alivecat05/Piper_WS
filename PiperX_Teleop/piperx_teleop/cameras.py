"""One worker per RealSense; D555 DDS and D405 USB share the same context."""
import json
import threading
import time

import cv2
import numpy as np

from .core import CAMERAS, History


def rs_context(settings):
    import pyrealsense2 as rs
    return rs.context(json.dumps(settings))


def discover(settings):
    import pyrealsense2 as rs
    context = rs_context(settings)
    time.sleep(4)  # Allow network/DDS discovery.
    result = []
    for device in context.query_devices():
        profiles = []
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == rs.stream.color:
                    video = profile.as_video_stream_profile()
                    profiles.append(f"{video.width()}x{video.height()} {profile.fps()}fps {profile.format()}")
        result.append({"name": device.get_info(rs.camera_info.name),
                       "serial": device.get_info(rs.camera_info.serial_number),
                       "color_profiles": sorted(set(profiles))})
    return result


class Cameras:
    def __init__(self, cfg, mock=False):
        self.cfg, self.mock = cfg, mock
        self.histories = {name: History(90) for name in CAMERAS}
        self.errors = {}
        self.stop_event = threading.Event()
        self.threads = []
        self.context = None

    def start(self):
        if not self.mock:
            serials = [c["serial"] for c in self.cfg["cameras"].values()]
            if not all(serials) or len(set(serials)) != 3:
                raise ValueError("Assign three distinct RealSense serial numbers in config.json")
            self.context = rs_context(self.cfg["realsense"])
            time.sleep(4)
        for name in CAMERAS:
            thread = threading.Thread(target=self._worker, args=(name,), daemon=True, name=name)
            self.threads.append(thread)
            thread.start()

    def _worker(self, name):
        pipe = None
        try:
            cfg = self.cfg["cameras"][name]
            if self.mock:
                index = 0
                while not self.stop_event.is_set():
                    t = time.perf_counter()
                    image = np.zeros((cfg["height"], cfg["width"], 3), np.uint8)
                    cv2.putText(image, f"MOCK {name} {index}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                                .8, (40, 210, 110), 2)
                    self.histories[name].add(t, {"bgr": image, "frame_number": index,
                                                "device_ms": t * 1000, "clock": "mock",
                                                "timestamp_basis": "host_monotonic", "receive_s": t})
                    index += 1
                    time.sleep(max(0, 1 / cfg["fps"] - (time.perf_counter() - t)))
                return
            import pyrealsense2 as rs
            pipe = rs.pipeline(self.context)
            rs_cfg = rs.config()
            rs_cfg.enable_device(cfg["serial"])
            rs_cfg.enable_stream(rs.stream.color, cfg["width"], cfg["height"], rs.format.bgr8, cfg["fps"])
            profile = pipe.start(rs_cfg)  # Fail clearly if the selected profile isn't supported.
            actual = profile.get_device().get_info(rs.camera_info.name)
            if cfg["model"] not in actual:
                raise ValueError(f"{name}: expected {cfg['model']}, found {actual}")
            for sensor in profile.get_device().query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
            for _ in range(30):
                pipe.wait_for_frames(3000)
            wall_offset = time.perf_counter() - time.time()
            while not self.stop_event.is_set():
                frame = pipe.wait_for_frames(2000).get_color_frame()
                receive = time.perf_counter()
                domain = frame.get_frame_timestamp_domain()
                device_ms = frame.get_timestamp()
                basis = "global_time" if domain == rs.timestamp_domain.global_time else "host_receive"
                # Never compare the hardware clocks of different cameras directly.
                stamp = device_ms / 1000 + wall_offset if basis == "global_time" else receive
                stamp -= cfg["latency_ms"] / 1000
                if abs(stamp - receive) > 1:
                    raise RuntimeError(f"{name}: timestamp mapping changed by >1 second")
                self.histories[name].add(stamp, {"bgr": np.asanyarray(frame.get_data()).copy(),
                    "frame_number": frame.get_frame_number(), "device_ms": device_ms,
                    "clock": str(domain), "timestamp_basis": basis, "receive_s": receive})
        except Exception as error:
            self.errors[name] = f"{type(error).__name__}: {error}"
        finally:
            if pipe is not None:
                try:
                    pipe.stop()
                except RuntimeError:
                    pass

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=4)

