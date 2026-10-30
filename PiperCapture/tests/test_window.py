import json
from pathlib import Path
import time
from types import SimpleNamespace

from piper_capture.app import CaptureWindow
from piper_capture.capture import Capture
from test_capture import cfg, wait_mode


def dispose(window):
    if not window.closed:
        window.root.after_cancel(window.timer)
        window.root.destroy()


def test_native_buttons_record_save_set_home_return_cancel(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        window = CaptureWindow(cap)
        window.root.withdraw()
        try:
            assert cap.status()["mode"] == "idle", cap.status()
            assert len(window.buttons) == 6
            window.buttons["g"].invoke()
            assert "请先" in window.result_text.get()
            window.buttons["h"].invoke()
            assert cap.home_path.exists()
            window.task_name.set("把杯子放入托盘")
            window.capture_index.set("12")
            for key in ("R", "S", "G", "H", "C", "Q"):
                window.on_key(SimpleNamespace(keysym=key, widget=window.task_entry))
            assert cap.status()["mode"] == "idle"
            assert not window.exit_requested
            window.buttons["r"].invoke()
            assert cap.status()["recording"]
            assert str(window.task_entry.cget("state")) == "disabled"
            assert str(window.capture_index_entry.cget("state")) == "disabled"
            window.buttons["h"].invoke()
            assert "未执行" in window.result_text.get()
            time.sleep(.35)
            window.buttons["s"].invoke()
            wait_mode(cap, "idle")
            assert cap.status()["frames"] > 0
            meta = json.loads((Path(cap.status()["episode"])/"manifest.json").read_text(encoding="utf-8"))
            assert Path(cap.status()["episode"]).parts[-2:] == ("把杯子放入托盘", "episode_012")
            assert "task" not in meta
            window.buttons["g"].invoke()
            assert cap.status()["mode"] == "returning"
            window.buttons["c"].invoke()
            assert cap.status()["mode"] == "idle"
            window.buttons["s"].invoke()
            assert "当前没有录制" in window.result_text.get()
        finally:
            dispose(window)


def test_uppercase_key_and_exit_button_save_before_closing(cfg, tmp_path):
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        window = CaptureWindow(cap)
        window.root.withdraw()
        try:
            window.on_key(SimpleNamespace(keysym="R"))
            assert cap.status()["recording"]
            time.sleep(.35)
            episode = Path(cap.status()["episode"])
            window.buttons["q"].invoke()
            assert window.exit_requested and not window.closed
            deadline = time.perf_counter() + 3
            while not window.closed and time.perf_counter() < deadline:
                window.root.update()
                time.sleep(.01)
            assert window.closed
            assert json.loads((episode/"manifest.json").read_text())["status"] == "complete"
            assert not any(robot.commands for robot in cap.robots.values())
        finally:
            dispose(window)


def test_shared_can_return_button_and_shortcut_cannot_send_motion(cfg, tmp_path):
    cfg["topology"] = "leader_follower_shared_can"
    with Capture(cfg, mock=True, home_path=tmp_path/"home.json") as cap:
        wait_mode(cap, "idle")
        cap.mock = False
        window = CaptureWindow(cap)
        window.root.withdraw()
        try:
            assert "disabled" in window.buttons["g"].state()
            window.buttons["g"].invoke()
            window.on_key(SimpleNamespace(keysym="G"))
            assert "未发送运动命令" in window.result_text.get()
            assert not any(robot.commands for robot in cap.robots.values())
        finally:
            dispose(window)
