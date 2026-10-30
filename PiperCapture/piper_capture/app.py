"""Local three-view RGB window with native Chinese operation buttons."""
import argparse
import json
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np

from .cameras import discover
from .capture import Capture, load_config
from .core import CAMERAS
from .robot import discover_can

TITLE = "PiperCapture | 三视角采集"
OPERATIONS = (
    ("r", "开始录制", "record"),
    ("s", "结束保存", "stop"),
    ("h", "设定初始位", "set_home"),
    ("g", "返回初始位", "go_home"),
    ("c", "取消返回", "cancel_home"),
)
MODES = {"warming": "设备准备中", "idle": "就绪", "recording": "录制中",
         "saving": "保存中", "returning": "返回初始位中", "fault": "故障", "closed": "已关闭"}
DROP_REASONS = {"camera_missing": "相机缺帧", "camera_reused": "相机重复帧",
                "camera_span": "三视角时间差超限", "state_missing_or_stale": "当前状态缺失",
                "action_missing_or_stale": "下一时刻action缺失", "episode_end": "正常尾帧舍弃"}


def render_views(capture, width, height):
    """Render BGR tiles; device acquisition runs independently of GUI refresh."""
    state = capture.status()
    tile_width = max(240, width // 3)
    height = max(160, height)
    tiles = []
    for name in CAMERAS:
        tile = np.zeros((height, tile_width, 3), np.uint8)
        sample = capture.cameras.histories[name].latest()
        if sample:
            image = sample.value["bgr"]
            scale = min(tile_width / image.shape[1], (height-40) / image.shape[0])
            image = cv2.resize(image, None, fx=scale, fy=scale)
            y, x = 40 + (height-40-image.shape[0]) // 2, (tile_width-image.shape[1]) // 2
            tile[y:y+image.shape[0], x:x+image.shape[1]] = image
        cv2.putText(tile, f"{name}  {state['fps']['cameras'][name]:.1f} FPS", (12, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 2)
        tiles.append(tile)
    return np.hstack(tiles)


class CaptureWindow:
    def __init__(self, capture):
        self.capture = capture
        self.root = tk.Tk()
        self.root.title(TITLE)
        self.root.geometry("1440x540")
        self.root.minsize(960, 420)
        self.closed = self.exit_requested = False
        self.timer = None
        self.photo = None
        self.status_text, self.device_text, self.result_text = (tk.StringVar(master=self.root) for _ in range(3))
        panel = ttk.Frame(self.root, padding=10)
        panel.pack(side="bottom", fill="x")
        ttk.Label(panel, textvariable=self.status_text, font=("Microsoft YaHei UI", 12)).pack(anchor="w")
        ttk.Label(panel, textvariable=self.device_text).pack(anchor="w", pady=4)
        ttk.Label(panel, textvariable=self.result_text, foreground="#005a9e").pack(anchor="w", pady=4)
        task_bar = ttk.Frame(panel)
        task_bar.pack(fill="x", pady=4)
        self.task_name = tk.StringVar(master=self.root, value=capture.cfg["task"])
        self.capture_index = tk.StringVar(master=self.root, value=str(capture.recorder.next_index(capture.cfg["task"])))
        ttk.Label(task_bar, text="任务名").pack(side="left")
        self.task_entry = ttk.Entry(task_bar, textvariable=self.task_name, width=40)
        self.task_entry.pack(side="left", fill="x", expand=True, padx=(6, 16))
        ttk.Label(task_bar, text="采集序号").pack(side="left")
        self.capture_index_entry = ttk.Entry(task_bar, textvariable=self.capture_index, width=10)
        self.capture_index_entry.pack(side="left", padx=6)
        self.was_recording = False
        self.task_name.trace_add("write", self.suggest_index)
        bar = ttk.Frame(panel)
        bar.pack(fill="x", pady=5)
        self.buttons = {}
        for key, label, _ in OPERATIONS:
            if capture.shared_can and not capture.mock:
                label = {"h": "保存从臂参考位", "g": "主从自定义回位未接入"}.get(key, label)
            button = ttk.Button(bar, text=label, command=lambda k=key: self.perform(k))
            button.pack(side="left", expand=True, fill="x", padx=4, ipady=8)
            self.buttons[key] = button
        if not capture.home_available:
            self.buttons["g"].state(["disabled"])
            self.result_text.set(capture.status()["home_unavailable_reason"])
        self.buttons["q"] = ttk.Button(bar, text="退出", command=self.request_close)
        self.buttons["q"].pack(side="left", expand=True, fill="x", padx=4, ipady=8)
        ttk.Label(panel, text="返回初始位前请松手并清空运动路径；R/S/H/G/C/Q 快捷键仍可在此窗口使用。"
                  ).pack(anchor="w")
        self.views = tk.Label(self.root, background="black", borderwidth=0)
        self.views.pack(side="top", expand=True, fill="both")
        self.root.bind("<KeyPress>", self.on_key)
        self.root.protocol("WM_DELETE_WINDOW", self.request_close)
        self.refresh()

    def suggest_index(self, *args):
        try:
            self.capture_index.set(str(self.capture.recorder.next_index(self.task_name.get().strip())))
        except ValueError:
            pass  # Show validation errors when starting, not during text entry.

    def on_key(self, event):
        if isinstance(getattr(event, "widget", None), (tk.Entry, ttk.Entry)):
            return  # Typing task names must never trigger robot/recording shortcuts.
        key = event.keysym.lower()
        if key in self.buttons:
            self.perform(key)
        elif key == "escape":
            self.request_close()

    def perform(self, key):
        if self.exit_requested:
            return
        if key == "q":
            self.request_close()
            return
        for shortcut, label, method in OPERATIONS:
            if key != shortcut:
                continue
            try:
                state = self.capture.status()
                if key == "s" and not state["recording"]:
                    raise RuntimeError("正在保存，请稍候" if state["saving"] else "当前没有录制")
                if key == "c" and state["mode"] != "returning":
                    raise RuntimeError("当前没有执行返回初始位")
                if key == "r":
                    try:
                        capture_index = int(self.capture_index.get())
                    except ValueError:
                        raise ValueError("采集序号必须是正整数") from None
                    result = self.capture.record(task=self.task_name.get(), capture_index=capture_index)
                    self.was_recording = True
                    self.task_entry.configure(state="disabled")
                    self.capture_index_entry.configure(state="disabled")
                else:
                    result = getattr(self.capture, method)()
                if result["mode"] == "fault":
                    raise RuntimeError(result["message"])
                self.result_text.set(f"已执行：{self.buttons[key].cget('text')}")
            except FileNotFoundError:
                self.result_text.set("未执行：请先点击“设定初始位”")
            except Exception as error:
                self.result_text.set(f"未执行“{label}”：{error}")
            print(self.result_text.get(), flush=True)
            return

    def request_close(self):
        if self.closed or self.exit_requested:
            return
        state = self.capture.status()
        if state["mode"] == "returning":
            self.capture.cancel_home()
        if state["recording"]:
            self.capture.stop()
        self.exit_requested = True
        self.result_text.set("正在保存并退出，请稍候")
        for button in self.buttons.values():
            button.state(["disabled"])

    def refresh(self):
        if self.closed:
            return
        state = self.capture.status()
        if self.exit_requested and not state["saving"]:
            self.closed = True
            if self.timer is not None:
                self.root.after_cancel(self.timer)
            self.root.destroy()
            return
        busy = state["recording"] or state["saving"]
        if self.was_recording and not busy:
            self.suggest_index()
        self.was_recording = busy
        fps = state["fps"]
        entry_state = "disabled" if state["recording"] or state["saving"] or self.exit_requested else "normal"
        for entry in (self.task_entry, self.capture_index_entry):
            entry.configure(state=entry_state)
        self.status_text.set(f"{MODES.get(state['mode'], state['mode'])}  |  action {fps['action']:.1f} Hz"
                             f"  |  写盘 {fps['recording']:.1f} FPS  |  已保存 {state['frames']} 帧"
                             f"  |  丢帧 {state['dropped']}")
        reasons = ", ".join(f"{DROP_REASONS.get(k.split(':')[0], k)} {v}"
                            for k, v in state["drop_reasons"].items())
        self.device_text.set(f"双臂反馈 {fps['feedback']['left']:.1f}/{fps['feedback']['right']:.1f} Hz"
                             f"  |  {state['message']}" + (f"  |  {reasons}" if reasons else ""))
        image = render_views(self.capture, self.views.winfo_width(), self.views.winfo_height())
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        self.photo = tk.PhotoImage(master=self.root, data=f"P6\n{w} {h}\n255\n".encode() + rgb.tobytes(), format="PPM")
        self.views.configure(image=self.photo)
        self.timer = self.root.after(33, self.refresh)

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("run", "discover", "discover-arms"), default="run")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    if args.command == "discover-arms":
        devices = discover_can()
        print(f"检测到 {len(devices)} 路 USB-CAN；以下编号不代表左右或主从，请按接线核对。")
        print(json.dumps(devices, indent=2, ensure_ascii=False))
        return
    cfg = load_config(args.config)
    if args.command == "discover":
        print(json.dumps(discover(cfg["realsense"]), indent=2))
        return
    print("请直接点击电脑预览窗口下方的中文按钮操作。", flush=True)
    print("手动拖动请使用机械臂自身的示教模式。返回初始位前松手并清空运动路径。", flush=True)
    home_path = "outputs/mock_home.json" if args.mock else "home.json"
    with Capture(cfg, mock=args.mock, home_path=home_path) as capture:
        CaptureWindow(capture).run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
