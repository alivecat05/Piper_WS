"""Startup failures must not stop hardware; recovery must be explicit and verified."""
import asyncio
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

from piperx_teleop import app
from piperx_teleop.cameras import Cameras
from piperx_teleop.controller import Controller
from piperx_teleop.robot import MockPiperX, PiperX


def config():
    cfg = app.load_config(Path(__file__).resolve().parents[1] / "config.json")
    cfg["feedback"] = {"sound": False, "haptics": False}
    for index, camera in enumerate(cfg["cameras"].values()):
        camera["serial"] = str(index)
    for arm in cfg["arms"].values():
        arm["firmware"] = "v189"
    return cfg


def status(code=0, error=0):
    return SimpleNamespace(arm_status=SimpleNamespace(value=code), err_code=error)


def test_busy_port_does_not_open_hardware_or_change_adb(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Startup touched hardware/ADB before reserving its port")
    monkeypatch.setattr(app, "run_adb", forbidden)
    monkeypatch.setattr(app, "PiperX", forbidden)
    monkeypatch.setattr(Cameras, "start", forbidden)
    with app.reserve_port("127.0.0.1", 0) as occupied:
        args = SimpleNamespace(mock=False, cert=None, ip=None, host="0.0.0.0",
                               transport="usb", port=occupied.getsockname()[1])
        with pytest.raises(RuntimeError, match="尚未连接机器人"):
            asyncio.run(app.run(args, config()))


@pytest.mark.parametrize("started", [False, True])
def test_stop_only_after_control_commands(started, monkeypatch):
    cfg = config()
    robots = {side: MockPiperX(arm) for side, arm in cfg["arms"].items()}
    stopped, closed = [], []
    for side, robot in robots.items():
        monkeypatch.setattr(robot, "stop", lambda side=side: stopped.append(side))
        monkeypatch.setattr(robot, "close", lambda side=side: closed.append(side))
    control = Controller(cfg, robots, Cameras(cfg, mock=True), mock=True)
    control.commands_started = started
    control._fault("test fault")
    assert set(stopped) == (set(robots) if started else set())
    stopped.clear()
    control.close()
    assert set(stopped) == (set(robots) if started else set())
    assert set(closed) == set(robots)


def test_recover_requires_supported_arms(monkeypatch):
    monkeypatch.setattr(app, "PiperX", lambda *_: pytest.fail("Opened hardware without support confirmation"))
    with pytest.raises(ValueError, match="失电"):
        app.recover_robots(SimpleNamespace(supported=False), config())


@pytest.mark.parametrize("initial,errors,reset_count", [(0, 0, 0), (1, 0, 1), (4, 0, 1),
    (2, 0, 0), (5, 0, 0), (7, 0, 0), (1, 1, 0), (4, 1, 0)])
def test_recover_only_resets_allowed_latches_and_never_enables(initial, errors, reset_count):
    robot = PiperX.__new__(PiperX)
    calls, checks = [], []
    # No enable/move methods: a call to either fails this test.
    robot.robot = SimpleNamespace(reset=lambda: calls.append("reset"))
    def fresh_status(**kwargs):
        checks.append(kwargs)
        return status(initial, errors) if len(checks) == 1 else status()
    robot.recovery_status = fresh_status
    if errors or initial not in (0, 1, 4):
        with pytest.raises(RuntimeError, match="其他故障"):
            robot.recover()
    else:
        robot.recover()
    assert len(calls) == reset_count
    if reset_count:
        assert checks[1]["after"] > 0  # Must await feedback newer than reset.


def test_recovery_rejects_stale_normal_feedback():
    robot = PiperX.__new__(PiperX)
    robot.robot = SimpleNamespace(get_arm_status=lambda: SimpleNamespace(timestamp=time.time()-1, msg=status()))
    with pytest.raises(TimeoutError, match="新鲜"):
        robot.recovery_status(timeout=.03)


def test_recovery_timeout_does_not_repeat_reset():
    robot = PiperX.__new__(PiperX)
    calls = []
    robot.robot = SimpleNamespace(reset=lambda: calls.append("reset"))
    def fresh_status(**kwargs):
        if kwargs:
            raise TimeoutError("No fresh feedback after reset")
        return status(1)
    robot.recovery_status = fresh_status
    with pytest.raises(TimeoutError):
        robot.recover()
    assert calls == ["reset"]


def test_target_fault_recovery_waits_for_normal_without_repeating_reset():
    robot = PiperX.__new__(PiperX)
    calls = []
    robot.robot = SimpleNamespace(reset=lambda: calls.append("reset"))
    reports = iter([status(4), status(4), status(1), status(0)])
    robot.recovery_status = lambda **_: next(reports)
    assert "已清除" in robot.recover()
    assert calls == ["reset"]


def test_recovery_stops_if_a_new_hardware_fault_appears():
    robot = PiperX.__new__(PiperX)
    calls = []
    robot.robot = SimpleNamespace(reset=lambda: calls.append("reset"))
    reports = iter([status(4), status(5, 1)])
    robot.recovery_status = lambda **_: next(reports)
    with pytest.raises(RuntimeError, match="恢复后状态异常"):
        robot.recover()
    assert calls == ["reset"]


@pytest.mark.parametrize("right_code,right_error,allowed", [(4, 0, True), (4, 1, False), (7, 0, False)])
def test_cli_checks_both_arms_before_recovering_target_fault(monkeypatch, right_code, right_error, allowed):
    calls, closed = [], []
    sides = iter(["left", "right"])
    def make_robot(_):
        side = next(sides)
        return SimpleNamespace(
            recovery_status=lambda: status(1) if side == "left" else status(right_code, right_error),
            recover=lambda: calls.append(side), close=lambda: closed.append(side))
    monkeypatch.setattr(app, "PiperX", make_robot)
    args = SimpleNamespace(supported=True, host="127.0.0.1", port=0)
    if allowed:
        app.recover_robots(args, config())
        assert calls == ["left", "right"]
    else:
        with pytest.raises(RuntimeError, match="未执行恢复"):
            app.recover_robots(args, config())
        assert calls == []
    assert closed == ["left", "right"]


def test_input_test_does_not_construct_real_robot(monkeypatch):
    original_cameras = app.Cameras
    def cameras(cfg, mock):
        assert mock is False  # Real camera path requested; replace only for the unit test.
        return original_cameras(cfg, mock=True)
    monkeypatch.setattr(app, "Cameras", cameras)
    monkeypatch.setattr(app, "PiperX", lambda *_: pytest.fail("Input test opened real robot"))
    monkeypatch.setattr(app, "run_adb", lambda *_: None)
    monkeypatch.setattr(app, "Preview", lambda suffix: SimpleNamespace(
        update=lambda cameras, status: False, close=lambda: None))
    args = SimpleNamespace(mock=False, input_test=True, cert=None, ip=None, host="0.0.0.0",
                           transport="usb", port=0, adb="adb", device=None,
                           config="config.json", no_preview=False)
    asyncio.run(app.run(args, config()))
