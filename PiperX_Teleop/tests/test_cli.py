"""Operator-facing startup failures stay concise; details remain in the log."""
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from piperx_teleop.app import load_config, validate_hardware, set_quest_awake, run_adb
import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_startup_error_has_no_terminal_traceback(tmp_path):
    result = subprocess.run(
        [sys.executable, "-m", "piperx_teleop.app", "--config", "missing.json"],
        cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=15,
        env={**os.environ, "PYTHONIOENCODING": "utf-8", "PYTHONPATH": str(ROOT)})
    assert result.returncode == 1
    terminal = result.stdout + result.stderr
    assert "Traceback" not in terminal and "missing.json" in terminal
    assert "teleop.log" in terminal
    log = (tmp_path / "outputs/logs/teleop.log").read_text(encoding="utf-8")
    assert "Traceback" in log and "FileNotFoundError" in log


def test_preflight_reports_all_missing_hardware_fields():
    cfg = load_config(ROOT / "config.json")
    for camera in cfg["cameras"].values():
        camera["serial"] = ""
    for arm in cfg["arms"].values():
        arm["firmware"] = None
    with pytest.raises(ValueError) as error:
        validate_hardware(cfg)
    message = str(error.value)
    for name in cfg["cameras"]:
        assert f"cameras.{name}.serial" in message
    for side in cfg["arms"]:
        assert f"arms.{side}.firmware" in message
    assert "--mock" in message


@pytest.mark.parametrize("command,device,selection,action", [
    ("awake", None, ["-d"], "prox_close"),
    ("sleep", "quest-serial", ["-s", "quest-serial"], "automation_disable"),
])
def test_quest_power_command(monkeypatch, capsys, command, device, selection, action):
    calls = []
    def fake_run(argv, **kwargs):
        calls.append(argv)
        assert kwargs["timeout"] == 10 and kwargs["check"]
        return subprocess.CompletedProcess(argv, 0, "Broadcast completed: result=0", "")
    monkeypatch.setattr("piperx_teleop.app.subprocess.run", fake_run)
    set_quest_awake(SimpleNamespace(command=command, device=device, adb="C:/tools/adb.exe"))
    assert calls == [["C:/tools/adb.exe", *selection, "shell", "am", "broadcast", "-a",
                      f"com.oculus.vrpowermanager.{action}"]]
    assert "已发送" in capsys.readouterr().out


@pytest.mark.parametrize("failure", ["missing", "unauthorized", "timeout", "denied"])
def test_quest_power_failure_is_reported(monkeypatch, failure):
    def fake_run(argv, **kwargs):
        if failure == "missing":
            raise FileNotFoundError()
        if failure == "unauthorized":
            raise subprocess.CalledProcessError(1, argv, stderr="device unauthorized")
        if failure == "timeout":
            raise subprocess.TimeoutExpired(argv, 10)
        return subprocess.CompletedProcess(argv, 0, "SecurityException: Permission Denial", "")
    monkeypatch.setattr("piperx_teleop.app.subprocess.run", fake_run)
    with pytest.raises(RuntimeError) as error:
        run_adb(["adb", "-d"], "shell", "am", "broadcast", "-a", "test")
    if failure == "unauthorized":
        assert "允许 USB 调试" in str(error.value)
