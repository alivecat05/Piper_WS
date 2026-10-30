"""Exercise the actual controller-only WebSocket protocol with simulated hardware."""
import asyncio
import time
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer
from aiohttp import WSServerHandshakeError
import pytest

from piperx_teleop.app import load_config, make_app
from piperx_teleop.cameras import Cameras
from piperx_teleop.controller import Controller
from piperx_teleop.core import ARMS
from piperx_teleop.robot import MockPiperX


def test_status_http_does_not_claim_or_reset_controller(tmp_path):
    async def exercise():
        cfg = load_config(Path(__file__).resolve().parents[1]/"config.json")
        cfg["output"] = str(tmp_path)
        cfg["feedback"] = {"sound": False, "haptics": False}
        cameras = Cameras(cfg, mock=True)
        control = Controller(cfg, {s: MockPiperX(cfg["arms"][s]) for s in ARMS}, cameras, mock=True)
        client = TestClient(TestServer(make_app(control)))
        try:
            await client.start_server()
            socket = await client.ws_connect("/ws")
            await socket.receive_json()
            control.event_ack = 7
            for _ in range(2):
                response = await client.get("/status")
                data = await response.json()
                assert data["event_ack"] == 7 and data["quest"]["connected"]
            assert not socket.closed and control.packet is None
            await socket.close()
        finally:
            await client.close()
            control.close()
    asyncio.run(exercise())


def test_bridge_plain_url_input_and_disconnect(tmp_path):
    async def exercise():
        cfg = load_config(Path(__file__).resolve().parents[1]/"config.json")
        cfg["output"] = str(tmp_path)
        cfg["feedback"] = {"sound": False, "haptics": False}
        cameras = Cameras(cfg, mock=True)
        robots = {side: MockPiperX(cfg["arms"][side]) for side in ARMS}
        control = Controller(cfg, robots, cameras, mock=True)
        cameras.start()
        control.start()
        client = TestClient(TestServer(make_app(control)))
        try:
            await client.start_server()
            await asyncio.sleep(.1)
            response = await client.get("/")
            assert response.status == 200 and "启动手柄追踪" in await response.text()
            socket = await client.ws_connect("/ws")
            status = await asyncio.wait_for(socket.receive_json(), timeout=1)
            assert status["mode"] == "waiting" and not status["ready"]
            with pytest.raises(WSServerHandshakeError) as duplicate:
                await client.ws_connect("/ws")
            assert duplicate.value.status == 409
            hand = {"tracked": True, "grip": True, "trigger": .4,
                    "position": [0, 1, 0], "quaternion": [0, 0, 0, 1]}
            data = {"type": "input", "active": True, "left": hand, "right": hand,
                    "age_ms": 0, "server_time_s": status["server_time_s"],
                    "buttons": {"record_start": False, "record_stop": False, "reset": False, "set_home": False}}
            await socket.send_json(data)
            await socket.receive_json()
            await asyncio.sleep(.05)
            assert control.mode == "teleop"
            await socket.close()
            deadline = time.perf_counter()+.3
            while control.mode != "paused" and time.perf_counter() < deadline:
                await asyncio.sleep(.01)
            assert control.mode == "paused"
        finally:
            await client.close()
            control.close()
            cameras.close()
    asyncio.run(exercise())


def test_button_execution_ack_and_pause_notification_without_new_input(tmp_path):
    async def exercise():
        cfg = load_config(Path(__file__).resolve().parents[1]/"config.json")
        cfg["output"] = str(tmp_path)
        cfg["feedback"] = {"sound": False, "haptics": True}
        cameras = Cameras(cfg, mock=True)
        control = Controller(cfg, {side: MockPiperX(cfg["arms"][side]) for side in ARMS}, cameras, mock=True)
        cameras.start()
        control.start()
        client = TestClient(TestServer(make_app(control)))
        try:
            await client.start_server()
            await asyncio.sleep(.1)
            socket = await client.ws_connect("/ws")
            status = await socket.receive_json()
            hand = {"tracked": True, "grip": False, "trigger": .4,
                    "position": [0, 1, 0], "quaternion": [0, 0, 0, 1]}
            data = {"type": "input", "active": True, "left": hand, "right": hand,
                    "age_ms": 0, "server_time_s": status["server_time_s"],
                    "buttons": dict.fromkeys(("record_start", "record_stop", "reset", "set_home"), False),
                    "events": [{"id": 1, "name": "record_start", "down": True, "age_ms": 0},
                               {"id": 2, "name": "record_start", "down": False, "age_ms": 0}]}
            await socket.send_json(data)
            async with asyncio.timeout(1):
                while status["event_ack"] < 2:
                    status = await socket.receive_json()
            assert status["recording"] and status["haptics"]
            assert [n["kind"] for n in status["notifications"]].count("record_started") == 1
            data["server_time_s"] = status["server_time_s"]
            await socket.send_json(data)  # Duplicate command IDs must not start another episode.
            async with asyncio.timeout(1):
                while status["mode"] != "paused":
                    status = await socket.receive_json()
            assert [n["kind"] for n in status["notifications"]].count("record_started") == 1
            assert status["notifications"][-1]["kind"] == "paused"
            await socket.close()
        finally:
            await client.close()
            control.close()
            cameras.close()
    asyncio.run(exercise())
