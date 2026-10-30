"""CLI, HTTPS/WebSocket bridge, and lifecycle. No robot commands in HTTP handlers."""
import argparse
import asyncio
from datetime import datetime, timedelta, timezone
import ipaddress
import json
import logging
from pathlib import Path
import ssl
import socket
import subprocess
import sys
import time

import numpy as np
from aiohttp import web, WSMsgType

from .cameras import Cameras, discover
from .controller import Controller
from .core import ARMS, CAMERAS
from .preview import Preview
from .robot import MockPiperX, PiperX, RECOVERABLE_STATUSES


class ConsoleFormatter(logging.Formatter):
    """Keep exception details in the file, not on the operator's terminal."""
    def format(self, record):
        return f"{record.levelname}: {record.getMessage()}"


def validate_hardware(cfg):
    missing = [f"cameras.{name}.serial" for name, camera in cfg["cameras"].items() if not camera["serial"]]
    missing += [f"arms.{side}.firmware" for side, arm in cfg["arms"].items() if not arm["firmware"]]
    if missing:
        raise ValueError("请先填写 config.json：" + ", ".join(missing) + "。未接硬件时请加 --mock。")


def adb_command(args):
    # Select USB explicitly; do not accidentally target an unrelated Wi-Fi device.
    return [args.adb, "-s", args.device] if args.device else [args.adb, "-d"]


def run_adb(command, *arguments):
    try:
        result = subprocess.run([*command, *arguments], check=True, timeout=10,
                                capture_output=True, text=True, encoding="utf-8", errors="replace")
    except FileNotFoundError as error:
        raise RuntimeError("找不到 ADB；请安装 Android Platform Tools，或用 --adb 指定 adb.exe。") from error
    except subprocess.CalledProcessError as error:
        detail = (error.stderr or error.stdout or "未知错误").strip()
        if "unauthorized" in detail:
            detail = "Quest 尚未授权；请在头显内点击“允许 USB 调试”，再重试"
        elif "no devices" in detail or "device offline" in detail:
            detail = "未找到在线 Quest；请检查 USB 连接、开发者模式和 USB 调试授权"
        elif "more than one" in detail:
            detail = "连接了多个设备；请用 --device 指定 Quest 序列号"
        else:
            detail = " ".join(detail.splitlines())
        raise RuntimeError(f"ADB 失败：{detail}") from error
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("ADB 超时；请检查 Quest USB 连接与调试授权。") from error
    # Android may report a rejected broadcast even when adb itself exits with zero.
    if any(word in result.stdout.lower() for word in ("securityexception", "permission denial", "error:")):
        raise RuntimeError(f"Quest 拒绝命令：{result.stdout.strip()}")
    return result.stdout


def set_quest_awake(args):
    # Meta documents prox_close for off-face operation, automation_disable to restore the sensor.
    action = "prox_close" if args.command == "awake" else "automation_disable"
    run_adb(adb_command(args), "shell", "am", "broadcast", "-a", f"com.oculus.vrpowermanager.{action}")
    if args.command == "awake":
        print("已发送挂脖保活命令。先用 --mock 确认离脸后追踪持续；头显重启后需重新执行。")
    else:
        print("已发送恢复佩戴检测命令。")


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    if set(cfg["cameras"]) != set(CAMERAS) or set(cfg["arms"]) != set(ARMS):
        raise ValueError("Exactly three camera roles and two arm roles are required")
    if not 1 <= cfg["record_hz"] <= 60 or not 1 <= cfg["control_hz"] <= 120:
        raise ValueError("Invalid recording/control frequency")
    cfg.setdefault("camera_timeout_s", .25)
    if not 0 < cfg["camera_timeout_s"] <= 2:
        raise ValueError("Camera timeout must be between 0 and 2 seconds")
    if cfg["sync"]["delay_ms"] < cfg["sync"]["camera_tolerance_ms"]:
        raise ValueError("Synchronization delay must cover the camera matching tolerance")
    for arm in cfg["arms"].values():
        for waypoint in arm["home_waypoints"]:
            if np.asarray(waypoint).shape != (6,) or not np.isfinite(waypoint).all():
                raise ValueError("Each home waypoint must contain six finite joint angles in radians")
    return cfg


def reserve_port(host, port):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        listener.bind((host, port))
        listener.listen(128)
        listener.setblocking(False)
        return listener
    except OSError as error:
        listener.close()
        raise RuntimeError(f"无法监听 {host}:{port}；请先关闭占用端口的旧程序（含模拟窗口）。尚未连接机器人。") from error


def recover_robots(args, cfg):
    if not args.supported:
        raise ValueError("恢复会令电机失电，悬空机械臂会下落。先放稳并支撑双臂，再运行 recover --supported。")
    # Refuse recovery while this project's normal listener is still running.
    with reserve_port(args.host, args.port):
        robots = {}
        try:
            for side in ARMS:
                robots[side] = PiperX(cfg["arms"][side])
            for side, robot in robots.items():
                status = robot.recovery_status()
                if status.err_code or status.arm_status.value not in RECOVERABLE_STATUSES:
                    raise RuntimeError(f"{side}: 存在其他故障，未执行恢复：{status.arm_status}")
            for side, robot in robots.items():
                print(f"{side}: {robot.recover()}", flush=True)
        finally:
            for robot in robots.values():
                robot.close()


def validate_packet(data):
    if data.get("type") != "input" or type(data.get("active")) is not bool:
        raise ValueError("Invalid input packet")
    if data.get("inactive_reason", "") not in ("", "server_ack_stale", "session_hidden", "session_blurred", "viewer_pose_missing"):
        raise ValueError("Invalid XR session reason")
    if set(data["buttons"]) != {"record_start", "record_stop", "reset", "set_home"}:
        raise ValueError("Invalid button names")
    if any(type(v) is not bool for v in data["buttons"].values()):
        raise ValueError("Buttons must be booleans")
    events = data.get("events", [])
    if not isinstance(events, list) or len(events) > 64:
        raise ValueError("Too many button events")
    for event in events:
        if (not isinstance(event, dict) or type(event.get("id")) is not int or event["id"] < 1
                or event.get("name") not in data["buttons"] or type(event.get("down")) is not bool
                or not 0 <= float(event.get("age_ms", -1)) <= 200):
            raise ValueError("Invalid or stale button event")
    for side in ARMS:
        hand = data[side]
        if hand.get("tracking_reason", "") not in ("", "controller_missing", "grip_space_missing", "gamepad_mapping", "pose_missing", "position_emulated"):
            raise ValueError("Invalid controller tracking reason")
        if type(hand["tracked"]) is not bool or type(hand["grip"]) is not bool:
            raise ValueError("Invalid tracking/grip flags")
        position, quat = np.asarray(hand["position"], dtype=float), np.asarray(hand["quaternion"], dtype=float)
        if (position.shape != (3,) or quat.shape != (4,) or not np.isfinite(position).all()
                or not np.isfinite(quat).all() or not .99 < np.linalg.norm(quat) < 1.01
                or np.linalg.norm(position) > 10 or not 0 <= float(hand["trigger"]) <= 1):
            raise ValueError("Invalid controller pose or trigger")
    return data


def make_app(controller):
    client = None
    web_dir = Path(__file__).parent / "web"

    async def index(request):
        return web.FileResponse(web_dir / "index.html", headers={"Cache-Control": "no-store"})

    async def script(request):
        return web.FileResponse(web_dir / "quest.js", headers={"Cache-Control": "no-store"})

    async def status(request):
        return web.json_response(controller.status(), headers={"Cache-Control": "no-store"})

    async def socket(request):
        nonlocal client
        if client is not None:
            raise web.HTTPConflict(text="One Quest controller session at a time")
        ws = web.WebSocketResponse(max_msg_size=16384, heartbeat=2)
        client = ws
        publisher = None
        async def publish_status():
            while not ws.closed:
                await ws.send_json({"type": "status", "server_time_s": time.perf_counter(), **controller.status()})
                await asyncio.sleep(.03)
        try:
            await ws.prepare(request)
            controller.begin_input()
            publisher = asyncio.create_task(publish_status())
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    data = json.loads(message.data)
                    if data.get("type") == "ping":
                        await ws.send_json({"type": "pong", "sent_ms": data["sent_ms"]})
                        continue
                    # Browser checks ack freshness; server additionally rejects queued/stale input.
                    age = float(data.get("age_ms", 1e9))
                    if not 0 <= age <= 100:
                        raise ValueError("Stale XR frame")
                    if not 0 <= time.perf_counter() - float(data.get("server_time_s", 0)) <= .2:
                        raise ValueError("Input has no recent server acknowledgement")
                    controller.accept(validate_packet(data))
                elif message.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
                    break
        except (ValueError, KeyError, TypeError, asyncio.TimeoutError) as error:
            logging.getLogger(__name__).info("Quest input rejected: %s", error)
            await ws.close(code=1008, message=b"Invalid or stale input")
        finally:
            controller.disconnect()
            if publisher:
                publisher.cancel()
                await asyncio.gather(publisher, return_exceptions=True)
            client = None
        return ws

    app = web.Application(client_max_size=16384)
    app.add_routes([web.get("/", index), web.get("/quest.js", script), web.get("/status", status), web.get("/ws", socket)])
    return app


def make_certificate(host):
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID

    root = Path("certs")
    root.mkdir(exist_ok=True)
    if (root / "cert.pem").exists() or (root / "key.pem").exists():
        raise FileExistsError("Certificate already exists; use it or select a new directory manually")
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, host)])
    try:
        name = x509.IPAddress(ipaddress.ip_address(host))
    except ValueError:
        name = x509.DNSName(host)
    now = datetime.now(timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(subject).issuer_name(subject).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - timedelta(minutes=5))
            .not_valid_after(now + timedelta(days=365)).add_extension(x509.SubjectAlternativeName([name]), False)
            .sign(key, hashes.SHA256()))
    (root / "key.pem").write_bytes(key.private_bytes(serialization.Encoding.PEM,
        serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    (root / "cert.pem").write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    print("Created certs/cert.pem and certs/key.pem")


async def run(args, cfg):
    input_test = getattr(args, "input_test", False)
    simulated_robots = args.mock or input_test
    cameras = Cameras(cfg, args.mock)
    robots, controller, preview, runner = {}, None, None, None
    usb_adb = None
    listener = None
    try:
        tls = None
        address = args.ip or (args.host if args.host != "0.0.0.0" else "localhost")
        if args.cert:
            from cryptography import x509
            tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            tls.load_cert_chain(args.cert, args.key)
            if not args.ip:
                cert = x509.load_pem_x509_certificate(Path(args.cert).read_bytes())
                try:
                    names = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
                    addresses = names.get_values_for_type(x509.IPAddress) or names.get_values_for_type(x509.DNSName)
                    if addresses:
                        address = str(addresses[0])
                except x509.ExtensionNotFound:
                    pass
        elif not args.mock and args.transport == "wifi":
            raise ValueError("Wi-Fi robot mode requires --cert and --key for Quest WebXR")
        if not simulated_robots:
            validate_hardware(cfg)
            addresses = [(a["interface"], a["channel"]) for a in cfg["arms"].values()]
            if len(set(addresses)) != 2:
                raise ValueError("Left and right arms must use distinct CAN interfaces/channels")
        host = "127.0.0.1" if args.transport == "usb" else args.host
        # Bind before ADB, cameras or robots: a second launch must have no hardware side effects.
        listener = reserve_port(host, args.port)
        if args.transport == "usb":
            # -d selects a physical USB device, excluding Wi-Fi adb sessions.
            adb_args = adb_command(args)
            run_adb(adb_args, "reverse", f"tcp:{args.port}", f"tcp:{args.port}")
            usb_adb = adb_args
        cameras.start()
        for side in ARMS:
            robots[side] = (MockPiperX if simulated_robots else PiperX)(cfg["arms"][side])
        controller = Controller(cfg, robots, cameras, simulated_robots, args.config)
        runner = web.AppRunner(make_app(controller))
        await runner.setup()
        await web.SockSite(runner, listener, ssl_context=tls).start()
        controller.start()
        address = "localhost" if args.transport == "usb" else address
        label = "Quest USB URL" if args.transport == "usb" else "Quest Wi-Fi URL"
        print(f"{label}: {'https' if tls else 'http'}://{address}:{args.port}", flush=True)
        if input_test:
            print("手柄测试：真实相机 + 模拟机械臂；不连接 CAN，不控制真实机械臂。", flush=True)
        print("B: 开始录制 | A: 保存 | 中指键: 按住使能跟随/松开保持 | 食指扳机: 夹爪闭合/张开 | Y长按: 回初始位 | X长按: 保存初始位", flush=True)
        if not args.no_preview:
            preview = Preview(" | INPUT TEST - ROBOTS DISCONNECTED" if input_test else "")
        while True:
            if preview and not preview.update(cameras, controller.status()):
                break
            await asyncio.sleep(.03)
    finally:
        if controller:
            await asyncio.to_thread(controller.close)
        else:
            for robot in robots.values():
                robot.close()
        if runner:
            await runner.cleanup()
        if listener:
            listener.close()
        cameras.close()
        if preview:
            preview.close()
        if usb_adb:
            try:
                run_adb(usb_adb, "reverse", "--remove", f"tcp:{args.port}")
            except RuntimeError as error:
                logging.getLogger(__name__).warning("清理 USB 转发失败：%s", error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("run", "discover", "cert", "awake", "sleep", "recover"), nargs="?", default="run")
    parser.add_argument("--supported", action="store_true", help="Confirm both arms are supported before motor-power-off recovery")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--input-test", action="store_true", help="Real RGB + Quest diagnostics; no robot connection or commands")
    parser.add_argument("--transport", choices=("wifi", "usb"), default="wifi")
    bundled_adb = Path(__file__).resolve().parents[1] / ".setup/platform-tools/adb.exe"
    parser.add_argument("--adb", default=str(bundled_adb) if bundled_adb.is_file() else "adb",
                        help="Android platform-tools adb.exe path")
    parser.add_argument("--device", help="ADB serial when more than one headset is attached")
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8012)
    parser.add_argument("--cert")
    parser.add_argument("--key")
    parser.add_argument("--ip", help="Computer Wi-Fi adapter IPv4 address for the certificate or Quest URL")
    args = parser.parse_args()
    log_path = Path("outputs/logs/teleop.log")
    file_log = None
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_log = logging.FileHandler(log_path, encoding="utf-8")
        file_log.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        console = logging.StreamHandler()
        console.setLevel(logging.WARNING)
        console.setFormatter(ConsoleFormatter())
        logging.basicConfig(level=logging.INFO, handlers=[file_log, console], force=True)
        if args.command == "cert":
            if not args.ip:
                parser.error("cert requires --ip <computer Wi-Fi IPv4 address>")
            make_certificate(args.ip)
        elif args.command == "discover":
            print(json.dumps(discover(load_config(args.config)["realsense"]), indent=2))
        elif args.command in ("awake", "sleep"):
            set_quest_awake(args)
        elif args.command == "recover":
            recover_robots(args, load_config(args.config))
        else:
            asyncio.run(run(args, load_config(args.config)))
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        if file_log is not None:
            logging.getLogger(__name__).exception("程序未能运行：%s；详情见 %s", error, log_path)
        else:
            print(f"程序未能运行：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

