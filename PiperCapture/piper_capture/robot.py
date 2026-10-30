"""Read-only feedback during capture; motor commands are used only for explicit homing."""
import time

import numpy as np

from .core import pose_to_vector


def discover_can():
    """Enumerate USB adapters only; never enable or send motion to a robot."""
    from agx_cando.bus import _CandoApi
    api = _CandoApi()
    devices = []
    for channel in range(api.scan()):
        info = api.describe(channel)
        devices.append({"interface": "agx_cando", "channel": str(channel),
                        "adapter_id": info.serial})
    return devices


class PiperX:
    def __init__(self, cfg):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config
        self.cfg = cfg
        self.feedback_issue = "Waiting for CAN feedback"
        self.clock_offset = time.perf_counter() - time.time()
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER_X, firmeware_version=cfg["firmware"],
            interface=cfg["interface"], channel=cfg["channel"], bitrate=1_000_000))
        try:
            self.robot.connect()
            self.robot.set_tcp_offset(cfg["tcp_offset"])
            self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        except Exception:
            self.robot.disconnect()
            raise

    def _stamp(self, message, field):
        if message is None:
            self.feedback_issue = f"{field}: no feedback"
            return None
        raw = float(message.timestamp)
        stamp = raw + self.clock_offset
        age = time.perf_counter() - stamp
        if raw <= 0 or not np.isfinite(raw) or not -.05 <= age <= 2:
            self.feedback_issue = f"{field}: invalid timestamp" if raw <= 0 else f"{field}: feedback age {age:.3f}s"
            return None  # Never retimestamp cached data. Capture's watchdog handles persistent loss.
        return stamp

    def read(self):
        pose, joints = self.robot.get_tcp_pose(), self.robot.get_joint_angles()
        grip, status = self.gripper.get_gripper_status(), self.robot.get_arm_status()
        status_stamp = self._stamp(status, "status")
        if status_stamp is None:
            return None
        if status.msg.arm_status.value or status.msg.err_code:
            raise RuntimeError(f"Robot error: {status.msg.arm_status}, code={status.msg.err_code:#06x}")
        fields = {"status": status_stamp}
        messages = {"tcp": pose, "joints": joints, "gripper": grip}
        for name in ("end_pose_xy", "end_pose_zrx", "end_pose_ryrz", "joint_12", "joint_34", "joint_56"):
            messages[name] = getattr(self.robot._parser, name, None)
        for name, message in messages.items():
            stamp = self._stamp(message, name)
            if stamp is None:
                return None
            fields[name] = stamp
        if grip.msg.mode != "width":
            raise RuntimeError("Gripper feedback must be width in meters")
        self.feedback_issue = ""
        return max(fields.values()), {"vector": pose_to_vector(list(pose.msg), float(grip.msg.value)),
                                    "joints": list(joints.msg), "source_timestamps": fields}

    def validate_home(self, joints, width):
        if np.asarray(joints).shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("Home needs six finite joint angles")
        if not np.isfinite(width) or not 0 <= width <= self.cfg["gripper_max_m"]:
            raise ValueError("Home gripper width is outside its limits")
        limits = self.robot.get_config()["joint_limits"]
        for index, angle in enumerate(joints, 1):
            low, high = limits[f"joint{index}"]
            if not low <= angle <= high:
                raise ValueError(f"Home J{index}={angle:.4f} outside [{low:.4f}, {high:.4f}]")

    def hold(self, state):
        # Never alter/clamp a measured hold target, or change the SDK limit switch.
        if self.robot.get_joint_limits_enabled():
            self.validate_home(state["joints"], state["vector"][7])
        self.robot.move_j(list(state["joints"]))
        self.gripper.move_gripper_m(state["vector"][7], force=self.cfg["gripper_force_n"])

    def enable(self):
        self.robot.set_speed_percent(self.cfg["home_speed_percent"])
        return self.robot.enable()

    def move_home(self, home):
        self.validate_home(home["joints"], home["gripper_m"])
        self.robot.set_speed_percent(self.cfg["home_speed_percent"])
        self.robot.move_j(list(home["joints"]))
        self.gripper.move_gripper_m(home["gripper_m"], force=self.cfg["gripper_force_n"])

    def stop(self):
        self.robot.electronic_emergency_stop()

    def close(self):
        self.robot.disconnect()


class MockPiperX:
    def __init__(self, cfg):
        self.cfg = cfg
        self.joints = [0., .6, -.3, 0., 0., 0.]
        self.vector = pose_to_vector([.3, 0., .3, 0., 0., 0.], .04)
        self.commands = []

    def read(self):
        t = time.perf_counter()
        return t, {"vector": list(self.vector), "joints": list(self.joints),
                   "source_timestamps": dict.fromkeys(("tcp", "joints", "gripper", "status"), t)}

    def validate_home(self, joints, width):
        if len(joints) != 6 or not np.isfinite(joints).all() or not 0 <= width <= self.cfg["gripper_max_m"]:
            raise ValueError("Invalid home")

    def hold(self, state):
        self.commands.append("hold")
        self.joints, self.vector = list(state["joints"]), list(state["vector"])

    def enable(self):
        self.commands.append("enable")
        return True

    def move_home(self, home):
        self.commands.append("home")
        self.joints, self.vector[7] = list(home["joints"]), home["gripper_m"]

    def stop(self):
        self.commands.append("stop")

    def close(self):
        pass
