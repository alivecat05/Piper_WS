"""The only module that calls the official pyAgxArm SDK."""
import logging
import time

import numpy as np

from .core import pose_to_vector, vector_to_pose

# Clear controller latches only; any joint/communication error bits still block recovery.
RECOVERABLE_STATUSES = (0, 1, 4)  # NORMAL, EMERGENCY_STOP, TARGET_POS_EXCEEDS_LIMIT


class PiperX:
    def __init__(self, cfg):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

        if cfg["firmware"] not in ("default", "v183", "v188", "v189"):
            raise ValueError("Set each arm's firmware profile explicitly after checking its firmware")
        self.cfg = cfg
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

    def _stamp(self, message):
        # agx_cando/python-can receive timestamps are Unix seconds.
        stamp = float(message.timestamp) + self.clock_offset
        if abs(time.perf_counter() - stamp) > 2:
            raise RuntimeError("CAN timestamp is not host Unix time, or feedback is stale")
        return stamp

    def read(self):
        pose = self.robot.get_tcp_pose()
        joints = self.robot.get_joint_angles()
        grip = self.gripper.get_gripper_status()
        status = self.robot.get_arm_status()
        if any(x is None for x in (pose, joints, grip, status)):
            return None
        if status.msg.arm_status.value != 0 or status.msg.err_code != 0:
            logging.getLogger(__name__).info("Robot status: %s", status.msg)
            if status.msg.arm_status.value == 1:
                raise RuntimeError("机械臂处于急停；放稳并支撑机械臂后运行 .\\teleop.cmd recover --supported")
            raise RuntimeError(f"Robot error: {status.msg.arm_status}, code=0x{status.msg.err_code:04x}")
        if grip.msg.mode != "width":
            raise RuntimeError("Gripper must report width in meters")
        fields = {"tcp": self._stamp(pose), "joints": self._stamp(joints),
                  "gripper": self._stamp(grip), "status": self._stamp(status)}
        # SDK 1.0.0 assembles a pose from three CAN frames. Check all parts,
        # rather than letting one fresh fragment hide two stale fragments.
        parser = self.robot._parser
        for name in ("end_pose_xy", "end_pose_zrx", "end_pose_ryrz", "joint_12", "joint_34", "joint_56"):
            fragment = getattr(parser, name, None)
            if fragment is None:
                return None
            fields[name] = self._stamp(fragment)
        return max(fields.values()), {
            "vector": pose_to_vector(list(pose.msg), float(grip.msg.value)),
            "joints": list(joints.msg), "source_timestamps": fields,
            "gripper_enabled": bool(grip.msg.foc_status.driver_enable_status),
            "gripper_status": int(grip.msg.status_code),
        }

    def enable(self):
        self.robot.set_speed_percent(self.cfg["speed_percent"])
        return self.robot.enable()  # Nonblocking; controller keeps checking its watchdogs.

    def recovery_status(self, after=0, timeout=3):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            status = self.robot.get_arm_status()
            if status and status.timestamp > after and -.05 <= time.time() - status.timestamp <= .15:
                return status.msg
            time.sleep(.01)
        raise TimeoutError("恢复检查未收到新鲜机器人状态")

    def recover(self):
        status = self.recovery_status()
        if status.err_code or status.arm_status.value not in RECOVERABLE_STATUSES:
            raise RuntimeError(f"存在其他故障，不能恢复：{status.arm_status}, code=0x{status.err_code:04x}")
        if status.arm_status.value == 0:
            return "状态正常，无需恢复"
        sent = time.time()
        self.robot.reset()  # Cuts motor power; CLI requires supported arms. Never enables or moves.
        deadline = time.perf_counter() + 3
        while time.perf_counter() < deadline:
            status = self.recovery_status(after=sent, timeout=max(.01, deadline-time.perf_counter()))
            if status.err_code or status.arm_status.value not in RECOVERABLE_STATUSES:
                raise RuntimeError(f"恢复后状态异常：{status.arm_status}, code=0x{status.err_code:04x}")
            if status.arm_status.value == 0:
                return "控制器故障已清除；未发送使能或运动命令"
            time.sleep(.02)
        raise TimeoutError("控制器故障仍未清除；请检查物理急停和控制器状态")

    def send(self, vector):
        flange = self.robot.get_tcp2flange_pose(vector_to_pose(vector))
        self.robot.move_p(list(flange))
        self.send_gripper(vector[7])

    def send_gripper(self, width):
        self.gripper.move_gripper_m(float(width), force=self.cfg["gripper_force_n"])

    def move_home(self, joints):
        self.robot.set_speed_percent(self.cfg["reset_speed_percent"])
        self.robot.move_j(list(joints))

    def cartesian_start_issue(self, joints):
        # Holding measured joints and entering Cartesian IK are different operations.
        # An exactly-on-limit pose may reject even a tiny Cartesian step. Require room to move.
        limits = self.robot.get_config()["joint_limits"]
        margin = np.deg2rad(2)
        for index, angle in enumerate(joints, 1):
            low, high = limits[f"joint{index}"]
            if not low + margin < angle < high - margin:
                return (f"J{index}={np.rad2deg(angle):.1f}deg near/outside limit; "
                        "set a working pose (>2deg from limits)")
        return ""

    def hold(self, joints):
        if np.asarray(joints).shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("保持位置需要六个有效的实测关节角")
        # Only an enabled SDK software limit clamps move_j. Presets alone are not active limits.
        # Keep fresh measured angles unchanged; never disable limits or clamp a hold target.
        if self.robot.get_joint_limits_enabled():
            limits = self.robot.get_config()["joint_limits"]
            for index, angle in enumerate(joints, 1):
                low, high = limits[f"joint{index}"]
                if not low <= angle <= high:
                    raise ValueError(f"机械臂实测 J{index}={angle:.4f} rad 超出已启用的软件限位"
                                     f" [{low:.4f}, {high:.4f}]；无法原位接管")
        self.robot.move_j(list(joints))

    def stop(self):
        self.robot.electronic_emergency_stop()

    def close(self):
        self.robot.disconnect()


class MockPiperX:
    def __init__(self, cfg):
        self.cfg = cfg
        self.vector = pose_to_vector([.3, 0, .3, 0, 0, 0], .04)
        self.joints = [0.] * 6

    def read(self):
        t = time.perf_counter()
        return t, {"vector": list(self.vector), "joints": list(self.joints),
                   "source_timestamps": dict.fromkeys(("tcp", "joints", "gripper", "status"), t)}

    def enable(self):
        return True

    def send(self, vector):
        self.vector = list(vector)

    def send_gripper(self, width):
        self.vector[7] = float(width)

    def move_home(self, joints):
        self.joints = list(joints)
        self.vector = pose_to_vector([.3, 0, .3, 0, 0, 0], self.vector[7])

    def hold(self, joints):
        self.joints = list(joints)

    def cartesian_start_issue(self, joints):
        return ""

    def stop(self):
        pass

    def close(self):
        pass


def validate_vector(vector, cfg):
    array = np.asarray(vector)
    if array.shape != (8,) or not np.isfinite(array).all():
        raise ValueError("Invalid robot target")
    if (np.any(array[:3] < cfg["workspace_min"]) or np.any(array[:3] > cfg["workspace_max"])
            or not 0 <= array[7] <= cfg["gripper_max_m"]):
        raise ValueError("Target outside configured workspace/gripper limits")

