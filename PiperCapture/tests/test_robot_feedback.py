from types import SimpleNamespace as NS
import time

import numpy as np
import pytest

from piper_capture.robot import PiperX


def reader():
    arm = object.__new__(PiperX)
    arm.clock_offset = time.perf_counter() - time.time()
    arm.feedback_issue = ""
    now = time.time()
    messages = {
        "pose": NS(timestamp=now, msg=[.3, 0, .3, 0, 0, 0]),
        "joints": NS(timestamp=now, msg=[0, .6, -.3, 0, 0, 0]),
        "grip": NS(timestamp=now, msg=NS(mode="width", value=.04)),
        "status": NS(timestamp=now, msg=NS(arm_status=NS(value=0), err_code=0)),
    }
    parser = NS(**{n: NS(timestamp=now) for n in (
        "end_pose_xy", "end_pose_zrx", "end_pose_ryrz", "joint_12", "joint_34", "joint_56")})
    arm.robot = NS(get_tcp_pose=lambda: messages["pose"], get_joint_angles=lambda: messages["joints"],
                   get_arm_status=lambda: messages["status"], _parser=parser)
    arm.gripper = NS(get_gripper_status=lambda: messages["grip"])
    return arm, messages


@pytest.mark.parametrize("timestamp", [0, float("nan"), float("inf"), -1])
def test_uninitialized_timestamp_is_not_a_sample(timestamp):
    arm, messages = reader()
    messages["pose"].timestamp = timestamp
    assert arm.read() is None
    assert "tcp" in arm.feedback_issue


def test_stale_fragment_waits_then_fresh_feedback_recovers():
    arm, _ = reader()
    fragment = arm.robot._parser.joint_12
    fragment.timestamp = time.time() - 5
    assert arm.read() is None
    assert "joint_12" in arm.feedback_issue
    fragment.timestamp = time.time()
    sample = arm.read()
    assert sample is not None
    assert arm.feedback_issue == ""
    assert np.isfinite(sample[1]["vector"]).all()


def test_stale_status_does_not_report_default_fault_but_fresh_fault_does():
    arm, messages = reader()
    messages["status"].timestamp = 0
    messages["status"].msg.arm_status.value = 1
    assert arm.read() is None
    messages["status"].timestamp = time.time()
    with pytest.raises(RuntimeError, match="Robot error"):
        arm.read()


def test_timestamp_conversion_preserves_original_feedback_time():
    arm, messages = reader()
    messages["grip"].timestamp = time.time() - .1
    stamp = arm.read()[1]["source_timestamps"]["gripper"]
    assert .09 < time.perf_counter() - stamp < .2
