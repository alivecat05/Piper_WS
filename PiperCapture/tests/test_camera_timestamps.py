import pytest

from piper_capture.cameras import frame_timestamp


@pytest.mark.parametrize("domain", ["global_time", "system_time"])
def test_sdk_host_clock_uses_frame_time_not_python_receive(domain):
    stamp, basis = frame_timestamp(1_700_000_000_000, f"timestamp_domain.{domain}",
                                  receive=100.07, wall_offset=100-1_700_000_000)
    assert stamp == 100
    assert basis == domain


def test_unmapped_hardware_clock_is_not_treated_as_unix():
    stamp, basis = frame_timestamp(1234, "timestamp_domain.hardware_clock", 100,
                                  wall_offset=100-1_700_000_000, latency_ms=10)
    assert stamp == pytest.approx(99.99)
    assert basis == "host_receive"


def test_invalid_host_clock_mapping_is_rejected():
    with pytest.raises(RuntimeError, match="timestamp mapping"):
        frame_timestamp(1234, "system_time", 100, wall_offset=100-1_700_000_000)
