"""Audio work cannot delay robot control; fault feedback takes priority."""
import time
from threading import Event

from piperx_teleop.feedback import Feedback, TONES


def test_slow_audio_is_nonblocking_and_fault_replaces_pending_sounds():
    entered, release, fault_heard = Event(), Event(), Event()
    played = []
    def slow_audio(frequency, duration):
        played.append((frequency, duration))
        if len(played) == 1:
            entered.set()
            release.wait(2)
        if frequency == TONES["fault"][0][0]:
            fault_heard.set()
    feedback = Feedback({"sound": True}, play=slow_audio)
    feedback.start()
    try:
        feedback.emit("record_started", "Recording")
        assert entered.wait(1)
        start = time.perf_counter()
        feedback.emit("saved", "Saved")
        feedback.emit("fault", "Disconnected")
        assert time.perf_counter()-start < .1
        assert [n["id"] for n in feedback.snapshot()] == [1, 2, 3]
        release.set()
        assert fault_heard.wait(1)
        assert played[1][0] == TONES["fault"][0][0]
    finally:
        release.set()
        feedback.close()


def test_muting_sound_keeps_numbered_confirmations():
    feedback = Feedback({"sound": False, "haptics": False})
    feedback.start()
    feedback.emit("saved", "Saved")
    assert feedback.thread is None and not feedback.pending
    assert feedback.snapshot() == [{"id": 1, "kind": "saved", "message": "Saved"}]
    feedback.close()
