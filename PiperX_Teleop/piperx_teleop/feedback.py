"""Nonblocking PC sounds and numbered confirmations for Quest haptics."""
from collections import deque
import logging
from threading import Condition, Thread

LOG = logging.getLogger(__name__)
TONES = {
    "record_started": [(1100, 100)],
    "saving": [(700, 100)],
    "saved": [(1300, 100), (1600, 100)],
    "rejected": [(450, 120)] * 2,
    "fault": [(350, 250)] * 3,
    "paused": [(500, 120)] * 2,
    "home_started": [(800, 180)],
    "home_done": [(1000, 100), (1300, 100)],
    "home_cancelled": [(600, 180)],
    "home_saved": [(1300, 100)] * 2,
}


class Feedback:
    def __init__(self, cfg=None, play=None):
        cfg = cfg or {}
        self.sound = cfg.get("sound", True)
        self.haptics = cfg.get("haptics", True)
        self.play = play or self._beep
        self.condition = Condition()
        self.notices = deque(maxlen=32)
        self.pending = deque(maxlen=16)
        self.sequence = 0
        self.stopped = False
        self.thread = None

    @staticmethod
    def _beep(frequency, duration):
        import winsound
        winsound.Beep(frequency, duration)

    def emit(self, kind, message):
        with self.condition:
            self.sequence += 1
            self.notices.append({"id": self.sequence, "kind": kind, "message": message})
            if self.sound and not self.stopped:
                if kind == "fault":
                    self.pending.clear()
                self.pending.append(kind)
                self.condition.notify()

    def snapshot(self):
        with self.condition:
            return list(self.notices)

    def start(self):
        if self.sound and self.thread is None:
            self.thread = Thread(target=self._run, daemon=True, name="operator-sound")
            self.thread.start()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopped or self.pending)
                if self.stopped:
                    return
                kind = self.pending.popleft()
            try:
                for frequency, duration in TONES[kind]:
                    self.play(frequency, duration)
                    with self.condition:
                        if self.stopped or (kind != "fault" and "fault" in self.pending):
                            break
                        self.condition.wait(timeout=.08)
            except Exception:
                LOG.warning("PC sound unavailable; check the local status window", exc_info=True)
                return

    def close(self):
        with self.condition:
            self.stopped = True
            self.pending.clear()
            self.condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=2)
