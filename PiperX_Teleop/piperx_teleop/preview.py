"""Local three-view RGB preview. Call from the application's main thread."""
import cv2
import numpy as np

from .core import CAMERAS


class Preview:
    title = "PiperX RGB | Head D555 | Left D405 | Right D405"

    def __init__(self, suffix=""):
        self.title += suffix
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.title, 1440, 500)

    def update(self, cameras, status):
        tiles = []
        for name in CAMERAS:
            sample = cameras.histories[name].latest()
            tile = np.zeros((320, 480, 3), np.uint8)
            if sample:
                image = sample.value["bgr"]
                scale = min(480 / image.shape[1], 280 / image.shape[0])
                image = cv2.resize(image, None, fx=scale, fy=scale)
                y, x = 40 + (280 - image.shape[0]) // 2, (480 - image.shape[1]) // 2
                tile[y:y + image.shape[0], x:x + image.shape[1]] = image
            fps = status["fps"]["cameras"][name]
            cv2.putText(tile, f"{name}  {fps:.1f} FPS", (12, 27), cv2.FONT_HERSHEY_SIMPLEX,
                        .65, (240, 240, 240), 2)
            tiles.append(tile)
        canvas = np.vstack([np.hstack(tiles), np.zeros((160, 1440, 3), np.uint8)])
        rates = status["fps"]
        recording = "SAVING" if status["saving"] else ("REC" if status["recording"] else "OFF")
        text = (f"{status['mode']} | {recording} {rates['recording']:.1f} FPS"
                f" | action L/R: {rates['action']['left']:.1f}/{rates['action']['right']:.1f} Hz"
                f" | frames: {status['frames']} | dropped: {status['dropped']}")
        color = (60, 60, 255) if status["mode"] == "fault" else (100, 240, 120)
        if status["mode"] == "paused":
            color = (60, 200, 255)
        cv2.putText(canvas, text, (12, 350), cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
        cv2.putText(canvas, status["message"][:150], (12, 385), cv2.FONT_HERSHEY_SIMPLEX, .5, (240, 240, 240), 1)
        quest = status["quest"]
        inputs = f"Quest {quest['hz']:.1f} Hz | age {quest['age_ms']} ms | active {int(quest['active'])}"
        inputs += f" | X {int(quest.get('buttons', {}).get('set_home', False))}"
        for side, label in (("left", "L"), ("right", "R")):
            hand = quest[side]
            inputs += (f" | {label}: tracked {int(hand['tracked'])} middle {int(hand['middle'])}"
                       f" trigger {hand['trigger']:.2f}")
        cv2.putText(canvas, inputs, (12, 425), cv2.FONT_HERSHEY_SIMPLEX, .52, (240, 240, 240), 1)
        grips = "Gripper actual/target mm"
        for side, label in (("left", "L"), ("right", "R")):
            grip = status["grippers"][side]
            grips += f" | {label}: {grip['actual_mm']}/{grip['target_mm']} enabled={grip['enabled']}"
        if status["mode"] == "fault":
            grips += " | FAULT: commands stopped; last feedback"
        cv2.putText(canvas, grips, (12, 460), cv2.FONT_HERSHEY_SIMPLEX, .5, (240, 240, 240), 1)
        cv2.imshow(self.title, canvas)
        key = cv2.waitKey(1) & 0xff
        return key not in (27, ord("q")) and cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) >= 1

    def close(self):
        cv2.destroyAllWindows()

