"""
camera.py — RPi Camera Module capture via picamera2.

Falls back to OpenCV VideoCapture if picamera2 is unavailable
(useful for testing on a laptop with a USB webcam).
"""

import logging
import numpy as np

logger = logging.getLogger(__name__)

# Try picamera2 first (RPi), fall back to OpenCV (desktop)
_USE_PICAMERA2 = False
try:
    from picamera2 import Picamera2

    _USE_PICAMERA2 = True
except ImportError:
    logger.info("picamera2 not available — falling back to OpenCV VideoCapture")


class Camera:
    """
    Thin abstraction over picamera2 / OpenCV.

    Usage::

        cam = Camera(config["camera"])
        cam.start()
        frame = cam.grab_frame()   # np.ndarray  BGR (H, W, 3)
        cam.stop()
    """

    def __init__(self, cam_cfg: dict):
        self.width = cam_cfg.get("width", 640)
        self.height = cam_cfg.get("height", 480)
        self.fmt = cam_cfg.get("format", "RGB888")
        self._cam = None
        self._started = False

    def start(self):
        if self._started:
            return

        if _USE_PICAMERA2:
            self._cam = Picamera2()
            cfg = self._cam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": self.fmt}
            )
            self._cam.configure(cfg)
            self._cam.start()
            logger.info(
                f"picamera2 started at {self.width}×{self.height} ({self.fmt})"
            )
        else:
            import cv2

            self._cam = cv2.VideoCapture(0)
            self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self._cam.isOpened():
                raise RuntimeError("Cannot open camera via OpenCV")
            logger.info(
                f"OpenCV camera started at {self.width}×{self.height}"
            )

        self._started = True

    def grab_frame(self) -> np.ndarray:
        """Return a single frame as a BGR numpy array (H, W, 3)."""
        if not self._started:
            self.start()

        if _USE_PICAMERA2:
            import cv2

            frame = self._cam.capture_array()
            # picamera2 returns RGB; convert to BGR for OpenCV compatibility
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        else:
            import cv2

            ret, frame = self._cam.read()
            if not ret:
                raise RuntimeError("Failed to grab frame from OpenCV camera")
            return frame

    def stop(self):
        if not self._started:
            return
        if _USE_PICAMERA2:
            self._cam.stop()
        else:
            self._cam.release()
        self._started = False
        logger.info("Camera stopped")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import yaml, pathlib, cv2

    logging.basicConfig(level=logging.INFO)
    cfg = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent / "config.yaml").read_text()
    )

    with Camera(cfg["camera"]) as cam:
        frame = cam.grab_frame()
        print(f"Captured frame: {frame.shape}")
        cv2.imwrite("test_frame.jpg", frame)
        print("Saved test_frame.jpg")
