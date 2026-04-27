"""
detector.py — YOLOv8-based object detection.

Uses Ultralytics YOLOv8n (nano) for fast inference on RPi 5.
The model file is auto-downloaded on first run.
"""

import logging
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Single detected object."""

    bbox: tuple           # (x1, y1, x2, y2) pixel coords
    centre: tuple         # (cx, cy) pixel coords
    label: str
    confidence: float
    area: float           # bbox area in pixels²

    @property
    def width(self):
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self):
        return self.bbox[3] - self.bbox[1]


class ObjectDetector:
    """
    YOLOv8 object detector.

    Usage::

        det = ObjectDetector(config["detection"])
        detections = det.detect(frame)
        for d in detections:
            print(d.label, d.confidence, d.centre)
    """

    def __init__(self, det_cfg: dict):
        from ultralytics import YOLO

        model_path = det_cfg.get("model", "yolov8n.pt")
        self.conf_threshold = det_cfg.get("confidence", 0.45)
        self.target_classes: List[str] = det_cfg.get("target_classes", [])

        logger.info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        logger.info("YOLO model loaded")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run detection on a BGR frame.

        Returns list of Detection objects sorted by area (largest first).
        """
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        detections: List[Detection] = []

        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                # Filter by target classes if specified
                if self.target_classes and label not in self.target_classes:
                    continue

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                area = (x2 - x1) * (y2 - y1)

                detections.append(
                    Detection(
                        bbox=(x1, y1, x2, y2),
                        centre=(cx, cy),
                        label=label,
                        confidence=conf,
                        area=area,
                    )
                )

        # Largest first (usually the closest / most graspable)
        detections.sort(key=lambda d: d.area, reverse=True)
        logger.debug(f"Detected {len(detections)} objects")
        return detections

    def detect_and_draw(self, frame: np.ndarray) -> tuple:
        """
        Detect objects and draw bounding boxes on a copy of the frame.

        Returns (annotated_frame, detections).
        """
        import cv2

        detections = self.detect(frame)
        annotated = frame.copy()

        for d in detections:
            x1, y1, x2, y2 = [int(v) for v in d.bbox]
            color = (0, 255, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            text = f"{d.label} {d.confidence:.0%}"
            cv2.putText(
                annotated, text, (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            )
            # Centre dot
            cv2.circle(annotated, (int(d.centre[0]), int(d.centre[1])), 4, (0, 0, 255), -1)

        return annotated, detections


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import cv2, yaml, pathlib

    logging.basicConfig(level=logging.INFO)
    cfg = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent / "config.yaml").read_text()
    )

    from vision.camera import Camera

    det = ObjectDetector(cfg["detection"])
    with Camera(cfg["camera"]) as cam:
        frame = cam.grab_frame()
        annotated, dets = det.detect_and_draw(frame)
        print(f"Found {len(dets)} objects")
        for d in dets:
            print(f"  {d.label} ({d.confidence:.0%}) at {d.centre}")
        cv2.imwrite("detection_result.jpg", annotated)
        print("Saved detection_result.jpg")
