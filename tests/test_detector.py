"""
test_detector.py — Unit tests for the object detector.

Run: python -m pytest tests/test_detector.py -v

Requires the ultralytics package and will auto-download yolov8n.pt
on first run.
"""

import numpy as np
import pytest
from vision.detector import ObjectDetector

DET_CONFIG = {
    "model": "yolov8n.pt",
    "confidence": 0.25,  # lower threshold for test
    "target_classes": [],
}


@pytest.fixture(scope="module")
def detector():
    """Load detector once for all tests (model download may take a moment)."""
    return ObjectDetector(DET_CONFIG)


class TestObjectDetector:
    def test_empty_image(self, detector):
        """A blank black image should return no (or very few) detections."""
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        dets = detector.detect(blank)
        assert isinstance(dets, list)
        # There may be false positives but shouldn't crash

    def test_random_image(self, detector):
        """Random noise image should not crash."""
        noise = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        dets = detector.detect(noise)
        assert isinstance(dets, list)

    def test_detection_fields(self, detector):
        """If any detections found, verify they have the right structure."""
        # Create a simple test image (solid color — unlikely to detect much)
        img = np.full((480, 640, 3), 128, dtype=np.uint8)
        dets = detector.detect(img)
        for d in dets:
            assert hasattr(d, "bbox")
            assert hasattr(d, "centre")
            assert hasattr(d, "label")
            assert hasattr(d, "confidence")
            assert hasattr(d, "area")
            assert len(d.bbox) == 4
            assert len(d.centre) == 2
            assert 0 <= d.confidence <= 1

    def test_detect_and_draw(self, detector):
        """detect_and_draw should return an annotated frame + detections."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        annotated, dets = detector.detect_and_draw(img)
        assert annotated.shape == img.shape
        assert isinstance(dets, list)
