"""
estimator.py — Pixel-to-world coordinate mapping.

Uses a homography (perspective transform) calibrated from known
reference points to convert a detected object's pixel position
into real-world (x, y, z) coordinates relative to the arm base.
"""

import logging
from typing import Optional, Tuple, List

import numpy as np

logger = logging.getLogger(__name__)


class PositionEstimator:
    """
    Maps 2-D pixel coordinates to 3-D world coordinates.

    Assumptions:
      • Objects rest on a flat table at a known height (table_height_mm).
      • The camera is fixed relative to the arm base.
      • A homography H maps pixel (u, v) → world (X, Y) on the table plane.

    Usage::

        est = PositionEstimator(config["calibration"])
        world_xyz = est.pixel_to_world(cx_px, cy_px)
    """

    def __init__(self, cal_cfg: dict):
        self.table_z = cal_cfg.get("table_height_mm", 0.0)
        self._H: Optional[np.ndarray] = None

        raw_points = cal_cfg.get("points", [])
        if raw_points and len(raw_points) >= 4:
            self._compute_homography(raw_points)
        else:
            logger.warning(
                "No calibration points (need ≥ 4). "
                "Run `python calibrate.py --camera` to calibrate."
            )

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------
    def _compute_homography(self, points: list):
        """
        Compute homography from a list of
        [pixel_x, pixel_y, world_x_mm, world_y_mm] entries.
        """
        import cv2

        pts = np.array(points, dtype=np.float32)
        src = pts[:, :2]   # pixel coords
        dst = pts[:, 2:]   # world coords

        self._H, status = cv2.findHomography(src, dst)
        if self._H is not None:
            logger.info(f"Homography computed from {len(points)} points")
        else:
            logger.error("Homography computation failed")

    def calibrate(self, points: List[List[float]]):
        """Re-compute homography from new calibration points."""
        self._compute_homography(points)

    @property
    def is_calibrated(self) -> bool:
        return self._H is not None

    # ------------------------------------------------------------------
    # Conversion
    # ------------------------------------------------------------------
    def pixel_to_world(
        self, px: float, py: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        Convert pixel (px, py) to world (x, y, z) in mm.

        Returns None if not calibrated.
        """
        if self._H is None:
            logger.error("Cannot convert — homography not calibrated")
            return None

        pt = np.array([[[px, py]]], dtype=np.float32)
        import cv2

        dst = cv2.perspectiveTransform(pt, self._H)
        wx, wy = dst[0, 0]
        return (float(wx), float(wy), self.table_z)

    # ------------------------------------------------------------------
    # Interactive calibration helper
    # ------------------------------------------------------------------
    @staticmethod
    def run_interactive_calibration(camera, num_points: int = 6) -> list:
        """
        Capture calibration frames interactively.

        Returns list of [px, py, wx, wy] entries that can be saved to config.
        """
        import cv2

        print(
            f"\n=== Camera Calibration ===\n"
            f"Place a marker at {num_points} known positions on the table.\n"
            f"For each position:\n"
            f"  1. Place marker, press SPACE to capture\n"
            f"  2. Click the marker centre in the image\n"
            f"  3. Enter the world X, Y coordinates (mm from arm base)\n"
        )

        points = []

        for i in range(num_points):
            print(f"\n--- Point {i + 1}/{num_points} ---")
            input("Place marker and press ENTER when ready...")

            frame = camera.grab_frame()
            clone = frame.copy()
            click_pos = []

            def on_click(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    click_pos.clear()
                    click_pos.extend([x, y])
                    cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
                    cv2.imshow("Calibration", clone)

            cv2.imshow("Calibration", frame)
            cv2.setMouseCallback("Calibration", on_click)

            print("Click the marker centre in the image, then press any key.")
            cv2.waitKey(0)

            if not click_pos:
                print("No click detected — skipping this point.")
                continue

            px, py = click_pos
            wx = float(input(f"  World X (mm) for pixel ({px}, {py}): "))
            wy = float(input(f"  World Y (mm) for pixel ({px}, {py}): "))
            points.append([px, py, wx, wy])
            print(f"  Recorded: pixel=({px}, {py}) → world=({wx}, {wy})")

        cv2.destroyAllWindows()
        return points


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("Run `python calibrate.py --camera` for interactive calibration.")
