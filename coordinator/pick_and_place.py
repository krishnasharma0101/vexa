"""
pick_and_place.py — State-machine coordinator for pick-and-place.

States:
    IDLE → SCAN → DETECT → APPROACH → DESCEND → GRIP → LIFT
         → MOVE_TO_DROP → RELEASE → HOME → (back to SCAN or IDLE)
"""

import time
import enum
import logging
from typing import Optional

from arm.arm_controller import ArmController
from vision.camera import Camera
from vision.detector import ObjectDetector, Detection
from vision.estimator import PositionEstimator

logger = logging.getLogger(__name__)


class State(enum.Enum):
    IDLE = "IDLE"
    SCAN = "SCAN"
    DETECT = "DETECT"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    GRIP = "GRIP"
    LIFT = "LIFT"
    MOVE_TO_DROP = "MOVE_TO_DROP"
    RELEASE = "RELEASE"
    HOME = "HOME"
    ERROR = "ERROR"
    DONE = "DONE"


class PickAndPlaceCoordinator:
    """
    Orchestrates the full pick-and-place cycle.

    Usage::

        coord = PickAndPlaceCoordinator(config, arm, camera, detector, estimator)
        coord.run_once()      # single pick-and-place cycle
        coord.run_loop()      # continuous loop
    """

    def __init__(
        self,
        config: dict,
        arm: ArmController,
        camera: Camera,
        detector: ObjectDetector,
        estimator: PositionEstimator,
    ):
        self.cfg = config.get("pick_and_place", {})
        self.arm = arm
        self.camera = camera
        self.detector = detector
        self.estimator = estimator

        self.state = State.IDLE
        self._target: Optional[Detection] = None
        self._target_world = None

        # Configurable heights/positions
        self.approach_h = self.cfg.get("approach_height_mm", 80)
        self.grip_h = self.cfg.get("grip_height_mm", 10)
        self.lift_h = self.cfg.get("lift_height_mm", 120)
        self.grip_delay = self.cfg.get("grip_delay_s", 0.5)
        self.release_delay = self.cfg.get("release_delay_s", 0.3)

        drop = self.cfg.get("drop_position", {"x": -150, "y": 0, "z": 80})
        self.drop_pos = (drop["x"], drop["y"], drop["z"])

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _transition(self, new_state: State):
        logger.info(f"  [{self.state.value}] → [{new_state.value}]")
        self.state = new_state

    def step(self) -> State:
        """Execute one step of the state machine. Returns the new state."""

        if self.state in (State.IDLE, State.SCAN):
            # No base motor to scan, so just detect directly ahead
            self._transition(State.DETECT)

        elif self.state == State.DETECT:
            self._do_detect()

        elif self.state == State.APPROACH:
            self._do_approach()

        elif self.state == State.DESCEND:
            self._do_descend()

        elif self.state == State.GRIP:
            self._do_grip()

        elif self.state == State.LIFT:
            self._do_lift()

        elif self.state == State.MOVE_TO_DROP:
            self._do_move_to_drop()

        elif self.state == State.RELEASE:
            self._do_release()

        elif self.state == State.HOME:
            self._do_home()

        return self.state

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------
    def _do_scan(self):
        """No longer used, jump directly to detect."""
        self._transition(State.DETECT)

    def _do_detect(self):
        """Capture a frame and look for objects."""
        frame = self.camera.grab_frame()
        detections = self.detector.detect(frame)

        if not detections:
            logger.info("No objects detected in front — returning to scan/idle")
            time.sleep(1) # delay before retrying
            self._transition(State.SCAN)
            return

        # Pick the largest / most prominent detection
        best = detections[0]
        logger.info(
            f"Detected '{best.label}' ({best.confidence:.0%}) "
            f"at pixel ({best.centre[0]:.0f}, {best.centre[1]:.0f})"
        )

        if not self.estimator.is_calibrated:
            logger.error("Camera not calibrated — cannot estimate position")
            self._transition(State.ERROR)
            return

        world = self.estimator.pixel_to_world(*best.centre)
        if world is None:
            self._transition(State.ERROR)
            return

        self._target = best
        self._target_world = world
        logger.info(
            f"Object world position: "
            f"x={world[0]:.1f}  y={world[1]:.1f}  z={world[2]:.1f} mm"
        )
        if abs(world[1]) > 50:
            logger.warning("Object is far off-center (y != 0). With no base motor, we may not reach it properly.")
        self._transition(State.APPROACH)

    def _do_approach(self):
        """Move above the object at safe height."""
        x, y, z = self._target_world
        ok = self.arm.move_to(x, y, self.approach_h, pitch_deg=-90)
        if not ok:
            self._transition(State.ERROR)
            return
        time.sleep(0.3)
        self.arm.open_gripper()
        time.sleep(0.2)
        self._transition(State.DESCEND)

    def _do_descend(self):
        """Lower to grip height."""
        x, y, _ = self._target_world
        ok = self.arm.move_to(x, y, self.grip_h, pitch_deg=-90)
        if not ok:
            self._transition(State.ERROR)
            return
        time.sleep(0.3)
        self._transition(State.GRIP)

    def _do_grip(self):
        """Close the gripper."""
        self.arm.close_gripper()
        time.sleep(self.grip_delay)
        self._transition(State.LIFT)

    def _do_lift(self):
        """Lift the object."""
        x, y, _ = self._target_world
        ok = self.arm.move_to(x, y, self.lift_h, pitch_deg=-90)
        if not ok:
            self._transition(State.ERROR)
            return
        time.sleep(0.3)
        self._transition(State.MOVE_TO_DROP)

    def _do_move_to_drop(self):
        """Move to the drop-off position."""
        ok = self.arm.move_to(*self.drop_pos, pitch_deg=-90)
        if not ok:
            self._transition(State.ERROR)
            return
        time.sleep(0.3)
        self._transition(State.RELEASE)

    def _do_release(self):
        """Open the gripper to release the object."""
        self.arm.open_gripper()
        time.sleep(self.release_delay)
        self._transition(State.HOME)

    def _do_home(self):
        """Return to home position."""
        self.arm.go_home()
        time.sleep(0.5)
        self._transition(State.DONE)

    # ------------------------------------------------------------------
    # Run modes
    # ------------------------------------------------------------------
    def run_once(self) -> bool:
        """Run a single pick-and-place cycle. Returns True on success."""
        self.state = State.IDLE

        while self.state not in (State.DONE, State.ERROR):
            self.step()

        success = self.state == State.DONE
        if success:
            logger.info("Pick-and-place cycle completed successfully")
        else:
            logger.error("Pick-and-place cycle ended in error")
        return success

    def run_loop(self, max_cycles: int = 0):
        """
        Run continuously until stopped (Ctrl-C) or max_cycles reached.

        max_cycles=0 means infinite.
        """
        cycle = 0
        try:
            while True:
                cycle += 1
                logger.info(f"\n{'='*50}\n  Cycle {cycle}\n{'='*50}")
                success = self.run_once()
                if not success:
                    logger.info("Retrying after error …")
                    self.arm.go_home()
                    time.sleep(1)
                if max_cycles and cycle >= max_cycles:
                    break
        except KeyboardInterrupt:
            logger.info("\nStopped by user")
            self.arm.go_home()
