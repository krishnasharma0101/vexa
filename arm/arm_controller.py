"""
arm_controller.py — High-level robotic arm API.

Translates world-coordinate commands into joint angles (via IK)
and dispatches them to the servo controller.
"""

import time
import logging
from typing import Optional, Tuple, Dict

from arm.servo_controller import ServoController
from arm.kinematics import ArmKinematics

logger = logging.getLogger(__name__)


class ArmController:
    """
    High-level interface for the robotic arm.

    Usage::

        arm = ArmController(config)
        arm.go_home()
        arm.move_to(150, 0, 50)
        arm.close_gripper()
    """

    def __init__(self, config: dict, dry_run: bool = False):
        self.dry_run = dry_run
        self.config = config

        self.kinematics = ArmKinematics(config["arm"])

        if not dry_run:
            self.servos = ServoController(config["servos"])
        else:
            self.servos = None
            logger.info("ArmController in DRY-RUN mode (no servo motion)")

        # Gripper angles from config
        grip_cfg = config["servos"]["gripper"]
        self._grip_open = grip_cfg.get("open_angle", 150)
        self._grip_close = grip_cfg.get("close_angle", 30)

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------
    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        pitch_deg: float = -45.0,
    ) -> bool:
        """
        Move end-effector to world position (x, y, z) mm.

        Returns True if the position was reachable and motion was executed.
        """
        angles = self.kinematics.inverse(x, y, z, pitch_deg)
        if angles is None:
            logger.error(f"Cannot reach ({x:.1f}, {y:.1f}, {z:.1f})")
            return False

        logger.info(
            f"Moving to ({x:.1f}, {y:.1f}, {z:.1f}) → "
            f"base={angles['base']:.1f}° sh={angles['shoulder']:.1f}° "
            f"el={angles['elbow']:.1f}° wp={angles['wrist_pitch']:.1f}°"
        )

        if self.dry_run:
            return True

        # Move base first, then the arm chain
        self.servos.set_angle("base", angles["base"])
        self.servos.set_angle("shoulder", angles["shoulder"])
        self.servos.set_angle("elbow", angles["elbow"])
        self.servos.set_angle("wrist_pitch", angles["wrist_pitch"])
        return True

    def move_joints(self, angles: Dict[str, float]):
        """Directly set joint angles (bypasses IK)."""
        if self.dry_run:
            logger.info(f"[DRY RUN] set joints: {angles}")
            return
        self.servos.set_all(angles)

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------
    def open_gripper(self):
        logger.info("Opening gripper")
        if not self.dry_run:
            self.servos.set_angle("gripper", self._grip_open)

    def close_gripper(self):
        logger.info("Closing gripper")
        if not self.dry_run:
            self.servos.set_angle("gripper", self._grip_close)

    # ------------------------------------------------------------------
    # Presets
    # ------------------------------------------------------------------
    def go_home(self):
        """Move all joints to their configured home angles."""
        logger.info("Going home")
        if not self.dry_run:
            self.servos.go_home()

    def get_position(self) -> Tuple[float, float, float]:
        """Return current end-effector position based on joint angles."""
        if self.dry_run:
            return (0.0, 0.0, 0.0)
        angles = self.servos.get_all_angles()
        return self.kinematics.forward(angles)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def shutdown(self):
        logger.info("Shutting down arm controller")
        if self.servos:
            self.servos.go_home()
            time.sleep(0.5)
            self.servos.relax_all()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
