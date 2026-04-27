"""
servo_controller.py — GPIO-based servo driver for RPi 5.

Drives 6 servos via software PWM on GPIO pins using gpiozero.
Falls back to pigpio backend automatically if pigpiod is running
(gives DMA-timed PWM for less jitter).
"""

import time
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Try pigpio backend first (better PWM), fall back to default (lgpio)
# ---------------------------------------------------------------------------
try:
    from gpiozero.pins.pigpio import PiGPIOFactory
    from gpiozero import Device

    Device.pin_factory = PiGPIOFactory()
    _BACKEND = "pigpio"
    logger.info("Using pigpio backend (DMA-timed PWM — best quality)")
except Exception:
    _BACKEND = "lgpio"
    logger.info("Using default lgpio backend (software PWM)")

from gpiozero import AngularServo


class ServoJoint:
    """Wrapper around a single AngularServo with smooth motion."""

    def __init__(self, name: str, cfg: dict):
        self.name = name
        self.min_angle: float = cfg["min_angle"]
        self.max_angle: float = cfg["max_angle"]
        self.home_angle: float = cfg["home_angle"]
        self.speed: float = cfg.get("speed", 60)  # deg/s
        self._current_angle: float = self.home_angle

        min_pw = cfg["min_pulse_ms"] / 1000.0  # gpiozero wants seconds
        max_pw = cfg["max_pulse_ms"] / 1000.0

        self.servo = AngularServo(
            cfg["gpio_pin"],
            min_angle=self.min_angle,
            max_angle=self.max_angle,
            min_pulse_width=min_pw,
            max_pulse_width=max_pw,
            initial_angle=self.home_angle,
        )
        logger.info(
            f"  [{name}] GPIO {cfg['gpio_pin']}  "
            f"range {self.min_angle}°–{self.max_angle}°  "
            f"home {self.home_angle}°  backend={_BACKEND}"
        )

    # --- public API ---------------------------------------------------------

    @property
    def angle(self) -> float:
        return self._current_angle

    def set_angle(self, target: float, smooth: bool = True):
        """Move to *target* degrees, optionally with interpolation."""
        target = self._clamp(target)
        if smooth:
            self._smooth_move(target)
        else:
            self.servo.angle = target
            self._current_angle = target

    def relax(self):
        """Detach the servo (stop sending PWM)."""
        self.servo.detach()
        logger.debug(f"  [{self.name}] relaxed")

    # --- internal -----------------------------------------------------------

    def _clamp(self, angle: float) -> float:
        return max(self.min_angle, min(self.max_angle, angle))

    def _smooth_move(self, target: float, step_deg: float = 1.0):
        """Move in small increments for smooth motion."""
        current = self._current_angle
        if abs(target - current) < step_deg:
            self.servo.angle = target
            self._current_angle = target
            return

        direction = 1.0 if target > current else -1.0
        delay = step_deg / self.speed  # seconds per step

        pos = current
        while True:
            pos += direction * step_deg
            if (direction > 0 and pos >= target) or (direction < 0 and pos <= target):
                pos = target
            self.servo.angle = pos
            self._current_angle = pos
            if pos == target:
                break
            time.sleep(delay)


class ServoController:
    """
    Manages all servo joints.

    Usage::

        ctrl = ServoController(config["servos"])
        ctrl.set_angle("shoulder", 45)
        ctrl.go_home()
        ctrl.relax_all()
    """

    JOINT_NAMES = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]

    def __init__(self, servos_cfg: dict):
        logger.info("Initialising servo controller …")
        self.joints: Dict[str, ServoJoint] = {}
        for name in self.JOINT_NAMES:
            if name in servos_cfg:
                self.joints[name] = ServoJoint(name, servos_cfg[name])
        logger.info(f"Servo controller ready — {len(self.joints)} joints")

    # --- single joint -------------------------------------------------------

    def set_angle(self, joint: str, angle: float, smooth: bool = True):
        self.joints[joint].set_angle(angle, smooth)

    def get_angle(self, joint: str) -> float:
        return self.joints[joint].angle

    # --- bulk operations ----------------------------------------------------

    def set_all(self, angles: Dict[str, float], smooth: bool = True):
        """Set multiple joints. Missing keys are left unchanged."""
        for name, angle in angles.items():
            if name in self.joints:
                self.joints[name].set_angle(angle, smooth)

    def go_home(self):
        """Move every joint to its configured home angle."""
        logger.info("Going home …")
        for joint in self.joints.values():
            joint.set_angle(joint.home_angle)

    def relax_all(self):
        """Detach all servos (stop PWM)."""
        for joint in self.joints.values():
            joint.relax()
        logger.info("All servos relaxed")

    # --- gripper shortcuts --------------------------------------------------

    def open_gripper(self):
        cfg = self.joints["gripper"]
        # open_angle stored in the YAML; fall back to max_angle
        target = getattr(cfg, "_open_angle", cfg.max_angle)
        cfg.set_angle(target)

    def close_gripper(self):
        cfg = self.joints["gripper"]
        target = getattr(cfg, "_close_angle", cfg.min_angle)
        cfg.set_angle(target)

    def get_all_angles(self) -> Dict[str, float]:
        return {name: j.angle for name, j in self.joints.items()}

    # --- context manager ----------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.relax_all()


# ---------------------------------------------------------------------------
# Quick self-test (run on the RPi)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import yaml, pathlib

    logging.basicConfig(level=logging.INFO)
    cfg = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent / "config.yaml").read_text()
    )

    with ServoController(cfg["servos"]) as ctrl:
        print("Moving all joints to home …")
        ctrl.go_home()
        time.sleep(1)

        print("Sweeping base 45° → 135° → 90° …")
        ctrl.set_angle("base", 45)
        time.sleep(0.5)
        ctrl.set_angle("base", 135)
        time.sleep(0.5)
        ctrl.set_angle("base", 90)
        time.sleep(0.5)

        print("Opening gripper …")
        ctrl.open_gripper()
        time.sleep(0.5)
        print("Closing gripper …")
        ctrl.close_gripper()
        time.sleep(0.5)

        print("Done — relaxing all servos.")
