"""
kinematics.py — Forward and Inverse Kinematics for a 4-DOF arm.

Joint layout (side view in the vertical plane):
    ┌── shoulder ── upper_arm ── elbow ── forearm ── wrist ── hand ── tip
    │
  base_height (fixed vertical offset from table)

The base rotation is solved separately (atan2 in the XY plane).
Shoulder + elbow + wrist_pitch are solved via 2-D geometric IK
in the vertical plane containing the target point.
"""

import math
import logging
from typing import Optional, Dict, Tuple

logger = logging.getLogger(__name__)


class ArmKinematics:
    """
    Analytical IK/FK for a planar 3-link arm with a rotating base.

    All lengths in mm.  Angles in degrees.
    World frame: X = forward, Y = left, Z = up.  Origin at base centre on the table.
    """

    def __init__(self, arm_cfg: dict):
        self.base_height = arm_cfg["base_height"]  # mm
        self.L1 = arm_cfg["upper_arm"]              # shoulder → elbow
        self.L2 = arm_cfg["forearm"]                # elbow → wrist
        self.L3 = arm_cfg["hand"]                   # wrist → gripper tip
        logger.info(
            f"Kinematics: base_h={self.base_height}  "
            f"L1={self.L1}  L2={self.L2}  L3={self.L3} mm"
        )

    # ------------------------------------------------------------------
    # Forward Kinematics
    # ------------------------------------------------------------------
    def forward(self, angles: Dict[str, float]) -> Tuple[float, float, float]:
        """
        Given joint angles (degrees), return end-effector (x, y, z) in mm.

        Required keys: base, shoulder, elbow, wrist_pitch.
        """
        base_rad = math.radians(angles["base"])
        sh_rad = math.radians(angles["shoulder"])
        el_rad = math.radians(angles["elbow"])
        wp_rad = math.radians(angles["wrist_pitch"])

        # In the vertical plane (r = horizontal reach, z = height)
        # shoulder angle: 0° = arm pointing straight up, 90° = horizontal forward
        # We remap so 90° config-angle = horizontal
        theta1 = math.pi / 2 - sh_rad   # angle from horizontal
        theta2 = el_rad - math.pi / 2   # elbow relative to upper arm
        theta3 = wp_rad - math.pi / 2   # wrist relative to forearm

        # Cumulative angles from horizontal
        a1 = theta1
        a2 = a1 + theta2
        a3 = a2 + theta3

        r = (self.L1 * math.cos(a1)
             + self.L2 * math.cos(a2)
             + self.L3 * math.cos(a3))

        z = (self.base_height
             + self.L1 * math.sin(a1)
             + self.L2 * math.sin(a2)
             + self.L3 * math.sin(a3))

        x = r * math.cos(base_rad)
        y = r * math.sin(base_rad)

        return (x, y, z)

    # ------------------------------------------------------------------
    # Inverse Kinematics
    # ------------------------------------------------------------------
    def inverse(
        self,
        x: float,
        y: float,
        z: float,
        pitch_deg: float = 0.0,
    ) -> Optional[Dict[str, float]]:
        """
        Compute joint angles to reach (x, y, z) with the given wrist pitch.

        pitch_deg: desired angle of the gripper from horizontal
                   (0 = pointing forward, -90 = pointing straight down).

        Returns dict with keys base, shoulder, elbow, wrist_pitch (degrees)
        or None if the target is unreachable.
        """
        # --- Base angle (top-down view) ---
        base_deg = math.degrees(math.atan2(y, x))

        # --- Work in the vertical plane ---
        r = math.hypot(x, y)                # horizontal distance
        z_rel = z - self.base_height        # height relative to shoulder

        pitch_rad = math.radians(pitch_deg)

        # Wrist position (subtract the hand/gripper link)
        r_w = r - self.L3 * math.cos(pitch_rad)
        z_w = z_rel - self.L3 * math.sin(pitch_rad)

        # 2-link IK for shoulder-elbow to reach (r_w, z_w)
        D_sq = r_w ** 2 + z_w ** 2
        D = math.sqrt(D_sq)

        # Check reachability
        if D > (self.L1 + self.L2) or D < abs(self.L1 - self.L2):
            logger.warning(
                f"Target ({x:.1f}, {y:.1f}, {z:.1f}) unreachable "
                f"(D={D:.1f}, range={abs(self.L1-self.L2):.1f}–{self.L1+self.L2:.1f})"
            )
            return None

        # Elbow angle (cosine rule) — "elbow-down" solution
        cos_beta = (D_sq - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)          # 0..π
        elbow_angle = math.pi - beta        # internal elbow flex

        # Shoulder angle
        alpha = math.atan2(z_w, r_w)
        gamma = math.atan2(
            self.L2 * math.sin(beta),
            self.L1 + self.L2 * math.cos(beta),
        )
        shoulder_angle = alpha + gamma      # from horizontal

        # Wrist pitch to achieve desired end-effector pitch
        wrist_angle = pitch_rad - shoulder_angle + elbow_angle

        # Convert to servo-space (0-180 mapping used in config)
        shoulder_deg = 90 - math.degrees(shoulder_angle)
        elbow_deg = 90 + math.degrees(elbow_angle)
        wrist_deg = 90 + math.degrees(wrist_angle)

        result = {
            "base": base_deg,
            "shoulder": shoulder_deg,
            "elbow": elbow_deg,
            "wrist_pitch": wrist_deg,
        }

        # Validate all angles within 0-180 range
        for name, val in result.items():
            if val < 0 or val > 180:
                logger.warning(
                    f"Joint '{name}' angle {val:.1f}° out of servo range [0, 180]"
                )
                return None

        logger.debug(f"IK solution: {result}")
        return result


# ---------------------------------------------------------------------------
# Quick self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import yaml, pathlib

    logging.basicConfig(level=logging.DEBUG)
    cfg = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent / "config.yaml").read_text()
    )
    kin = ArmKinematics(cfg["arm"])

    # Test: home position FK
    home = {"base": 90, "shoulder": 90, "elbow": 90, "wrist_pitch": 90}
    pos = kin.forward(home)
    print(f"Home FK → x={pos[0]:.1f}  y={pos[1]:.1f}  z={pos[2]:.1f} mm")

    # Test: IK round-trip
    target = (150, 0, 50)
    angles = kin.inverse(*target, pitch_deg=-45)
    if angles:
        print(f"IK for {target} → {angles}")
        check = kin.forward(angles)
        print(f"FK verify → x={check[0]:.1f}  y={check[1]:.1f}  z={check[2]:.1f}")
    else:
        print(f"Target {target} is unreachable")
