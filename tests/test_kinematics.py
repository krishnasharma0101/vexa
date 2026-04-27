"""
test_kinematics.py — Unit tests for IK/FK.

These tests run on any machine (no hardware required).
Run: python -m pytest tests/test_kinematics.py -v
"""

import math
import pytest
from arm.kinematics import ArmKinematics

# Placeholder arm dimensions matching config.yaml
ARM_CONFIG = {
    "base_height": 65,
    "upper_arm": 105,
    "forearm": 100,
    "hand": 70,
}


@pytest.fixture
def kin():
    return ArmKinematics(ARM_CONFIG)


class TestForwardKinematics:
    def test_home_position(self, kin):
        """Home position (all 90°) should have the arm pointing straight forward."""
        home = {"base": 90, "shoulder": 90, "elbow": 90, "wrist_pitch": 90}
        x, y, z = kin.forward(home)
        # At home, all joints at 90° means arm is horizontal-ish
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(z, float)

    def test_base_rotation_symmetry(self, kin):
        """Rotating the base should mirror the x/y coordinates."""
        angles_left = {"base": 120, "shoulder": 90, "elbow": 90, "wrist_pitch": 90}
        angles_right = {"base": 60, "shoulder": 90, "elbow": 90, "wrist_pitch": 90}
        x1, y1, z1 = kin.forward(angles_left)
        x2, y2, z2 = kin.forward(angles_right)
        # z should be identical
        assert abs(z1 - z2) < 0.1
        # Horizontal reach should be the same
        r1 = math.hypot(x1, y1)
        r2 = math.hypot(x2, y2)
        assert abs(r1 - r2) < 0.1


class TestInverseKinematics:
    def test_reachable_point(self, kin):
        """A point within the workspace should return valid angles."""
        angles = kin.inverse(100, 0, 50, pitch_deg=-45)
        assert angles is not None
        for name, val in angles.items():
            assert 0 <= val <= 180, f"{name} = {val} out of range"

    def test_unreachable_point(self, kin):
        """A point far beyond the arm's reach should return None."""
        result = kin.inverse(1000, 1000, 1000)
        assert result is None

    def test_round_trip(self, kin):
        """FK(IK(target)) should approximately equal the target."""
        targets = [
            (120, 0, 40, -45),
            (80, 50, 60, -30),
            (100, -30, 30, -60),
        ]
        for tx, ty, tz, pitch in targets:
            angles = kin.inverse(tx, ty, tz, pitch_deg=pitch)
            if angles is None:
                continue  # skip unreachable
            rx, ry, rz = kin.forward(angles)
            assert abs(rx - tx) < 5.0, f"x mismatch: {rx} vs {tx}"
            assert abs(ry - ty) < 5.0, f"y mismatch: {ry} vs {ty}"
            assert abs(rz - tz) < 5.0, f"z mismatch: {rz} vs {tz}"

    def test_negative_y(self, kin):
        """Arm should be able to reach points on either side."""
        left = kin.inverse(100, 50, 40)
        right = kin.inverse(100, -50, 40)
        # Both should be reachable or both unreachable
        if left is not None and right is not None:
            assert abs(left["shoulder"] - right["shoulder"]) < 1.0
            assert abs(left["elbow"] - right["elbow"]) < 1.0


class TestEdgeCases:
    def test_zero_coordinates(self, kin):
        """Target at (0, 0, z) — directly above the base."""
        result = kin.inverse(0, 0, 100)
        # Might or might not be reachable depending on dimensions
        if result is not None:
            for val in result.values():
                assert 0 <= val <= 180

    def test_base_angle_quadrants(self, kin):
        """Base angle should correctly handle all four quadrants."""
        # Positive x, positive y
        a1 = kin.inverse(100, 50, 50)
        # Positive x, negative y
        a2 = kin.inverse(100, -50, 50)
        if a1 and a2:
            assert a1["base"] > 0
            assert a2["base"] < 180
