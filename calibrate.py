#!/usr/bin/env python3
"""
calibrate.py — Interactive calibration for servos and camera.

Usage:
    python calibrate.py              # full calibration (servos + camera)
    python calibrate.py --servos     # servo limits only
    python calibrate.py --camera     # camera homography only
"""

import argparse
import logging
import pathlib
import time

import yaml

logger = logging.getLogger(__name__)

CONFIG_PATH = pathlib.Path(__file__).resolve().parent / "config.yaml"


def load_config() -> dict:
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def save_config(config: dict):
    with open(CONFIG_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"Configuration saved to {CONFIG_PATH}")


# ======================================================================
# Servo calibration
# ======================================================================

def calibrate_servos(config: dict):
    """Interactive servo calibration — test each joint's range."""
    from arm.servo_controller import ServoController

    print("\n" + "=" * 50)
    print("  SERVO CALIBRATION")
    print("=" * 50)
    print(
        "\nFor each joint, you'll set the min and max angles.\n"
        "Type an angle (0–180) and press Enter to move.\n"
        "Type 'ok' when you're happy with the current position.\n"
        "Type 'skip' to skip this joint.\n"
    )

    with ServoController(config["servos"]) as ctrl:
        for name in ctrl.JOINT_NAMES:
            if name not in ctrl.joints:
                continue

            joint = ctrl.joints[name]
            print(f"\n--- {name.upper()} (GPIO {config['servos'][name]['gpio_pin']}) ---")
            print(f"  Current range: {joint.min_angle}°–{joint.max_angle}°")
            print(f"  Home: {joint.home_angle}°")

            # Move to home first
            joint.set_angle(joint.home_angle)
            time.sleep(0.3)

            while True:
                cmd = input(f"  [{name}] Enter angle / 'ok' / 'skip': ").strip()
                if cmd.lower() == "skip":
                    break
                if cmd.lower() == "ok":
                    new_home = input(f"  Set new home angle (current={joint.home_angle}°, Enter to keep): ").strip()
                    if new_home:
                        config["servos"][name]["home_angle"] = float(new_home)
                    break
                try:
                    angle = float(cmd)
                    joint.set_angle(angle, smooth=True)
                except ValueError:
                    print("  Invalid input — enter a number, 'ok', or 'skip'")
                except Exception as e:
                    print(f"  Error: {e}")

    save_config(config)
    print("\nServo calibration complete!")


# ======================================================================
# Camera calibration
# ======================================================================

def calibrate_camera(config: dict):
    """Interactive camera-to-world homography calibration."""
    from vision.camera import Camera
    from vision.estimator import PositionEstimator

    print("\n" + "=" * 50)
    print("  CAMERA CALIBRATION")
    print("=" * 50)

    with Camera(config["camera"]) as cam:
        points = PositionEstimator.run_interactive_calibration(cam, num_points=6)

    if len(points) >= 4:
        if "calibration" not in config:
            config["calibration"] = {}
        config["calibration"]["points"] = points
        save_config(config)

        # Verify
        est = PositionEstimator(config["calibration"])
        if est.is_calibrated:
            print("\nCalibration successful! Testing with captured points:")
            for pt in points:
                world = est.pixel_to_world(pt[0], pt[1])
                print(
                    f"  pixel ({pt[0]}, {pt[1]}) → world ({world[0]:.1f}, {world[1]:.1f}) mm  "
                    f"(expected: ({pt[2]:.1f}, {pt[3]:.1f}))"
                )
    else:
        print(f"\nOnly {len(points)} points captured (need ≥ 4). Calibration skipped.")


# ======================================================================
# Main
# ======================================================================

def main():
    parser = argparse.ArgumentParser(description="Robotic Arm Calibration Tool")
    parser.add_argument("--servos", action="store_true", help="Servo calibration only")
    parser.add_argument("--camera", action="store_true", help="Camera calibration only")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    config = load_config()

    if args.servos:
        calibrate_servos(config)
    elif args.camera:
        calibrate_camera(config)
    else:
        calibrate_servos(config)
        calibrate_camera(config)

    print("\nAll done!")


if __name__ == "__main__":
    main()
