#!/usr/bin/env python3
"""
main.py — Entry point for the robotic arm pick-and-place system.

Usage:
    python main.py                 # continuous pick-and-place loop
    python main.py --single        # pick one object and stop
    python main.py --dry-run       # print angles without moving servos
    python main.py --calibrate     # run interactive calibration
    python main.py --test-servos   # sweep each servo for verification
    python main.py --test-camera   # capture and display a frame
"""

import sys
import argparse
import logging
import pathlib

import yaml

logger = logging.getLogger("robotic_arm")


def load_config(path: str = "config.yaml") -> dict:
    cfg_path = pathlib.Path(__file__).resolve().parent / path
    with open(cfg_path) as f:
        return yaml.safe_load(f)


def setup_logging(verbose: bool = False):
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


# ======================================================================
# Sub-commands
# ======================================================================

def cmd_run(config: dict, single: bool, dry_run: bool):
    """Main pick-and-place operation."""
    from arm.arm_controller import ArmController
    from vision.camera import Camera
    from vision.detector import ObjectDetector
    from vision.estimator import PositionEstimator
    from coordinator.pick_and_place import PickAndPlaceCoordinator

    with ArmController(config, dry_run=dry_run) as arm:
        with Camera(config["camera"]) as cam:
            detector = ObjectDetector(config["detection"])
            estimator = PositionEstimator(config.get("calibration", {}))

            coord = PickAndPlaceCoordinator(
                config, arm, cam, detector, estimator
            )

            if single:
                logger.info("Running single pick-and-place cycle")
                coord.run_once()
            else:
                logger.info("Running continuous pick-and-place loop (Ctrl-C to stop)")
                coord.run_loop()


def cmd_test_servos(config: dict):
    """Sweep each servo to verify wiring."""
    import time
    from arm.servo_controller import ServoController

    with ServoController(config["servos"]) as ctrl:
        print("\n=== Servo Test ===")
        print("Each joint will move: home → 45° → 135° → home\n")

        for name in ctrl.JOINT_NAMES:
            if name not in ctrl.joints:
                continue
            print(f"Testing {name} …", end=" ", flush=True)
            ctrl.set_angle(name, 45)
            time.sleep(0.4)
            ctrl.set_angle(name, 135)
            time.sleep(0.4)
            ctrl.set_angle(name, ctrl.joints[name].home_angle)
            time.sleep(0.3)
            print("OK")

        print("\nAll servos tested. Relaxing …")


def cmd_test_camera(config: dict):
    """Capture a frame and optionally run detection."""
    import cv2
    from vision.camera import Camera
    from vision.detector import ObjectDetector

    with Camera(config["camera"]) as cam:
        frame = cam.grab_frame()
        print(f"Frame captured: {frame.shape}")

        det = ObjectDetector(config["detection"])
        annotated, dets = det.detect_and_draw(frame)
        print(f"Detected {len(dets)} objects:")
        for d in dets:
            print(f"  {d.label} ({d.confidence:.0%})  centre={d.centre}")

        out_path = "test_detection.jpg"
        cv2.imwrite(out_path, annotated)
        print(f"Saved annotated frame to {out_path}")


def cmd_calibrate(config: dict):
    """Run the calibrate.py script."""
    import subprocess
    script = pathlib.Path(__file__).resolve().parent / "calibrate.py"
    subprocess.run([sys.executable, str(script)], check=True)


# ======================================================================
# CLI
# ======================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Robotic Arm Pick-and-Place System"
    )
    parser.add_argument("--single", action="store_true",
                        help="Pick one object and stop")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print angles without moving servos")
    parser.add_argument("--calibrate", action="store_true",
                        help="Run interactive calibration")
    parser.add_argument("--test-servos", action="store_true",
                        help="Sweep each servo for wiring verification")
    parser.add_argument("--test-camera", action="store_true",
                        help="Capture a frame and run detection")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging")
    parser.add_argument("--config", default="config.yaml",
                        help="Path to config file")
    args = parser.parse_args()

    setup_logging(args.verbose)
    config = load_config(args.config)
    logger.info("Configuration loaded")

    if args.calibrate:
        cmd_calibrate(config)
    elif args.test_servos:
        cmd_test_servos(config)
    elif args.test_camera:
        cmd_test_camera(config)
    else:
        cmd_run(config, single=args.single, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
