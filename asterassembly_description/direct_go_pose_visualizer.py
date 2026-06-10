#!/usr/bin/env python3
"""
Safe DIRECT_GO pose visualizer for RViz.

This node reads servo_calibration.json, converts stored DIRECT_GO servo angles
back to ROS joint radians, and publishes sensor_msgs/JointState for RViz only.
It never opens the serial port and refuses to start while ros2_serial_bridge.py
is running, because the bridge subscribes to /joint_states and would forward
published poses to the real robot.
"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SCRIPT_DIR = Path(__file__).parent.absolute()
CALIB_PATH = SCRIPT_DIR / "servo_calibration.json"
ALIGNMENT_PATH = SCRIPT_DIR / "ros_rviz_alignment.json"

POSE_ORDER = ["bonjour", "oui_A", "oui_B", "non_A", "non_B", "pense"]
REST_POSE_NAME = "rest"
SCALE_JOINT_DEG = 1.0

# This table is intentionally separate from ros2_serial_bridge.py.
# The bridge table maps ROS JointState -> physical servo direction. This table
# maps stored servo angles -> RViz joint direction for visual validation only.
VISUALIZER_SIGN_TABLE: Dict[str, float] = {
    "bras_droit_de_ar_joint": +1.0,
    "bras_droit_h_b_joint": +1.0,
    "bras_droit_rot_joint": +1.0,
    "bras_droit_coud_joint": +1.0,
    "bras_gauche_de_ar_joint": +1.0,
    "bras_gauche_h_b_joint": +1.0,
    "bras_gauche_rot_joint": +1.0,
    "bras_gauche_coude_joint": +1.0,
    "cuisse_droit_joint": +1.0,
    "genou_droit_joint": +1.0,
    "cheville_droite_joint": +1.0,
    "cuisse_gauche_joint": +1.0,
    "genou_gauche_joint": +1.0,
    "cheville_gauche_joint": +1.0,
    "tete_g_d_joint": +1.0,
    "tete_h_b_joint": +1.0,
}


def bridge_process_is_running() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", "ros2_serial_bridge.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_sign_overrides(overrides: List[str]) -> Dict[str, float]:
    parsed: Dict[str, float] = {}
    for override in overrides:
        if "=" not in override:
            raise ValueError(f"Invalid --sign format: {override}. Expected joint=+1 or joint=-1")
        joint_name, raw_sign = override.split("=", 1)
        joint_name = joint_name.strip()
        raw_sign = raw_sign.strip()
        if raw_sign not in {"+1", "1", "-1"}:
            raise ValueError(f"Invalid sign for {joint_name}: {raw_sign}. Expected +1 or -1")
        parsed[joint_name] = -1.0 if raw_sign == "-1" else +1.0
    return parsed


class DirectGoPoseLibrary:
    def __init__(
        self,
        calib_path: Path,
        alignment_path: Path,
        sign_overrides: Optional[Dict[str, float]] = None,
    ) -> None:
        if not calib_path.exists():
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        self.calib_path = calib_path
        self.calib = json.loads(calib_path.read_text(encoding="utf-8"))
        self.alignment = self._load_alignment(alignment_path)
        self.sign_table = dict(VISUALIZER_SIGN_TABLE)
        for joint_name, entry in self.alignment.items():
            if "visual_sign" in entry:
                self.sign_table[joint_name] = float(entry.get("visual_sign", 1.0))
        if sign_overrides:
            unknown = [name for name in sign_overrides if name not in self.sign_table]
            if unknown:
                raise ValueError(f"Unknown joint in sign override: {', '.join(unknown)}")
            self.sign_table.update(sign_overrides)
        self.joint_names: List[str] = []
        self.servo_rest: List[float] = []
        self.servo_min: List[float] = []
        self.servo_max: List[float] = []
        self._load_servos()

    def _load_alignment(self, alignment_path: Path) -> Dict[str, dict]:
        if not alignment_path.exists():
            return {}
        data = json.loads(alignment_path.read_text(encoding="utf-8"))
        return data.get("joints", {})

    def _alignment_entry(self, joint_name: str) -> dict:
        return self.alignment.get(joint_name, {})

    def _visual_scale(self, joint_name: str) -> float:
        return float(self._alignment_entry(joint_name).get("visual_scale", 1.0))

    def _visual_offset_rad(self, joint_name: str) -> float:
        return float(self._alignment_entry(joint_name).get("visual_offset_rad", 0.0))

    def _load_servos(self) -> None:
        servos_raw = self.calib.get("servos", {})
        if isinstance(servos_raw, dict):
            items = sorted(servos_raw.items(), key=lambda kv: int(kv[0]))
            servos = [value for _, value in items]
        else:
            servos = list(servos_raw)

        if len(servos) != 16:
            raise ValueError(f"Expected 16 servos, got {len(servos)}")

        for idx, servo in enumerate(servos):
            name = servo.get("name")
            if not name:
                raise ValueError(f"Servo {idx} has no joint name")
            low = float(servo.get("low_mech_constraint", 0.0))
            high = float(servo.get("high_mech_constraint", 180.0))
            rest_raw = float(servo.get("rest_position", 90.0))
            rest = clamp(rest_raw, low, high)

            self.joint_names.append(name)
            self.servo_min.append(low)
            self.servo_max.append(high)
            self.servo_rest.append(rest)

    def available_poses(self) -> List[str]:
        predefined = self.calib.get("positions_predefinies", {})
        return [pose for pose in POSE_ORDER if pose in predefined]

    def rest_positions(self) -> Tuple[List[str], List[float]]:
        positions = [
            self.servo_deg_to_joint_rad(joint_name, self.servo_rest[idx])
            for idx, joint_name in enumerate(self.joint_names)
        ]
        return self.joint_names, positions

    def servo_deg_to_joint_rad(self, joint_name: str, servo_deg: float) -> float:
        idx = self.joint_names.index(joint_name)
        servo_eff = clamp(float(servo_deg), self.servo_min[idx], self.servo_max[idx])
        sign = self.sign_table.get(joint_name, +1.0)
        scale = self._visual_scale(joint_name)
        offset = self._visual_offset_rad(joint_name)
        return math.radians((servo_eff - self.servo_rest[idx]) * sign * scale / SCALE_JOINT_DEG) + offset

    def pose_to_joint_positions(self, pose_name: str) -> Tuple[List[str], List[float]]:
        if pose_name == REST_POSE_NAME:
            return self.rest_positions()

        predefined = self.calib.get("positions_predefinies", {})
        pose = predefined.get(pose_name)
        if pose is None:
            raise KeyError(f"Unknown DIRECT_GO pose: {pose_name}")

        positions: List[float] = []
        for idx, joint_name in enumerate(self.joint_names):
            raw_value = pose.get(str(idx), pose.get(idx))
            if raw_value is None:
                raw_value = self.servo_rest[idx]
            positions.append(self.servo_deg_to_joint_rad(joint_name, float(raw_value)))
        return self.joint_names, positions

    def test_joint_positions(self, joint_name: str, delta_deg: float, invert: bool) -> Tuple[List[str], List[float]]:
        if joint_name not in self.joint_names:
            raise ValueError(f"Unknown joint: {joint_name}")
        names, positions = self.rest_positions()
        idx = self.joint_names.index(joint_name)
        sign = -1.0 if invert else +1.0
        positions[idx] += math.radians(delta_deg * sign)
        return names, positions

    def describe_rest_mismatches(self) -> List[str]:
        global_rest = self.calib.get("rest_positions", [])
        if len(global_rest) != len(self.servo_rest):
            return []
        mismatches: List[str] = []
        for idx, (global_value, servo_value) in enumerate(zip(global_rest, self.servo_rest)):
            if abs(float(global_value) - servo_value) > 1.0:
                mismatches.append(
                    f"servo {idx} {self.joint_names[idx]}: rest_positions={global_value} servos.rest_position={servo_value:g}"
                )
        return mismatches


class DirectGoPoseVisualizer(Node):
    def __init__(
        self,
        pose_library: DirectGoPoseLibrary,
        pose_names: List[str],
        period_s: float,
        test_joint: Optional[str],
        test_delta_deg: float,
        invert_test_joint: bool,
    ) -> None:
        super().__init__("direct_go_pose_visualizer")
        self.pose_library = pose_library
        self.pose_names = pose_names
        self.period_s = max(0.2, period_s)
        self.pose_index = 0
        self.test_joint = test_joint
        self.test_delta_deg = test_delta_deg
        self.invert_test_joint = invert_test_joint
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self.publish_current_pose)
        if not self.test_joint and len(self.pose_names) > 1:
            self.create_timer(self.period_s, self.advance_pose)

        self.get_logger().info(
            "DIRECT_GO RViz visualizer active. Bridge is not running; publishing /joint_states only."
        )
        if self.test_joint:
            direction = "inverted" if self.invert_test_joint else "normal"
            self.get_logger().info(
                f"Testing one joint from rest: {self.test_joint} {self.test_delta_deg:g} deg ({direction})"
            )
        else:
            self.get_logger().info(f"Poses: {', '.join(self.pose_names)}")

        mismatches = self.pose_library.describe_rest_mismatches()
        for mismatch in mismatches:
            self.get_logger().warn(f"Calibration rest mismatch: {mismatch}")

    def advance_pose(self) -> None:
        self.pose_index = (self.pose_index + 1) % len(self.pose_names)
        self.get_logger().info(f"Showing pose: {self.pose_names[self.pose_index]}")

    def publish_current_pose(self) -> None:
        if self.test_joint:
            names, positions = self.pose_library.test_joint_positions(
                self.test_joint,
                self.test_delta_deg,
                self.invert_test_joint,
            )
        else:
            pose_name = self.pose_names[self.pose_index]
            names, positions = self.pose_library.pose_to_joint_positions(pose_name)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize DIRECT_GO poses in RViz without moving ASTER.")
    parser.add_argument(
        "--pose",
        default=REST_POSE_NAME,
        choices=[REST_POSE_NAME] + POSE_ORDER + ["all"],
        help="Pose to display. Defaults to rest so RViz receives a complete neutral JointState.",
    )
    parser.add_argument(
        "--rest",
        action="store_true",
        help="Shortcut for --pose rest.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=3.0,
        help="Seconds between poses when --pose all is used.",
    )
    parser.add_argument(
        "--calibration",
        default=str(CALIB_PATH),
        help="Path to servo_calibration.json.",
    )
    parser.add_argument(
        "--alignment",
        default=str(ALIGNMENT_PATH),
        help="Path to ros_rviz_alignment.json.",
    )
    parser.add_argument(
        "--sign",
        action="append",
        default=[],
        help="Temporary visual sign override for one joint, format joint=+1 or joint=-1. Can be repeated.",
    )
    parser.add_argument(
        "--test-joint",
        choices=list(VISUALIZER_SIGN_TABLE.keys()),
        help="Publish rest pose plus one joint offset, for visual sign testing only.",
    )
    parser.add_argument(
        "--test-deg",
        type=float,
        default=20.0,
        help="Joint offset in degrees for --test-joint.",
    )
    parser.add_argument(
        "--invert-test-joint",
        action="store_true",
        help="Invert the --test-joint offset direction without changing calibration or poses.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if bridge_process_is_running():
        print(
            "SAFETY REFUSAL: ros2_serial_bridge.py is running. "
            "Stop the bridge before visualizing DIRECT_GO poses in RViz.",
            file=sys.stderr,
        )
        return 2

    try:
        sign_overrides = parse_sign_overrides(args.sign)
        pose_library = DirectGoPoseLibrary(Path(args.calibration), Path(args.alignment), sign_overrides)
    except Exception as exc:
        print(f"Failed to load DIRECT_GO visualizer: {exc}", file=sys.stderr)
        return 1

    if args.rest:
        args.pose = REST_POSE_NAME

    if args.pose == "all":
        pose_names = [REST_POSE_NAME] + pose_library.available_poses()
    else:
        pose_names = [args.pose]

    if not pose_names:
        print("No DIRECT_GO poses available to visualize.", file=sys.stderr)
        return 1

    rclpy.init()
    node = DirectGoPoseVisualizer(
        pose_library,
        pose_names,
        args.period,
        args.test_joint,
        args.test_deg,
        args.invert_test_joint,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
