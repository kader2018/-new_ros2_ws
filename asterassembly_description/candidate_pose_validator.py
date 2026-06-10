#!/usr/bin/env python3
"""Validate temporary ASTER candidate poses with MoveIt2 fake hardware only."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


VALID = "VALID"
INVALID = "INVALID"
COLLISION = "COLLISION"
OUT_OF_BOUNDS = "OUT_OF_BOUNDS"

DEFAULT_MOVEIT_PACKAGE = "moveit_aster_config"
DEFAULT_ALIGNMENT_PATH = Path(__file__).with_name("ros_rviz_alignment.json")
STATE_VALIDITY_SERVICE = "/check_state_validity"


@dataclass(frozen=True)
class JointLimit:
    lower: float | None
    upper: float | None
    joint_type: str


@dataclass(frozen=True)
class ValidationResult:
    status: str
    message: str
    joints: dict[str, float]


class ValidationError(Exception):
    def __init__(self, status: str, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


def load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError as exc:
        raise ValidationError(INVALID, f"Fichier JSON introuvable: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValidationError(INVALID, f"JSON invalide: {exc}") from exc
    if not isinstance(data, dict):
        raise ValidationError(INVALID, "La pose candidate doit etre un objet JSON.")
    return data


def parse_candidate(path: Path) -> tuple[str, str, dict[str, float]]:
    data = load_json(path)
    name = data.get("name")
    group = data.get("group")
    joints = data.get("joints")

    if not isinstance(name, str) or not name.strip():
        raise ValidationError(INVALID, "Champ obligatoire invalide: name.")
    if not isinstance(group, str) or not group.strip():
        raise ValidationError(INVALID, "Champ obligatoire invalide: group.")
    if not isinstance(joints, dict) or not joints:
        raise ValidationError(INVALID, "Champ obligatoire invalide: joints.")

    parsed: dict[str, float] = {}
    for joint_name, value in joints.items():
        if not isinstance(joint_name, str) or not joint_name.strip():
            raise ValidationError(INVALID, "Nom de joint invalide.")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValidationError(INVALID, f"Valeur non numerique pour {joint_name}.")
        value = float(value)
        if not math.isfinite(value):
            raise ValidationError(INVALID, f"Valeur non finie pour {joint_name}.")
        parsed[joint_name] = value

    return name.strip(), group.strip(), parsed


def get_config_dir(package_name: str) -> Path:
    try:
        return Path(get_package_share_directory(package_name)) / "config"
    except PackageNotFoundError as exc:
        raise ValidationError(
            INVALID,
            f"Package MoveIt2 introuvable: {package_name}. Source le workspace ROS2.",
        ) from exc


def load_srdf_groups(srdf_path: Path) -> dict[str, list[str]]:
    try:
        root = ET.parse(srdf_path).getroot()
    except (FileNotFoundError, ET.ParseError) as exc:
        raise ValidationError(INVALID, f"SRDF illisible: {srdf_path}") from exc

    direct: dict[str, list[str]] = {}
    nested: dict[str, list[str]] = {}
    for group in root.findall("group"):
        group_name = group.attrib.get("name")
        if not group_name:
            continue
        direct[group_name] = [
            joint.attrib["name"]
            for joint in group.findall("joint")
            if "name" in joint.attrib
        ]
        nested[group_name] = [
            subgroup.attrib["name"]
            for subgroup in group.findall("group")
            if "name" in subgroup.attrib
        ]

    def expand(group_name: str, stack: tuple[str, ...] = ()) -> list[str]:
        if group_name in stack:
            raise ValidationError(INVALID, f"Cycle SRDF detecte: {' -> '.join(stack + (group_name,))}")
        joints = list(direct.get(group_name, []))
        for subgroup_name in nested.get(group_name, []):
            joints.extend(expand(subgroup_name, stack + (group_name,)))
        return list(dict.fromkeys(joints))

    return {group_name: expand(group_name) for group_name in direct}


def load_urdf_limits(urdf_xacro_path: Path) -> dict[str, JointLimit]:
    try:
        doc = xacro.process_file(str(urdf_xacro_path))
        root = ET.fromstring(doc.toxml())
    except Exception as exc:
        raise ValidationError(INVALID, f"URDF/Xacro illisible: {urdf_xacro_path}") from exc

    limits: dict[str, JointLimit] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if not name:
            continue
        joint_type = joint.attrib.get("type", "")
        limit_tag = joint.find("limit")
        lower = upper = None
        if limit_tag is not None:
            lower = float(limit_tag.attrib["lower"]) if "lower" in limit_tag.attrib else None
            upper = float(limit_tag.attrib["upper"]) if "upper" in limit_tag.attrib else None
        limits[name] = JointLimit(lower=lower, upper=upper, joint_type=joint_type)
    return limits


def load_controllers(controllers_path: Path) -> dict[str, list[str]]:
    try:
        with controllers_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except FileNotFoundError as exc:
        raise ValidationError(INVALID, f"ros2_controllers.yaml introuvable: {controllers_path}") from exc

    controllers: dict[str, list[str]] = {}
    for name, value in data.items():
        if name == "controller_manager" or not isinstance(value, dict):
            continue
        params = value.get("ros__parameters", {})
        joints = params.get("joints", [])
        if isinstance(joints, list) and joints:
            controllers[name] = [str(joint) for joint in joints]
    return controllers


def load_alignment(path: Path) -> dict[str, dict[str, float]]:
    if not path.exists():
        return {}
    data = load_json(path)
    raw_joints = data.get("joints", {})
    if not isinstance(raw_joints, dict):
        raise ValidationError(INVALID, f"Format d'alignement invalide: {path}")

    alignment: dict[str, dict[str, float]] = {}
    for joint_name, entry in raw_joints.items():
        if not isinstance(entry, dict):
            continue
        alignment[joint_name] = {
            "visual_sign": float(entry.get("visual_sign", 1.0)),
            "visual_scale": float(entry.get("visual_scale", 1.0)),
            "visual_offset_rad": float(entry.get("visual_offset_rad", 0.0)),
        }
    return alignment


def apply_alignment(joints: dict[str, float], alignment: dict[str, dict[str, float]]) -> dict[str, float]:
    aligned: dict[str, float] = {}
    for joint_name, value in joints.items():
        entry = alignment.get(joint_name, {})
        aligned[joint_name] = (
            value
            * entry.get("visual_sign", 1.0)
            * entry.get("visual_scale", 1.0)
            + entry.get("visual_offset_rad", 0.0)
        )
    return aligned


def validate_joint_names(group: str, joints: dict[str, float], groups: dict[str, list[str]]) -> None:
    if group not in groups:
        raise ValidationError(INVALID, f"Groupe MoveIt2 inconnu: {group}. Disponibles: {sorted(groups)}")

    known_joints = {joint for group_joints in groups.values() for joint in group_joints}
    group_joints = set(groups[group])
    for joint_name in joints:
        if joint_name not in known_joints:
            raise ValidationError(INVALID, f"Joint inconnu: {joint_name}")
        if joint_name not in group_joints:
            raise ValidationError(INVALID, f"Joint {joint_name} hors du groupe {group}")


def complete_group_pose(group: str, candidate_joints: dict[str, float], groups: dict[str, list[str]]) -> dict[str, float]:
    completed = {joint_name: 0.0 for joint_name in groups[group]}
    completed.update(candidate_joints)
    return completed


def validate_limits(joints: dict[str, float], limits: dict[str, JointLimit], tolerance: float) -> None:
    for joint_name, value in joints.items():
        limit = limits.get(joint_name)
        if limit is None:
            raise ValidationError(INVALID, f"Limites URDF introuvables pour {joint_name}")
        if limit.joint_type == "continuous":
            continue
        if limit.lower is not None and value < limit.lower - tolerance:
            raise ValidationError(OUT_OF_BOUNDS, f"{joint_name}={value:.6f} < {limit.lower:.6f}")
        if limit.upper is not None and value > limit.upper + tolerance:
            raise ValidationError(OUT_OF_BOUNDS, f"{joint_name}={value:.6f} > {limit.upper:.6f}")


def make_robot_state(joints: dict[str, float]) -> RobotState:
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(joints.keys())
    state.joint_state.position = list(joints.values())
    state.is_diff = True
    return state


class MoveItRuntime(Node):
    def __init__(self) -> None:
        super().__init__("candidate_pose_validator")
        self.state_validity = self.create_client(GetStateValidity, STATE_VALIDITY_SERVICE)
        self.action_clients: dict[str, ActionClient] = {}

    def check_collision(self, group: str, joints: dict[str, float], timeout: float) -> ValidationResult:
        if not self.state_validity.wait_for_service(timeout_sec=timeout):
            raise ValidationError(INVALID, f"Service MoveIt2 indisponible: {STATE_VALIDITY_SERVICE}")

        request = GetStateValidity.Request()
        request.group_name = group
        request.robot_state = make_robot_state(joints)

        future = self.state_validity.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise ValidationError(INVALID, "Timeout pendant la verification MoveIt2.")

        response = future.result()
        if response.valid:
            return ValidationResult(VALID, "Pose valide MoveIt2.", joints)
        if response.contacts:
            contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in response.contacts[:5]]
            return ValidationResult(COLLISION, "Collision detectee: " + ", ".join(contacts), joints)
        return ValidationResult(INVALID, "Etat MoveIt2 invalide.", joints)

    def preview_fake_hardware(
        self,
        group: str,
        joints: dict[str, float],
        groups: dict[str, list[str]],
        controllers: dict[str, list[str]],
        timeout: float,
    ) -> None:
        controller_name = f"{group}_controller"
        if controller_name not in controllers:
            raise ValidationError(INVALID, f"Controleur fake hardware introuvable: {controller_name}")

        controller_joints = controllers[controller_name]
        trajectory = JointTrajectory()
        trajectory.joint_names = controller_joints
        point = JointTrajectoryPoint()
        point.positions = [joints.get(joint_name, 0.0) for joint_name in controller_joints]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        trajectory.points = [point]

        action_name = f"/{controller_name}/follow_joint_trajectory"
        client = ActionClient(self, FollowJointTrajectory, action_name)
        self.action_clients[controller_name] = client
        if not client.wait_for_server(timeout_sec=timeout):
            raise ValidationError(INVALID, f"Action fake hardware indisponible: {action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None or not future.result().accepted:
            raise ValidationError(INVALID, f"Preview refusee par {controller_name}")

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout + 2.0)
        time.sleep(0.2)


def validate_candidate(args: argparse.Namespace) -> ValidationResult:
    _, group, raw_joints = parse_candidate(args.candidate_json)
    config_dir = get_config_dir(args.moveit_package)
    groups = load_srdf_groups(config_dir / "asterassembly.srdf")
    limits = load_urdf_limits(config_dir / "asterassembly.urdf.xacro")
    alignment = load_alignment(args.alignment)

    validate_joint_names(group, raw_joints, groups)
    full_pose = complete_group_pose(group, raw_joints, groups)
    aligned_pose = apply_alignment(full_pose, alignment)
    validate_limits(aligned_pose, limits, args.tolerance)

    rclpy.init(args=None)
    node = MoveItRuntime()
    try:
        result = node.check_collision(group, aligned_pose, args.timeout)
        if args.preview and result.status == VALID:
            controllers = load_controllers(config_dir / "ros2_controllers.yaml")
            node.preview_fake_hardware(group, aligned_pose, groups, controllers, args.timeout)
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate a temporary ASTER candidate pose with MoveIt2.")
    parser.add_argument("candidate_json", type=Path)
    parser.add_argument("--preview", action="store_true", help="Afficher la pose dans RViz fake hardware.")
    parser.add_argument("--check-only", action="store_true", help="Validation console uniquement.")
    parser.add_argument("--alignment", type=Path, default=DEFAULT_ALIGNMENT_PATH)
    parser.add_argument("--moveit-package", default=DEFAULT_MOVEIT_PACKAGE)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.preview and args.check_only:
        print(f"{INVALID}: --preview et --check-only sont incompatibles.")
        return 2
    if not args.preview:
        args.check_only = True

    try:
        result = validate_candidate(args)
    except ValidationError as exc:
        print(f"{exc.status}: {exc.message}")
        return {INVALID: 2, OUT_OF_BOUNDS: 3, COLLISION: 4}.get(exc.status, 2)
    except KeyboardInterrupt:
        print(f"{INVALID}: interruption utilisateur.")
        return 130

    print(f"{result.status}: {result.message}")
    for joint_name in sorted(result.joints):
        print(f"  {joint_name}: {result.joints[joint_name]:.6f}")
    return {VALID: 0, INVALID: 2, OUT_OF_BOUNDS: 3, COLLISION: 4}.get(result.status, 2)


if __name__ == "__main__":
    raise SystemExit(main())
