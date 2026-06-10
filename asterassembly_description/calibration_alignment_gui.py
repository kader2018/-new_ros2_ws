#!/usr/bin/env python3
"""
ASTER calibration alignment GUI.

Standalone tool for physical servo calibration checks and RViz alignment.
It moves the real servo through Play.ino using only S{id}:{angle}, publishes
/joint_states for RViz, and stores visual-only corrections in
ros_rviz_alignment.json.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:  # pragma: no cover - reported at runtime in main
    serial = None

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError:  # pragma: no cover - reported at runtime in main
    QtCore = None
    QtWidgets = None

SCRIPT_DIR = Path(__file__).parent.absolute()
CALIB_PATH = SCRIPT_DIR / "servo_calibration.json"
ALIGNMENT_PATH = SCRIPT_DIR / "ros_rviz_alignment.json"
SERIAL_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUDRATE = 115200
PUBLISH_HZ = 10.0
SERIAL_MIN_INTERVAL_S = 0.08
ARDUINO_RESET_WAIT_S = 1.5
REST_SYNC_TIMEOUT_MS = 6000
SERIAL_OPEN_TIMEOUT_S = 1.0
ARDUINO_READY_TIMEOUT_S = 8.0
REST_APPLY_DELAY_S = 0.4
RVIZ_LOG_PATH = Path("/tmp/aster_rviz_launch.log")
SERIAL_LOG_PATH = Path("/tmp/aster_calibration_serial.log")
RVIZ_LAUNCH_CMD = (
    "cd /home/addala/ros2_moveit_ws && "
    "source /opt/ros/jazzy/setup.bash && "
    "source install/setup.bash && "
    "ros2 launch asterassembly_description display.launch.py"
)


def bridge_process_is_running() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", "ros2_serial_bridge.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def rviz_process_is_running() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", "rviz2"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def rviz_launch_pids() -> List[str]:
    result = subprocess.run(
        ["pgrep", "-f", "ros2 launch asterassembly_description display.launch.py"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return []
    return [pid for pid in result.stdout.split() if pid.isdigit() and pid != str(os.getpid())]


def stop_rviz_launch_processes() -> None:
    for pid in rviz_launch_pids():
        try:
            os.kill(int(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass


def port_users(port: str) -> List[str]:
    if not Path(port).exists():
        return []
    commands = (["lsof", "-t", port], ["fuser", port])
    for command in commands:
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=1.0)
        except (FileNotFoundError, subprocess.SubprocessError):
            continue
        output = (result.stdout or "") + " " + (result.stderr or "")
        pids = []
        for token in output.replace(":", " ").split():
            if token.isdigit() and token != str(os.getpid()):
                pids.append(token)
        if pids:
            return sorted(set(pids))
    return []


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class ServoCalibration:
    def __init__(self, path: Path) -> None:
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")
        self.path = path
        self.data = json.loads(path.read_text(encoding="utf-8"))
        self.servos = self._load_servos()

    def _load_servos(self) -> List[dict]:
        raw = self.data.get("servos", {})
        if isinstance(raw, dict):
            items = sorted(raw.items(), key=lambda kv: int(kv[0]))
            servos = [value for _, value in items]
        else:
            servos = list(raw)
        if len(servos) != 16:
            raise ValueError(f"Expected 16 servos, got {len(servos)}")
        normalized = []
        for idx, servo in enumerate(servos):
            name = servo.get("name")
            if not name:
                raise ValueError(f"Servo {idx} has no joint name")
            low = float(servo.get("low_mech_constraint", 0.0))
            high = float(servo.get("high_mech_constraint", 180.0))
            rest = clamp(float(servo.get("rest_position", 90.0)), low, high)
            normalized.append({"index": idx, "name": name, "min": low, "max": high, "rest": rest})
        return normalized

    @property
    def joint_names(self) -> List[str]:
        return [servo["name"] for servo in self.servos]

    def rest_positions_int(self) -> List[int]:
        return [int(round(servo["rest"])) for servo in self.servos]

    def set_mechanical_value(self, servo_index: int, key: str, value: float) -> None:
        if key not in {"min", "max", "rest"}:
            raise ValueError(f"Unsupported mechanical key: {key}")
        servo = self.servos[servo_index]
        value = float(clamp(value, 0.0, 180.0))
        if key == "min" and value > servo["max"]:
            raise ValueError("Le MIN reel ne peut pas etre superieur au MAX reel actuel")
        if key == "max" and value < servo["min"]:
            raise ValueError("Le MAX reel ne peut pas etre inferieur au MIN reel actuel")
        if key == "rest" and not servo["min"] <= value <= servo["max"]:
            raise ValueError("Le REPOS reel doit rester entre MIN reel et MAX reel")
        servo[key] = value
        if key in {"min", "max"} and not servo["min"] <= servo["rest"] <= servo["max"]:
            servo["rest"] = clamp(servo["rest"], servo["min"], servo["max"])

    def save(self) -> None:
        raw = self.data.get("servos", {})
        for servo in self.servos:
            idx = servo["index"]
            raw_servo = raw[str(idx)] if isinstance(raw, dict) else raw[idx]
            raw_servo["low_mech_constraint"] = float(servo["min"])
            raw_servo["high_mech_constraint"] = float(servo["max"])
            raw_servo["rest_position"] = float(servo["rest"])
        rest_positions = self.data.get("rest_positions")
        if isinstance(rest_positions, list):
            for servo in self.servos:
                idx = servo["index"]
                if idx < len(rest_positions):
                    rest_positions[idx] = int(round(servo["rest"]))
        self.path.write_text(json.dumps(self.data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")


class AlignmentStore:
    def __init__(self, path: Path, calibration: ServoCalibration) -> None:
        self.path = path
        self.calibration = calibration
        self.data = self._load_or_default()
        self._ensure_all_joints()

    def _load_or_default(self) -> dict:
        if self.path.exists():
            return json.loads(self.path.read_text(encoding="utf-8"))
        return {
            "version": 1,
            "source_calibration": "servo_calibration.json",
            "joints": {},
        }

    def _ensure_all_joints(self) -> None:
        self.data.setdefault("version", 1)
        self.data.setdefault("source_calibration", "servo_calibration.json")
        joints = self.data.setdefault("joints", {})
        for servo in self.calibration.servos:
            joint = servo["name"]
            entry = joints.setdefault(joint, {})
            entry.setdefault("servo_index", servo["index"])
            entry.setdefault("visual_sign", 1.0)
            entry.setdefault("visual_scale", 1.0)
            entry.setdefault("visual_offset_rad", 0.0)
            entry.setdefault("note", "")
            entry.setdefault("validated_at", None)

    def entry(self, joint_name: str) -> dict:
        return self.data["joints"][joint_name]

    def set_entry(
        self,
        joint_name: str,
        visual_sign: float,
        visual_scale: float,
        visual_offset_rad: float,
        note: str,
    ) -> None:
        entry = self.entry(joint_name)
        entry["visual_sign"] = float(visual_sign)
        entry["visual_scale"] = float(visual_scale)
        entry["visual_offset_rad"] = float(visual_offset_rad)
        entry["note"] = note
        entry["validated_at"] = datetime.now(timezone.utc).isoformat()

    def save(self) -> None:
        self.path.write_text(json.dumps(self.data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")


class SerialWorker(QtCore.QObject):
    status = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal(str)
    connection_failed = QtCore.pyqtSignal(str)
    rest_sent = QtCore.pyqtSignal()
    serial_error = QtCore.pyqtSignal(str)
    angle_sent = QtCore.pyqtSignal(int, int)

    def __init__(self, ports: List[str], baudrate: int) -> None:
        super().__init__()
        self.ports = ports
        self.baudrate = baudrate
        self.ser = None
        self.port = ""
        self.last_sent: Dict[int, int] = {}
        self.last_send_time = 0.0
        self.opening = False
        self.state = "DISCONNECTED"

    def _log_serial(self, message: str) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        with SERIAL_LOG_PATH.open("a", encoding="utf-8") as log_file:
            log_file.write(f"{timestamp} {message}\n")

    def _is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _close_current(self) -> None:
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self._enable_telemetry_before_close()
                    self.ser.flush()
                    self.ser.close()
            except Exception as exc:
                self.serial_error.emit(f"Erreur fermeture port serie: {exc}")
            finally:
                old_port = self.port or "Arduino"
                self._log_serial(f"CLOSE {old_port}")
                self.ser = None
                self.port = ""
                self.state = "DISCONNECTED"
                self.last_sent.clear()
                self.last_send_time = 0.0
                self.disconnected.emit(f"{old_port} deconnecte")

    def _wait_for_arduino_ready(self) -> bool:
        if not self._is_open():
            return False
        deadline = time.monotonic() + ARDUINO_READY_TIMEOUT_S
        self.state = "WAIT_ARDUINO_READY"
        self.status.emit("Attente ASTER_READY...")
        while time.monotonic() < deadline:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if raw:
                        self._log_serial(f"RX {raw}")
                    if "ASTER_READY" in raw:
                        return True
                else:
                    time.sleep(0.02)
            except Exception as exc:
                self.serial_error.emit(f"Erreur lecture Arduino: {exc}")
                return False
        self._log_serial("TIMEOUT ASTER_READY")
        return False

    @QtCore.pyqtSlot()
    def open_serial(self) -> None:
        if self.opening:
            self.status.emit("Connexion Arduino deja en cours...")
            return
        if self._is_open():
            self.connected.emit(self.port)
            return
        if serial is None:
            self.connection_failed.emit("pyserial is not installed")
            return

        self.opening = True
        last_error = None
        self.status.emit("Connexion Arduino...")
        try:
            for port in self.ports:
                users = port_users(port)
                if users:
                    self.connection_failed.emit(f"{port} deja occupe par PID(s): {', '.join(users)}")
                    return

                try:
                    self.state = "CONNECTING"
                    self.ser = serial.Serial(port, self.baudrate, timeout=0.1, write_timeout=None)
                    self.port = port
                    self._log_serial(f"OPEN {port}")
                    self.status.emit(f"Arduino detecte sur {port}; attente reset...")
                    time.sleep(ARDUINO_RESET_WAIT_S)
                    if not self._wait_for_arduino_ready():
                        self.connection_failed.emit(f"{port} ouvert mais ASTER_READY non recu")
                        self._close_current()
                        return
                    self._disable_telemetry_for_calibration()
                    self.state = "CONNECTED"
                    self.connected.emit(port)
                    return
                except Exception as exc:
                    last_error = exc
                    self._close_current()
            self.connection_failed.emit(f"Impossible d'ouvrir {self.ports}: {last_error}")
        finally:
            self.opening = False

    def _write_line(self, line: str) -> None:
        if not self._is_open():
            raise RuntimeError("Port serie non ouvert")
        frame = line.rstrip("\n") + "\n"
        self._log_serial(f"TX {frame.strip()}")
        self.ser.write(frame.encode("ascii"))
        self.ser.flush()

    def _disable_telemetry_for_calibration(self) -> None:
        self.status.emit("Desactivation telemetrie Arduino...")
        self._write_line("TELEMETRY_OFF")
        time.sleep(0.1)
        try:
            self.ser.reset_input_buffer()
            self._log_serial("INPUT_BUFFER_CLEARED")
        except Exception as exc:
            self._log_serial(f"INPUT_BUFFER_CLEAR_FAILED {exc}")

    def _enable_telemetry_before_close(self) -> None:
        if not self._is_open():
            return
        try:
            self._write_line("TELEMETRY_ON")
            time.sleep(0.05)
        except Exception as exc:
            self._log_serial(f"TELEMETRY_ON_FAILED {exc}")

    @QtCore.pyqtSlot(list)
    def send_rest_frame(self, rest_positions: List[int]) -> None:
        if not self._is_open():
            self.serial_error.emit("Port serie non ouvert")
            return
        if len(rest_positions) != 16:
            self.serial_error.emit(f"Expected 16 rest positions, got {len(rest_positions)}")
            return
        try:
            self.state = "SENDING_REST"
            self.status.emit("Envoi repos JSON vers Arduino...")
            values = [str(int(clamp(value, 0, 180))) for value in rest_positions]
            frame = "M;" + ";".join(values) + "\n"
            self._write_line(frame.strip())
            time.sleep(REST_APPLY_DELAY_S)
            for idx, value in enumerate(rest_positions):
                self.last_sent[idx] = int(clamp(value, 0, 180))
            self.last_send_time = time.monotonic()
            self.state = "READY"
            self.rest_sent.emit()
        except Exception as exc:
            self.serial_error.emit(str(exc))
            self._close_current()

    @QtCore.pyqtSlot(int, int, bool)
    def send_angle(self, servo_id: int, angle: int, force: bool = False) -> None:
        if not self._is_open():
            self.serial_error.emit("Port serie non ouvert")
            return
        angle = int(clamp(angle, 0, 180))
        now = time.monotonic()
        if not force:
            if self.last_sent.get(servo_id) == angle:
                return
            if now - self.last_send_time < SERIAL_MIN_INTERVAL_S:
                return
        frame = f"S{servo_id}:{angle}\n"
        try:
            self._write_line(frame.strip())
            self.last_sent[servo_id] = angle
            self.last_send_time = now
            self.angle_sent.emit(servo_id, angle)
        except Exception as exc:
            # S{id}:{angle} has no Arduino ACK. Report the write failure, keep the
            # port open, and let the next command try again.
            self._log_serial(f"ERR TX {frame.strip()} {exc}")
            self.serial_error.emit(f"Commande {frame.strip()} non envoyee: {exc}")


    @QtCore.pyqtSlot()
    def close_serial(self) -> None:
        self.opening = False
        self._close_current()


class AlignmentRosNode(Node):
    def __init__(self, calibration: ServoCalibration, alignment: AlignmentStore) -> None:
        super().__init__("calibration_alignment_gui")
        self.calibration = calibration
        self.alignment = alignment
        self.selected_index = 0
        self.current_angles = [servo["rest"] for servo in calibration.servos]
        self.current_angle = calibration.servos[0]["rest"]
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_joint_states)

    def set_selected(self, servo_index: int, angle: float) -> None:
        self.selected_index = servo_index
        self.current_angle = angle
        self.current_angles[servo_index] = angle

    def set_angle(self, angle: float) -> None:
        self.current_angle = angle
        self.current_angles[self.selected_index] = angle

    def set_all_rest(self) -> None:
        self.current_angles = [servo["rest"] for servo in self.calibration.servos]
        self.current_angle = self.current_angles[self.selected_index]

    def servo_to_joint_rad(self, servo: dict, servo_angle: float) -> float:
        entry = self.alignment.entry(servo["name"])
        visual_sign = float(entry.get("visual_sign", 1.0))
        visual_scale = float(entry.get("visual_scale", 1.0))
        offset = float(entry.get("visual_offset_rad", 0.0))
        return math.radians((servo_angle - servo["rest"]) * visual_sign * visual_scale) + offset

    def publish_joint_states(self) -> None:
        names = self.calibration.joint_names
        positions = []
        for idx, servo in enumerate(self.calibration.servos):
            positions.append(self.servo_to_joint_rad(servo, self.current_angles[idx]))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.publisher.publish(msg)


class CalibrationAlignmentGui(QtWidgets.QWidget):
    send_rest_requested = QtCore.pyqtSignal(list)
    send_angle_requested = QtCore.pyqtSignal(int, int, bool)
    reconnect_requested = QtCore.pyqtSignal()
    close_serial_requested = QtCore.pyqtSignal()

    def __init__(
        self,
        calibration: ServoCalibration,
        alignment: AlignmentStore,
        ros_node: AlignmentRosNode,
        serial_worker: SerialWorker,
        auto_rviz: bool,
    ) -> None:
        super().__init__()
        self.calibration = calibration
        self.alignment = alignment
        self.ros_node = ros_node
        self.serial_worker = serial_worker
        self.current_servo_index = 0
        self._updating_widgets = False
        self.rest_synchronized = False
        self.rest_sync_in_progress = False
        self.connected = False
        self.active_port = ""
        self.auto_rviz = auto_rviz
        self.rviz_process: Optional[subprocess.Popen] = None
        self.owns_rviz_process = False
        self._shutting_down = False

        self.setWindowTitle("ASTER - Calibration alignement RViz")
        self.resize(900, 560)
        self._build_ui()
        self._set_controls_enabled(False)
        self._load_servo(0, send_rest=False)
        self._connect_serial_worker()
        self._check_rviz_status()
        if self.auto_rviz:
            QtCore.QTimer.singleShot(200, self._open_rviz)

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)

        self.status_label = QtWidgets.QLabel("Connexion Arduino...")
        root.addWidget(self.status_label)

        connection_row = QtWidgets.QHBoxLayout()
        self.connection_label = QtWidgets.QLabel("Arduino: deconnecte")
        self.port_label = QtWidgets.QLabel("Port: -")
        self.disconnect_button = QtWidgets.QPushButton("Deconnecter Arduino")
        self.reconnect_button = QtWidgets.QPushButton("Reconnecter Arduino")
        self.disconnect_button.clicked.connect(self._disconnect_arduino)
        self.reconnect_button.clicked.connect(self._reconnect_arduino)
        connection_row.addWidget(self.connection_label)
        connection_row.addWidget(self.port_label)
        connection_row.addStretch(1)
        connection_row.addWidget(self.disconnect_button)
        connection_row.addWidget(self.reconnect_button)
        root.addLayout(connection_row)

        rviz_row = QtWidgets.QHBoxLayout()
        self.rviz_status_label = QtWidgets.QLabel("RViz: verification...")
        self.open_rviz_button = QtWidgets.QPushButton("Ouvrir RViz")
        self.close_rviz_button = QtWidgets.QPushButton("Fermer RViz")
        self.open_rviz_button.clicked.connect(self._open_rviz)
        self.close_rviz_button.clicked.connect(self._close_rviz)
        rviz_row.addWidget(self.rviz_status_label)
        rviz_row.addStretch(1)
        rviz_row.addWidget(self.open_rviz_button)
        rviz_row.addWidget(self.close_rviz_button)
        root.addLayout(rviz_row)

        selector_row = QtWidgets.QHBoxLayout()
        selector_row.addWidget(QtWidgets.QLabel("Servo"))
        self.servo_combo = QtWidgets.QComboBox()
        for servo in self.calibration.servos:
            self.servo_combo.addItem(f"{servo['index']:02d} - {servo['name']}", servo["index"])
        self.servo_combo.currentIndexChanged.connect(self._on_servo_changed)
        selector_row.addWidget(self.servo_combo, 1)
        root.addLayout(selector_row)

        self.info_label = QtWidgets.QLabel("")
        root.addWidget(self.info_label)

        real_group = QtWidgets.QGroupBox("ZONE 1 - Calibration du reel")
        real_layout = QtWidgets.QGridLayout(real_group)
        self.real_min_button = QtWidgets.QPushButton("Aller MIN reel")
        self.real_rest_button = QtWidgets.QPushButton("Aller REPOS reel")
        self.real_max_button = QtWidgets.QPushButton("Aller MAX reel")
        self.step_minus_button = QtWidgets.QPushButton("-5 deg")
        self.step_plus_button = QtWidgets.QPushButton("+5 deg")
        self.set_min_button = QtWidgets.QPushButton("Definir actuel = MIN reel")
        self.set_rest_button = QtWidgets.QPushButton("Definir actuel = REPOS reel")
        self.set_max_button = QtWidgets.QPushButton("Definir actuel = MAX reel")
        self.save_calibration_button = QtWidgets.QPushButton("Sauvegarder servo_calibration.json")

        self.real_min_button.clicked.connect(lambda: self._set_real_angle_to_key("min"))
        self.real_rest_button.clicked.connect(lambda: self._set_real_angle_to_key("rest"))
        self.real_max_button.clicked.connect(lambda: self._set_real_angle_to_key("max"))
        self.step_minus_button.clicked.connect(lambda: self._step_real_angle(-5))
        self.step_plus_button.clicked.connect(lambda: self._step_real_angle(+5))
        self.set_min_button.clicked.connect(lambda: self._define_current_as_mechanical("min"))
        self.set_rest_button.clicked.connect(lambda: self._define_current_as_mechanical("rest"))
        self.set_max_button.clicked.connect(lambda: self._define_current_as_mechanical("max"))
        self.save_calibration_button.clicked.connect(self._save_physical_calibration)

        real_layout.addWidget(self.real_min_button, 0, 0)
        real_layout.addWidget(self.real_rest_button, 0, 1)
        real_layout.addWidget(self.real_max_button, 0, 2)
        real_layout.addWidget(self.step_minus_button, 1, 0)
        real_layout.addWidget(self.step_plus_button, 1, 1)
        real_layout.addWidget(self.set_min_button, 2, 0)
        real_layout.addWidget(self.set_rest_button, 2, 1)
        real_layout.addWidget(self.set_max_button, 2, 2)

        self.angle_label = QtWidgets.QLabel("Angle reel: 0 deg")
        self.angle_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angle_slider.setRange(0, 180)
        self.angle_slider.valueChanged.connect(self._on_real_angle_changed)
        self.angle_slider.sliderReleased.connect(self._send_current_real_angle_force)
        real_layout.addWidget(self.angle_label, 3, 0)
        real_layout.addWidget(self.angle_slider, 3, 1, 1, 2)
        real_layout.addWidget(self.save_calibration_button, 4, 2)
        root.addWidget(real_group)

        align_group = QtWidgets.QGroupBox("ZONE 2 - Alignement RViz sur le reel")
        align_layout = QtWidgets.QGridLayout(align_group)
        self.rviz_min_button = QtWidgets.QPushButton("Afficher RViz MIN")
        self.rviz_rest_button = QtWidgets.QPushButton("Afficher RViz REPOS")
        self.rviz_max_button = QtWidgets.QPushButton("Afficher RViz MAX")
        self.rviz_min_button.clicked.connect(lambda: self._set_rviz_angle_to_key("min"))
        self.rviz_rest_button.clicked.connect(lambda: self._set_rviz_angle_to_key("rest"))
        self.rviz_max_button.clicked.connect(lambda: self._set_rviz_angle_to_key("max"))
        align_layout.addWidget(self.rviz_min_button, 0, 0)
        align_layout.addWidget(self.rviz_rest_button, 0, 1)
        align_layout.addWidget(self.rviz_max_button, 0, 2)

        self.normal_button = QtWidgets.QPushButton("RViz normal")
        self.invert_button = QtWidgets.QPushButton("RViz inverse")
        self.normal_button.clicked.connect(lambda: self._set_visual_sign(+1.0))
        self.invert_button.clicked.connect(lambda: self._set_visual_sign(-1.0))
        align_layout.addWidget(self.normal_button, 1, 0)
        align_layout.addWidget(self.invert_button, 1, 1)

        align_layout.addWidget(QtWidgets.QLabel("Scale RViz"), 2, 0)
        self.scale_spin = QtWidgets.QDoubleSpinBox()
        self.scale_spin.setRange(0.05, 5.0)
        self.scale_spin.setDecimals(3)
        self.scale_spin.setSingleStep(0.05)
        self.scale_spin.valueChanged.connect(self._on_scale_changed)
        align_layout.addWidget(self.scale_spin, 2, 1)

        align_layout.addWidget(QtWidgets.QLabel("Offset RViz (deg)"), 3, 0)
        self.offset_spin = QtWidgets.QDoubleSpinBox()
        self.offset_spin.setRange(-180.0, 180.0)
        self.offset_spin.setDecimals(2)
        self.offset_spin.setSingleStep(1.0)
        self.offset_spin.valueChanged.connect(self._on_offset_changed)
        align_layout.addWidget(self.offset_spin, 3, 1)

        align_layout.addWidget(QtWidgets.QLabel("Note"), 4, 0)
        self.note_edit = QtWidgets.QLineEdit()
        self.note_edit.setPlaceholderText("Validation physique du mouvement reel")
        align_layout.addWidget(self.note_edit, 4, 1, 1, 2)

        self.save_button = QtWidgets.QPushButton("Sauvegarder ros_rviz_alignment.json")
        self.save_button.clicked.connect(self._save_alignment)
        align_layout.addWidget(self.save_button, 5, 2)
        root.addWidget(align_group)

    def _check_rviz_status(self) -> None:
        launch_pids = rviz_launch_pids()
        if rviz_process_is_running():
            self.rviz_status_label.setText("RViz: deja actif")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
        elif launch_pids:
            self.rviz_status_label.setText(f"RViz: launch actif sans fenetre visible PID(s): {', '.join(launch_pids)}")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
        else:
            self.rviz_status_label.setText("RViz: arrete")
            self.open_rviz_button.setEnabled(True)
            self.close_rviz_button.setEnabled(False)

    def _open_rviz(self) -> None:
        self._check_rviz_status()
        if rviz_process_is_running():
            self.rviz_status_label.setText("RViz deja actif")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
            return
        launch_pids = rviz_launch_pids()
        if launch_pids:
            self.rviz_status_label.setText(f"RViz launch deja actif PID(s): {', '.join(launch_pids)}")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
            return

        env = os.environ.copy()
        env["DISPLAY"] = ":0"
        try:
            log_file = RVIZ_LOG_PATH.open("ab")
            log_file.write(b"\n===== ASTER RViz launch =====\n")
            log_file.flush()
            self.rviz_process = subprocess.Popen(
                ["bash", "-lc", RVIZ_LAUNCH_CMD],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            self.owns_rviz_process = True
            self.rviz_status_label.setText("RViz lance")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
            QtCore.QTimer.singleShot(2000, self._verify_rviz_started)
        except Exception:
            self.rviz_process = None
            self.owns_rviz_process = False
            self.rviz_status_label.setText("Erreur lancement RViz, voir log")
            self.open_rviz_button.setEnabled(True)
            self.close_rviz_button.setEnabled(bool(rviz_launch_pids()))

    def _verify_rviz_started(self) -> None:
        if rviz_process_is_running() or rviz_launch_pids():
            if rviz_process_is_running():
                self.rviz_status_label.setText("RViz lance")
            else:
                self.rviz_status_label.setText("RViz launch actif")
            self.open_rviz_button.setEnabled(False)
            self.close_rviz_button.setEnabled(True)
            return
        self.rviz_status_label.setText("Erreur lancement RViz, voir log")
        self.open_rviz_button.setEnabled(True)
        self.close_rviz_button.setEnabled(False)

    def _close_rviz(self) -> None:
        if self.rviz_process is not None and self.owns_rviz_process and self.rviz_process.poll() is None:
            self.rviz_status_label.setText("RViz: fermeture du launch GUI...")
            try:
                os.killpg(self.rviz_process.pid, signal.SIGTERM)
                self.rviz_process.wait(timeout=1.5)
            except Exception:
                try:
                    os.killpg(self.rviz_process.pid, signal.SIGKILL)
                except Exception:
                    pass
            self.rviz_process = None
            self.owns_rviz_process = False
        else:
            pids = rviz_launch_pids()
            if pids:
                self.rviz_status_label.setText(f"RViz: fermeture launch fantome PID(s): {', '.join(pids)}")
                stop_rviz_launch_processes()
        QtCore.QTimer.singleShot(500, self._check_rviz_status)


    def _connect_serial_worker(self) -> None:
        self.send_rest_requested.connect(self.serial_worker.send_rest_frame, QtCore.Qt.QueuedConnection)
        self.send_angle_requested.connect(self.serial_worker.send_angle, QtCore.Qt.QueuedConnection)
        self.reconnect_requested.connect(self.serial_worker.open_serial, QtCore.Qt.QueuedConnection)
        self.close_serial_requested.connect(self.serial_worker.close_serial, QtCore.Qt.QueuedConnection)
        self.serial_worker.status.connect(self.status_label.setText)
        self.serial_worker.connected.connect(self._on_serial_connected)
        self.serial_worker.disconnected.connect(self._on_serial_disconnected)
        self.serial_worker.connection_failed.connect(self._on_serial_failed)
        self.serial_worker.rest_sent.connect(self._on_rest_sent)
        self.serial_worker.serial_error.connect(self._on_serial_error)
        self.serial_worker.angle_sent.connect(self._on_angle_sent)

    def _set_controls_enabled(self, enabled: bool) -> None:
        widgets = [
            self.servo_combo,
            self.real_min_button,
            self.real_rest_button,
            self.real_max_button,
            self.step_minus_button,
            self.step_plus_button,
            self.set_min_button,
            self.set_rest_button,
            self.set_max_button,
            self.save_calibration_button,
            self.angle_slider,
            self.rviz_min_button,
            self.rviz_rest_button,
            self.rviz_max_button,
            self.normal_button,
            self.invert_button,
            self.scale_spin,
            self.offset_spin,
            self.note_edit,
            self.save_button,
        ]
        for widget in widgets:
            widget.setEnabled(enabled)
        self.disconnect_button.setEnabled(self.connected)
        self.reconnect_button.setEnabled(not self.connected and not self.rest_sync_in_progress)

    def _disconnect_arduino(self) -> None:
        self.status_label.setText("Deconnexion Arduino...")
        self.connected = False
        self.active_port = ""
        self.rest_synchronized = False
        self.rest_sync_in_progress = False
        self.connection_label.setText("Arduino: deconnecte")
        self.port_label.setText("Port: -")
        self._set_controls_enabled(False)
        self.reconnect_button.setEnabled(True)
        self.close_serial_requested.emit()

    def _reconnect_arduino(self) -> None:
        if self.connected or self.rest_sync_in_progress:
            return
        self.status_label.setText("Reconnexion Arduino...")
        self.rest_synchronized = False
        self.rest_sync_in_progress = True
        self._set_controls_enabled(False)
        self.reconnect_requested.emit()


    def _on_serial_connected(self, port: str) -> None:
        self.connected = True
        self.active_port = port
        self.connection_label.setText("Arduino: connecte")
        self.port_label.setText(f"Port: {port}")
        self.status_label.setText(f"Arduino pret sur {port}. Telemetrie OFF. Envoi repos JSON...")
        self.rest_sync_in_progress = True
        self._set_controls_enabled(False)
        QtCore.QTimer.singleShot(REST_SYNC_TIMEOUT_MS, self._on_rest_sync_timeout)
        self.send_rest_requested.emit(self.calibration.rest_positions_int())

    def _on_serial_disconnected(self, message: str) -> None:
        self.connected = False
        self.active_port = ""
        self.rest_synchronized = False
        self.rest_sync_in_progress = False
        self.connection_label.setText("Arduino: deconnecte")
        self.port_label.setText("Port: -")
        self.status_label.setText(message)
        self._set_controls_enabled(False)

    def _on_serial_failed(self, message: str) -> None:
        self.connected = False
        self.active_port = ""
        self.rest_sync_in_progress = False
        self.rest_synchronized = False
        self.connection_label.setText("Arduino: deconnecte")
        self.port_label.setText("Port: -")
        self._set_controls_enabled(False)
        self.status_label.setText(f"Connexion Arduino echouee: {message} | log: /tmp/aster_calibration_serial.log")

    def _on_rest_sent(self) -> None:
        self.rest_sync_in_progress = False
        self.rest_synchronized = True
        self.ros_node.set_all_rest()
        self._set_controls_enabled(True)
        self.status_label.setText("Pret: ASTER_READY recu, repos JSON envoye, RViz au repos")

    def _on_rest_sync_timeout(self) -> None:
        if not self.rest_sync_in_progress:
            return
        self.rest_sync_in_progress = False
        self.rest_synchronized = False
        self._set_controls_enabled(False)
        self.status_label.setText("Timeout: synchronisation repos Arduino non terminee")
        self.close_serial_requested.emit()

    def _on_serial_error(self, message: str) -> None:
        self.status_label.setText(f"Erreur serie: {message} | log: /tmp/aster_calibration_serial.log")

    def _on_angle_sent(self, servo_id: int, angle: int) -> None:
        self.status_label.setText(f"Envoye S{servo_id}:{angle}")

    def _current_servo(self) -> dict:
        return self.calibration.servos[self.current_servo_index]

    def _on_servo_changed(self, combo_index: int) -> None:
        if combo_index < 0:
            return
        servo_index = int(self.servo_combo.itemData(combo_index))
        self._load_servo(servo_index, send_rest=False)

    def _load_servo(self, servo_index: int, send_rest: bool) -> None:
        self.current_servo_index = servo_index
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        self._updating_widgets = True
        self.angle_slider.setRange(0, 180)
        self.angle_slider.setValue(int(round(servo["rest"])))
        self.scale_spin.setValue(float(entry.get("visual_scale", 1.0)))
        self.offset_spin.setValue(math.degrees(float(entry.get("visual_offset_rad", 0.0))))
        self.note_edit.setText(str(entry.get("note", "")))
        self._updating_widgets = False
        self._update_info_label()
        self.ros_node.set_selected(servo_index, float(self.angle_slider.value()))
        if send_rest:
            self._send_current_real_angle_force()

    def _update_info_label(self) -> None:
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        self.info_label.setText(
            f"REEL min={servo['min']:g} deg | repos={servo['rest']:g} deg | max={servo['max']:g} deg | "
            f"RViz sign={float(entry.get('visual_sign', 1.0)):+g} | "
            f"scale={float(entry.get('visual_scale', 1.0)):.3g} | "
            f"offset={math.degrees(float(entry.get('visual_offset_rad', 0.0))):+.2f} deg"
        )
        self.angle_label.setText(f"Angle reel: {self.angle_slider.value()} deg")

    def _set_real_angle_to_key(self, key: str) -> None:
        if not self.rest_synchronized:
            return
        servo = self._current_servo()
        value = int(round(servo[key]))
        self._set_real_angle(value, force=True)

    def _step_real_angle(self, delta: int) -> None:
        if not self.rest_synchronized:
            return
        self._set_real_angle(int(self.angle_slider.value()) + int(delta), force=True)

    def _set_real_angle(self, value: int, force: bool) -> None:
        servo = self._current_servo()
        value = int(clamp(value, 0, 180))
        self._updating_widgets = True
        self.angle_slider.setValue(value)
        self._updating_widgets = False
        self.angle_label.setText(f"Angle reel: {value} deg")
        self.ros_node.set_selected(servo["index"], float(value))
        self.send_angle_requested.emit(servo["index"], value, force)
        self.status_label.setText(f"Envoi reel S{servo['index']}:{value}")

    def _on_real_angle_changed(self, value: int) -> None:
        if self._updating_widgets or not self.rest_synchronized:
            return
        servo = self._current_servo()
        self.angle_label.setText(f"Angle reel: {value} deg")
        self.ros_node.set_selected(servo["index"], float(value))
        self.send_angle_requested.emit(servo["index"], value, False)
        self.status_label.setText(f"Demande reel S{servo['index']}:{value} | RViz suit la position reelle")

    def _send_current_real_angle_force(self) -> None:
        if not self.rest_synchronized:
            return
        servo = self._current_servo()
        value = int(self.angle_slider.value())
        self.ros_node.set_selected(servo["index"], float(value))
        self.send_angle_requested.emit(servo["index"], value, True)
        self.status_label.setText(f"Envoi reel S{servo['index']}:{value}")

    def _define_current_as_mechanical(self, key: str) -> None:
        servo = self._current_servo()
        value = float(self.angle_slider.value())
        try:
            self.calibration.set_mechanical_value(servo["index"], key, value)
        except Exception as exc:
            self.status_label.setText(f"Calibration reelle refusee: {exc}")
            return
        self.ros_node.set_selected(servo["index"], value)
        self._update_info_label()
        label = {"min": "MIN", "max": "MAX", "rest": "REPOS"}[key]
        self.status_label.setText(f"{servo['name']}: position actuelle definie comme {label} reel ({value:g} deg)")

    def _save_physical_calibration(self) -> None:
        try:
            self.calibration.save()
            self.ros_node.set_all_rest()
            self.status_label.setText(f"Calibration reelle sauvegardee: {self.calibration.path}")
            self._update_info_label()
        except Exception as exc:
            self.status_label.setText(f"Erreur sauvegarde calibration reelle: {exc}")

    def _set_rviz_angle_to_key(self, key: str) -> None:
        servo = self._current_servo()
        value = float(servo[key])
        self.ros_node.set_selected(servo["index"], value)
        self.status_label.setText(f"RViz uniquement: affichage {servo['name']} {key}={value:g} deg")

    def _set_visual_sign(self, sign: float) -> None:
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        entry["visual_sign"] = float(sign)
        self.ros_node.set_selected(servo["index"], float(self.angle_slider.value()))
        self._update_info_label()
        self.status_label.setText(f"Correction RViz appliquee: {servo['name']} visual_sign={sign:+g}")

    def _on_scale_changed(self, value: float) -> None:
        if self._updating_widgets:
            return
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        entry["visual_scale"] = float(value)
        self.ros_node.set_selected(servo["index"], float(self.angle_slider.value()))
        self._update_info_label()

    def _on_offset_changed(self, value_deg: float) -> None:
        if self._updating_widgets:
            return
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        entry["visual_offset_rad"] = math.radians(float(value_deg))
        self.ros_node.set_selected(servo["index"], float(self.angle_slider.value()))
        self._update_info_label()

    def _save_alignment(self) -> None:
        servo = self._current_servo()
        entry = self.alignment.entry(servo["name"])
        visual_sign = float(entry.get("visual_sign", 1.0))
        visual_scale = float(self.scale_spin.value())
        visual_offset_rad = math.radians(float(self.offset_spin.value()))
        note = self.note_edit.text().strip()
        self.alignment.set_entry(servo["name"], visual_sign, visual_scale, visual_offset_rad, note)
        try:
            self.alignment.save()
            self.status_label.setText(f"Alignement sauvegarde: {self.alignment.path}")
            self._update_info_label()
        except Exception as exc:
            self.status_label.setText(f"Erreur sauvegarde: {exc}")

    def shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        self.status_label.setText("Fermeture: liberation du port Arduino...")
        self.close_serial_requested.emit()
        if self.rviz_process is not None and self.owns_rviz_process and self.rviz_process.poll() is None:
            try:
                os.killpg(self.rviz_process.pid, signal.SIGTERM)
                self.rviz_process.wait(timeout=1.0)
            except Exception:
                try:
                    os.killpg(self.rviz_process.pid, signal.SIGKILL)
                except Exception:
                    pass
            self.rviz_process = None
            self.owns_rviz_process = False

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.shutdown()
        super().closeEvent(event)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ASTER real-servo calibration and RViz alignment GUI")
    parser.add_argument("--calibration", default=str(CALIB_PATH), help="Path to servo_calibration.json")
    parser.add_argument("--alignment", default=str(ALIGNMENT_PATH), help="Path to ros_rviz_alignment.json")
    parser.add_argument("--no-rviz", action="store_true", help="Do not auto-launch RViz")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if QtWidgets is None or QtCore is None:
        print("PyQt5 is required to run calibration_alignment_gui.py", file=sys.stderr)
        return 1
    if serial is None:
        print("pyserial is required to run calibration_alignment_gui.py", file=sys.stderr)
        return 1
    if bridge_process_is_running():
        print(
            "SAFETY REFUSAL: ros2_serial_bridge.py is running. "
            "Stop the bridge before starting calibration alignment.",
            file=sys.stderr,
        )
        return 2

    calibration = ServoCalibration(Path(args.calibration))
    alignment = AlignmentStore(Path(args.alignment), calibration)
    if not Path(args.alignment).exists():
        alignment.save()

    rclpy.init()
    app = QtWidgets.QApplication([sys.argv[0]])
    node = AlignmentRosNode(calibration, alignment)

    serial_thread = QtCore.QThread()
    serial_worker = SerialWorker(SERIAL_PORTS, BAUDRATE)
    serial_worker.moveToThread(serial_thread)
    serial_thread.started.connect(serial_worker.open_serial)

    gui = CalibrationAlignmentGui(calibration, alignment, node, serial_worker, auto_rviz=not args.no_rviz)
    gui.show()

    def _quit_from_signal(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, _quit_from_signal)
    signal.signal(signal.SIGTERM, _quit_from_signal)

    serial_thread.start()

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    try:
        return_code = app.exec_()
    finally:
        gui.shutdown()
        if serial_thread.isRunning():
            QtCore.QMetaObject.invokeMethod(serial_worker, "close_serial", QtCore.Qt.BlockingQueuedConnection)
        serial_thread.quit()
        serial_thread.wait(2000)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
