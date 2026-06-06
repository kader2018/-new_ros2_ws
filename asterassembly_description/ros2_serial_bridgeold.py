"""
ASTER Serial Bridge — V4 (ROS2 Jazzy)

Nouveautés vs V3 :
  - Trame capteurs étendue : "S;dist;ir;iaID;iaX;iaY;temp;roll;pitch"
  - Publication /aster/imu  (std_msgs/String JSON {"roll":X,"pitch":Y})
  - Correcteur d'équilibre intégré :
      • Reçoit roll/pitch depuis Arduino (MPU6050)
      • Calcule une correction PD sur bras_*_h_b_joint (hanches frontales)
        et publie un /joint_states correctif superposé à la cinématique
      • Gain Kp / Kd configurables via constantes
  - Commande RESET_IMU envoyée à l'Arduino si demandée via topic /aster/cmd
"""

import math
import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32, String

import serial


# ─── Paramètres ──────────────────────────────────────────────────────────────

SCRIPT_DIR  = Path(__file__).parent.absolute()
CALIB_PATH  = SCRIPT_DIR / "servo_calibration.json"

SERIAL_PORTS      = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUDRATE          = 115_200
BRIDGE_DT         = 0.02           # 20 Hz
MAX_SPEED_DEG_S   = 120.0
CHANGE_THRESH_DEG = 0.0
SCALE_JOINT_DEG   = 1.0
OBSTACLE_DIST_MM  = 250

# ── Correcteur d'équilibre PD (roll → hanches frontales) ─────────────────────
# Kp : correction proportionnelle — augmenter si le robot oscille trop lentement
# Kd : amortissement dérivé      — augmenter si la correction oscille
BALANCE_KP     = 0.6    # deg de correction hanche / deg de roll
BALANCE_KD     = 0.1    # amortissement
BALANCE_CLAMP  = 15.0   # correction max en degrés (évite les mouvements brusques)


# ─── Table de signes explicite ────────────────────────────────────────────────
JOINT_SIGN_TABLE: Dict[str, float] = {
    "bras_droit_de_ar_joint":   +1.0,
    "bras_droit_h_b_joint":     +1.0,
    "bras_droit_rot_joint":     +1.0,
    "bras_droit_coud_joint":    +1.0,
    "bras_gauche_de_ar_joint":  -1.0,
    "bras_gauche_h_b_joint":    -1.0,
    "bras_gauche_rot_joint":    -1.0,
    "bras_gauche_coude_joint":  -1.0,
    "cuisse_droit_joint":       +1.0,
    "genou_droit_joint":        +1.0,
    "cheville_droite_joint":    +1.0,
    "cuisse_gauche_joint":      -1.0,
    "genou_gauche_joint":       -1.0,
    "cheville_gauche_joint":    -1.0,
    "tete_g_d_joint":           +1.0,
    "tete_h_b_joint":           +1.0,
}


    def _open_serial() -> Optional[serial.Serial]:
        f# Modifie cette boucle de connexion dans ros2_serial_bridge.py
        for port in SERIAL_PORTS:
            try:
                self.get_logger().info(f"Tentative d'ouverture de {port}...")
                self.ser = serial.Serial(port, BAUDRATE, timeout=0.1)
                self.get_logger().info(f"✅ Connecté sur {port}")
                break
            except serial.SerialException as e:
                    # ─── AJOUTE CETTE LIGNE DE LOG ICI ───
                self.get_logger().warn(f"⚠️ Échec sur {port} : {str(e)}")
                continue
        return None


# ─── Nœud ────────────────────────────────────────────────────────────────────

class AsterSerialBridge(Node):
    """
    Bridge ROS2 ↔ Arduino bidirectionnel V4.

    SORTIE  ROS → Arduino
    ──────────────────────
    /joint_states → "v0;v1;...;v15\\n"

    ENTRÉE  Arduino → ROS
    ──────────────────────
    Trame : "S;dist;ir;iaID;iaX;iaY;temp;roll;pitch"

    Topics publiés :
      /aster/distance_mm   Int32
      /aster/ir_state      Int32
      /aster/vision        String  JSON
      /aster/temperature   Float32
      /aster/obstacle      Int32
      /aster/imu           String  JSON {"roll":X,"pitch":Y}

    Correcteur d'équilibre :
      roll  → correction différentielle sur bras_*_h_b_joint
              (incline le buste du côté opposé à la chute)
      pitch → non utilisé en correcteur pour l'instant (sert au monitoring)

    Topic entrant :
      /aster/cmd  String  "RESET_IMU" → recalibre le gyroscope Arduino
    """

    def __init__(self) -> None:
        super().__init__("aster_serial_bridge")
       
        self.joint_names:     List[str]      = []
        self.servo_rest:      np.ndarray
        self.servo_min:       np.ndarray
        self.servo_max:       np.ndarray
        self.index_from_name: Dict[str, int] = {}

        self._load_calibration()
        n = len(self.servo_rest)

        self.current_cmd   = self.servo_rest.copy()
        self.target_cmd    = self.servo_rest.copy()
        self.last_sent_cmd = self.servo_rest.copy()
        self.max_delta     = MAX_SPEED_DEG_S * BRIDGE_DT
        
        # Correcteur PD — état interne
        self._roll_prev  = 0.0
        self._roll_corr  = 0.0   # correction courante (degrés servo)

        # Buffer lecture série
        self._serial_buf: str = ""

        # Publishers
        self.pub_dist   = self.create_publisher(Int32,   "/aster/distance_mm", 10)
        self.pub_ir     = self.create_publisher(Int32,   "/aster/ir_state",    10)
        self.pub_vision = self.create_publisher(String,  "/aster/vision",      10)
        self.pub_temp   = self.create_publisher(Float32, "/aster/temperature", 10)
        self.pub_obs    = self.create_publisher(Int32,   "/aster/obstacle",    10)
        self.pub_imu    = self.create_publisher(String,  "/aster/imu",         10)

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(String,     "/aster/cmd",    self._cmd_cb,   10)

        # Port série
        self.ser = _open_serial()
        if self.ser:
            self.get_logger().info(f"Port série : {self.ser.port} @ {BAUDRATE}")
        else:
            self.get_logger().error(f"Aucun port série parmi {SERIAL_PORTS}")

        self.create_timer(BRIDGE_DT, self._timer_cb)
        self.get_logger().info(
            f"AsterSerialBridge V4 : {n} servos | IMU roll/pitch | correcteur PD"
        )

    # ── Calibration ──────────────────────────────────────────────────────────

    def _load_calibration(self) -> None:
        if not CALIB_PATH.exists():
            self.get_logger().warn("Calibration JSON introuvable — valeurs par défaut.")
            self._default_calibration()
            return
        try:
            with CALIB_PATH.open("r", encoding="utf-8") as f:
                calib = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Erreur JSON : {e}")
            self._default_calibration()
            return

        servos_raw = calib.get("servos")
        if not servos_raw:
            self._default_calibration()
            return

        if isinstance(servos_raw, dict):
            try:
                items = sorted(servos_raw.items(), key=lambda kv: int(kv[0]))
            except Exception:
                items = list(servos_raw.items())
            servos_list = [v for _, v in items]
        else:
            servos_list = servos_raw

        num = len(servos_list)
        self.servo_rest = np.zeros(num)
        self.servo_min  = np.zeros(num)
        self.servo_max  = np.full(num, 180.0)
        self.joint_names     = []
        self.index_from_name = {}

        for idx, s in enumerate(servos_list):
            if not isinstance(s, dict):
                continue
            name = s.get("name", f"servo_{idx}")
            low  = float(s.get("low_mech_constraint",   0))
            high = float(s.get("high_mech_constraint", 180))
            rest = float(s.get("rest_position",         90))
            rest = max(low, min(high, rest))
            self.joint_names.append(name)
            self.index_from_name[name] = idx
            self.servo_rest[idx] = rest
            self.servo_min[idx]  = low
            self.servo_max[idx]  = high

        self.get_logger().info(f"Calibration : {num} servos")
        self._check_consistency(calib)

    def _check_consistency(self, calib: dict) -> None:
        cp = calib.get("current_positions", [])
        if not cp or len(cp) != len(self.servo_rest):
            return
        for i, (exp, act) in enumerate(zip(cp, self.servo_rest)):
            if abs(exp - act) > 5:
                name = self.joint_names[i] if i < len(self.joint_names) else f"servo_{i}"
                self.get_logger().warn(
                    f"Incohérence servo {i} ({name}) : current={exp}° ≠ rest={act}°")

    def _default_calibration(self) -> None:
        num = 16
        self.servo_rest      = np.full(num, 90.0)
        self.servo_min       = np.zeros(num)
        self.servo_max       = np.full(num, 180.0)
        self.joint_names     = [f"servo_{i}" for i in range(num)]
        self.index_from_name = {n: i for i, n in enumerate(self.joint_names)}

    # ── Conversion joint → servo ──────────────────────────────────────────────

    def joint_to_servo_deg(self, joint_name: str, angle_rad: float) -> float:
        if joint_name not in self.index_from_name:
            return 90.0
        idx  = self.index_from_name[joint_name]
        sign = JOINT_SIGN_TABLE.get(joint_name, +1.0)
        val  = self.servo_rest[idx] + sign * math.degrees(angle_rad) * SCALE_JOINT_DEG
        return float(np.clip(val, self.servo_min[idx], self.servo_max[idx]))

    # ── Correcteur PD roll → hanches frontales ────────────────────────────────

    def _apply_balance_correction(self, roll: float) -> None:
        """
        Correction PD :
          roll > 0 → robot penche à droite → on lève le bras droit (h_b) et
                     abaisse le gauche pour déplacer le CoM vers la gauche.
          La correction est appliquée directement sur target_cmd APRÈS
          la mise à jour depuis /joint_states.

        Joints concernés :
          servo 1  = bras_droit_h_b_joint   (sign +1 → augmenter = lever)
          servo 5  = bras_gauche_h_b_joint  (sign -1 → augmenter URDF = servo diminue)
        """
        d_roll = roll - self._roll_prev
        self._roll_prev = roll

        # Correction brute PD
        corr = BALANCE_KP * roll + BALANCE_KD * (d_roll / BRIDGE_DT)
        corr = float(np.clip(corr, -BALANCE_CLAMP, BALANCE_CLAMP))
        self._roll_corr = corr

        # Indices dans target_cmd
        idx_r = self.index_from_name.get("bras_droit_h_b_joint")
        idx_l = self.index_from_name.get("bras_gauche_h_b_joint")

        if idx_r is not None:
            # roll+ → robot à droite → lever le bras droit (servo augmente)
            raw = self.target_cmd[idx_r] + corr
            self.target_cmd[idx_r] = float(
                np.clip(raw, self.servo_min[idx_r], self.servo_max[idx_r]))

        if idx_l is not None:
            # Symétrie : baisser le bras gauche (servo diminue car sign=-1)
            raw = self.target_cmd[idx_l] - corr
            self.target_cmd[idx_l] = float(
                np.clip(raw, self.servo_min[idx_l], self.servo_max[idx_l]))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        target      = self.servo_rest.copy()
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for joint_name, servo_idx in self.index_from_name.items():
            j_idx = name_to_idx.get(joint_name)
            if j_idx is None or j_idx >= len(msg.position):
                continue
            angle_rad = msg.position[j_idx]
            if not math.isfinite(angle_rad):
                continue
                # ─── AJOUT DE L'INVERSION DU BRAS DROIT ───
            if joint_name == "bras_gauche_h_b_joint":
                angle_rad = -angle_rad  # On inverse le sens pour qu'il s'écarte au lieu de rentrer en collision !
            # ──────────────────────────────────────────
            target[servo_idx] = self.joint_to_servo_deg(joint_name, angle_rad)
        self.target_cmd = target

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip()
        if cmd == "RESET_IMU" and self.ser and self.ser.is_open:
            try:
                self.ser.write(b"RESET_IMU\n")
                self.get_logger().info("RESET_IMU envoyé à l'Arduino")
            except serial.SerialException as e:
                self.get_logger().error(f"Erreur RESET_IMU : {e}")

    # ── Lecture série ─────────────────────────────────────────────────────────

    def _read_serial_lines(self) -> List[str]:
        if self.ser is None or not self.ser.is_open:
            return []
        lines = []
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._serial_buf += self.ser.read(waiting).decode("ascii", errors="ignore")
            while "\n" in self._serial_buf:
                line, self._serial_buf = self._serial_buf.split("\n", 1)
                line = line.strip()
                if line:
                    lines.append(line)
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur lecture : {e}")
        return lines

    def _parse_sensor_frame(self, line: str) -> None:
        """
        Parse "S;dist;ir;iaID;iaX;iaY;temp;roll;pitch"
        Les deux derniers champs (roll, pitch) sont optionnels pour
        rester compatible avec un Arduino sans MPU6050.
        """
        parts = line.split(";")
        if len(parts) < 7:
            return
        try:
            dist  = int(parts[1])
            ir    = int(parts[2])
            ia_id = int(parts[3])
            ia_x  = int(parts[4])
            ia_y  = int(parts[5])
            temp  = float(parts[6])
            roll  = float(parts[7]) if len(parts) > 7 else 0.0
            pitch = float(parts[8]) if len(parts) > 8 else 0.0
        except ValueError:
            self.get_logger().warn(f"Trame invalide : {line}")
            return

        # Distance
        m = Int32(); m.data = dist;  self.pub_dist.publish(m)

        # Obstacle
        m2 = Int32()
        m2.data = 1 if (dist != 999 and dist < OBSTACLE_DIST_MM) else 0
        self.pub_obs.publish(m2)

        # IR
        m3 = Int32(); m3.data = ir;  self.pub_ir.publish(m3)

        # Vision
        m4 = String()
        m4.data = json.dumps({"id": ia_id, "x": ia_x, "y": ia_y})
        self.pub_vision.publish(m4)

        # Température
        m5 = Float32(); m5.data = float(temp); self.pub_temp.publish(m5)

        # IMU
        m6 = String()
        m6.data = json.dumps({"roll": round(roll, 2), "pitch": round(pitch, 2)})
        self.pub_imu.publish(m6)

        # Correcteur d'équilibre (roll uniquement)
        self._apply_balance_correction(roll)

    # ── Timer principal ───────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        # Reconnexion auto
        if self.ser is None or not self.ser.is_open:
            self.ser = _open_serial()
            if self.ser:
                self.get_logger().info(f"Port rouvert : {self.ser.port}")
            else:
                return

        # Lecture + dispatch capteurs
        for line in self._read_serial_lines():
            if line.startswith("S;"):
                self._parse_sensor_frame(line)
            elif line == "ASTER_READY":
                self.get_logger().info("Arduino prêt -> Envoi de la calibration de démarrage !")
                # On force l'envoi des positions de repos dès que l'Arduino est réveillé
                cmd_int = np.round(self.servo_rest).astype(int)
                frame = "M;" + ";".join(str(int(v)) for v in cmd_int.tolist()) + "\n"
                try:
                    self.ser.write(frame.encode("ascii"))
                    self.last_sent_cmd = cmd_int.copy()
                except Exception as e:
                    self.get_logger().error(f"Échec envoi synchro : {e}")

        # Lissage + envoi servo
        delta = np.clip(
            self.target_cmd - self.current_cmd,
            -self.max_delta, self.max_delta)
        self.current_cmd += delta
        cmd_int = np.round(self.current_cmd).astype(int)

        if np.all(np.abs(cmd_int - self.last_sent_cmd) < CHANGE_THRESH_DEG):
            return

        frame = "M;" + ";".join(str(int(v)) for v in cmd_int.tolist()) + "\n"
        try:
            self.ser.write(frame.encode("ascii"))
            self.last_sent_cmd = cmd_int.copy()
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur écriture : {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsterSerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
