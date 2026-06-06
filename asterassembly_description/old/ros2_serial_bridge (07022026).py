import math
import json
from pathlib import Path
from typing import Dict, List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import serial  # pyserial


# ========= PARAMÈTRES GÉNÉRAUX =========

# Détection automatique du chemin (là où est le script)
SCRIPT_DIR = Path(__file__).parent.absolute()
CALIB_PATH = SCRIPT_DIR / "servo_calibration.json"

# On essaie d'abord ACM0, sinon ACM1 (Sécurité)
SERIAL_PORT = "/dev/ttyACM0" 
if not Path(SERIAL_PORT).exists():
    SERIAL_PORT = "/dev/ttyACM1"

BAUDRATE = 115200

BRIDGE_DT = 0.05              # 20 Hz vers Arduino
MAX_SPEED_DEG_S = 15.0        # vitesse max de variation (°/s) vers le servo
CHANGE_THRESH_DEG = 1.0       # pas d’envoi si variation < 1°
SCALE_JOINT_DEG = 1.0         # échelle appliquée aux angles (1.0 = 100% de la simu)


class AsterSerialBridge(Node):
    """
    Bridge ROS2 -> Arduino :
    - lit /joint_states (ex: quasistatic_walker)
    - utilise le JSON de calibration :
        - name
        - low_mech_constraint
        - high_mech_constraint
        - rest_position
    - convertit les angles URDF (rad autour de 0) en angles servo (deg)
      autour de rest_position, en respectant les contraintes méca.
    - règle de signe :
        * joints contenant "droit"  : URDF + => servo augmente (min -> max)
        * joints contenant "gauche" : URDF + => servo diminue (max -> min)
    """

    def __init__(self) -> None:
        super().__init__("aster_serial_bridge")

        # Ces tableaux seront remplis depuis le JSON
        self.joint_names: List[str] = []
        self.servo_rest: np.ndarray
        self.servo_min: np.ndarray
        self.servo_max: np.ndarray
        self.index_from_name: Dict[str, int] = {}

        self._load_calibration_from_json()
        self.num_servos = len(self.servo_rest)

        # États internes (en degrés servo)
        self.current_cmd = self.servo_rest.copy()   # ce qu’on envoie réellement
        self.target_cmd = self.servo_rest.copy()    # cible calculée depuis /joint_states
        self.last_sent_cmd = self.servo_rest.copy()

        self.max_delta = MAX_SPEED_DEG_S * BRIDGE_DT

        # Port série
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.01
            )
            self.get_logger().info(f"Port série ouvert sur {SERIAL_PORT} @ {BAUDRATE}")
        except serial.SerialException as e:
            self.get_logger().error(f"Impossible d’ouvrir le port série {SERIAL_PORT}: {e}")
            self.ser = None

        # Sub /joint_states
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            10
        )

        # Timer périodique
        self.timer = self.create_timer(BRIDGE_DT, self.timer_cb)

        self.get_logger().info(
            f"AsterSerialBridge démarré : {self.num_servos} servos, "
            f"limites JSON respectées, SCALE_JOINT_DEG={SCALE_JOINT_DEG}"
        )

    # ========= Lecture calibration JSON =========

    def _load_calibration_from_json(self) -> None:
        if not CALIB_PATH.exists():
            self.get_logger().warn(
                f"{CALIB_PATH} introuvable. "
                "Calibration par défaut (16 servos, 0–180, repos=90)."
            )
            self._create_default_calibration()
            return

        try:
            with CALIB_PATH.open("r", encoding="utf-8") as f:
                calib = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Erreur lecture JSON calibration: {e}")
            self._create_default_calibration()
            return

        servos_raw = calib.get("servos", None)
        if servos_raw is None:
            self.get_logger().error("JSON ne contient pas 'servos'.")
            self._create_default_calibration()
            return

        if isinstance(servos_raw, list):
            servos_list = servos_raw
        elif isinstance(servos_raw, dict):
            try:
                items = sorted(servos_raw.items(), key=lambda kv: int(kv[0]))
            except Exception:
                items = list(servos_raw.items())
            servos_list = [v for _, v in items]
        else:
            self.get_logger().error("Format 'servos' invalide dans le JSON.")
            self._create_default_calibration()
            return

        num = len(servos_list)
        if num == 0:
            self.get_logger().error("Liste 'servos' vide dans le JSON.")
            self._create_default_calibration()
            return

        self.servo_rest = np.zeros(num)
        self.servo_min  = np.zeros(num)
        self.servo_max  = np.zeros(num)
        self.joint_names = []
        self.index_from_name = {}

        for idx, servo in enumerate(servos_list):
            if not isinstance(servo, dict):
                self.get_logger().warn(f"entrée servo[{idx}] invalide: {servo}")
                continue

            name = servo.get("name", f"servo_{idx}")
            low  = float(servo.get("low_mech_constraint", 0))
            high = float(servo.get("high_mech_constraint", 180))
            rest = float(servo.get("rest_position", 90))

            # On garantit low <= rest <= high
            if rest < low:
                self.get_logger().warn(
                    f"{name}: rest_position {rest} < low {low}, on remonte rest à low."
                )
                rest = low
            if rest > high:
                self.get_logger().warn(
                    f"{name}: rest_position {rest} > high {high}, on descend rest à high."
                )
                rest = high

            self.joint_names.append(name)
            self.index_from_name[name] = idx
            self.servo_rest[idx] = rest
            self.servo_min[idx]  = low
            self.servo_max[idx]  = high

        self.get_logger().info(
            f"Calibration (bridge) chargée : {num} servos depuis {CALIB_PATH}"
        )

    def _create_default_calibration(self) -> None:
        num = 16
        self.servo_rest = np.full(num, 90.0)
        self.servo_min  = np.zeros(num)
        self.servo_max  = np.full(num, 180.0)
        self.joint_names = [f"servo_{i}" for i in range(num)]
        self.index_from_name = {name: i for i, name in enumerate(self.joint_names)}

    # ========= Conversion joint (rad) -> servo (deg) =========

    def joint_to_servo_deg(self, joint_name: str, joint_angle_rad: float) -> float:
        """
        Convertit un angle URDF (rad autour de 0) en angle servo (deg) autour de rest_position,
        en respectant les contraintes mécaniques JSON.

        Convention mécanique :
          - joints *droit*  : angle URDF positif -> servo va du min vers le max  (sens +)
          - joints *gauche* : angle URDF positif -> servo va du max vers le min  (sens -)
        """

        if joint_name not in self.index_from_name:
            return 90.0  # fallback safe

        idx = self.index_from_name[joint_name]
        rest = self.servo_rest[idx]
        low  = self.servo_min[idx]
        high = self.servo_max[idx]

        # 1) angle URDF (rad) -> degrés (avec réduction éventuelle d’amplitude)
        joint_deg = math.degrees(joint_angle_rad) * SCALE_JOINT_DEG

        # 2) signe auto en fonction du côté (nom du joint)
        name = joint_name.lower()
        if "gauche" in name:
            s = -1.0    # côté gauche : URDF + => servo diminue
        elif "droit" in name:
            s = +1.0    # côté droit : URDF + => servo augmente
        else:
            s = 1.0     # par défaut, pas d’inversion (tête, etc.)

        # 3) mouvement autour de la position de repos
        servo_deg = rest + s * joint_deg

        # 4) clamp strict aux contraintes méca JSON
        if servo_deg < low:
            servo_deg = low
        if servo_deg > high:
            servo_deg = high

        return servo_deg

    # ========= Callback /joint_states =========

    def joint_cb(self, msg: JointState) -> None:
        target = self.servo_rest.copy()  # base = pose de repos

        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        for joint_name, servo_idx in self.index_from_name.items():
            if joint_name not in name_to_idx:
                continue
            j_idx = name_to_idx[joint_name]
            if j_idx >= len(msg.position):
                continue

            angle_rad = msg.position[j_idx]
            servo_deg = self.joint_to_servo_deg(joint_name, angle_rad)
            target[servo_idx] = servo_deg

        self.target_cmd = target

    # ========= Timer : lissage + envoi série =========

    def timer_cb(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return

        # lissage : limitation de vitesse
        delta = self.target_cmd - self.current_cmd
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        self.current_cmd += delta

        cmd_int = np.round(self.current_cmd).astype(int)

        # anti-spam : n’envoie pas si quasi pas de changement
        if np.all(np.abs(cmd_int - self.last_sent_cmd) < CHANGE_THRESH_DEG):
            return

        values = [str(int(v)) for v in cmd_int.tolist()]
        frame = ";".join(values) + "\n"

        try:
            self.ser.write(frame.encode("ascii"))
            self.last_sent_cmd = cmd_int.copy()
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur écriture série: {e}")

    def destroy_node(self):
        try:
            if self.ser is not None and self.ser.is_open:
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
        pass # Ignore l'erreur qui fait crasher le bridge
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


