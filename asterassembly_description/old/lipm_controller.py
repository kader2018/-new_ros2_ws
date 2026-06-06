#!/usr/bin/env python3
import math
import json
from pathlib import Path
from typing import Dict, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# =========================
#  PARAMÈTRES GÉNÉRAUX
# =========================

# Géométrie des jambes (m)
L1 = 0.18      # longueur cuisse
L2 = 0.18      # longueur tibia
COM_HEIGHT = 0.36

# Paramètres de marche LENTE
STEP_LENGTH = 0.015   # 1.5 cm
STEP_HEIGHT = 0.01    # 1 cm
CYCLE_TIME = 4.0      # 4 s pour un cycle complet
DT = 0.02             # fréquence de contrôle 50 Hz

# Gravité (pour éventuellement du LIPM plus tard)
G = 9.81

# Chemin du JSON de calibration fourni par l'utilisateur
CALIB_PATH = Path("/home/ilyana/venv/aster/servo_calibration.json")


class ASTERGaitNode(Node):
    """
    Générateur de marche lente pour ASTER.
    - Produit des JointState cohérents avec l'URDF (pour RViz, etc.).
    - Calcule aussi les angles servos (en degrés) en utilisant :
        * le fichier JSON de calibration (repos + contraintes méca)
        * un signe par joint pour gérer gauche/droite inversés.
    """

    def __init__(self) -> None:
        super().__init__("aster_gait_node")

        # -------------------------------
        #  Chargement de la calibration
        # -------------------------------
        self.joint_names: List[str] = []
        self.servo_rest: np.ndarray
        self.servo_min: np.ndarray
        self.servo_max: np.ndarray
        self.index_from_name: Dict[str, int] = {}

        self._load_calibration_from_json()

        # Table de signe pour gérer gauche/droite
        # +1 : même sens que l'angle URDF
        # -1 : sens opposé (servo monté en miroir ou axe URDF inversé)
        self.servo_sign: Dict[str, float] = {
            # Jambes gauche
            "cuisse_gauche_joint":   +1.0,
            "genou_gauche_joint":    +1.0,
            "cheville_gauche_joint": +1.0,
            # Jambes droite (inversées)
            "cuisse_droit_joint":    -1.0,
            "genou_droit_joint":     +1.0,   # à ajuster si nécessaire
            "cheville_droite_joint": -1.0,
            # Bras / tête (par défaut +1, à ajuster après tests)
            "bras_droit_de_ar_joint": +1.0,
            "bras_droit_h_b_joint":   +1.0,
            "bras_droit_rot_joint":   +1.0,
            "bras_droit_coud_joint":  +1.0,
            "bras_gauche_de_ar_joint":+1.0,
            "bras_gauche_h_b_joint":  +1.0,
            "bras_gauche_rot_joint":  +1.0,
            "bras_gauche_coude_joint":+1.0,
            "tete_g_d_joint":         +1.0,
            "tete_h_b_joint":         +1.0,
        }

        # Indices utiles pour les jambes (si présents dans la calibration)
        self.has_left_leg = all(
            name in self.index_from_name
            for name in ("cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint")
        )
        self.has_right_leg = all(
            name in self.index_from_name
            for name in ("cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint")
        )

        if not (self.has_left_leg and self.has_right_leg):
            self.get_logger().warn(
                "Certains joints de jambe sont absents de la calibration JSON. "
                "La marche ne sera peut-être pas complète."
            )

        # -------------------------------
        #  État de marche
        # -------------------------------
        self.t = 0.0
        self.is_walking = False
        self.desired_linear_x = 0.0
        self.desired_yaw_rate = 0.0

        # Position du bassin / COM simple (pour TF + markers)
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0

        # -------------------------------
        #  ROS I/O
        # -------------------------------
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Markers pour debug (com / zmp / trajectoires pieds)
        self.zmp_pub = self.create_publisher(Marker, "/zmp_marker", 10)
        self.com_pub = self.create_publisher(Marker, "/com_marker", 10)
        self.left_foot_pub = self.create_publisher(Marker, "/left_foot_traj", 10)
        self.right_foot_pub = self.create_publisher(Marker, "/right_foot_traj", 10)

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer principal
        self.timer = self.create_timer(DT, self.timer_callback)

        self.get_logger().info("ASTERGaitNode initialisé (marche lente, calibration JSON prise en compte).")

    # ======================================================
    #  Calibration
    # ======================================================
    def _load_calibration_from_json(self) -> None:
        """
        Charge le fichier de calibration JSON donné par l'utilisateur.
        Supporte 2 structures :
        - "servos": [ {..}, {..}, ... ]
        - "servos": { "0":{..}, "1":{..}, ... }
        """
        if not CALIB_PATH.exists():
            self.get_logger().warn(
                f"Fichier de calibration {CALIB_PATH} introuvable. "
                "Utilisation calibration par défaut."
            )
            self._create_default_calibration()
            return

        try:
            with CALIB_PATH.open("r", encoding="utf-8") as f:
                calib = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Erreur lecture JSON: {e}")
            self._create_default_calibration()
            return

        servos_raw = calib.get("servos", None)
        if servos_raw is None:
            self.get_logger().error("JSON ne contient pas 'servos'")
            self._create_default_calibration()
            return

        # --- si c’est une liste (format facile)
        if isinstance(servos_raw, list):
            servos_list = servos_raw

        # --- sinon c’est un dict { "0":{...}, "1":{...} }
        elif isinstance(servos_raw, dict):
            try:
                # tri dans l'ordre numérique
                items = sorted(servos_raw.items(), key=lambda kv: int(kv[0]))
            except Exception:
                items = list(servos_raw.items())
            servos_list = [v for _, v in items]

        else:
            self.get_logger().error("Format JSON 'servos' invalide")
            self._create_default_calibration()
            return

        num_servos = len(servos_list)
        if num_servos == 0:
            self.get_logger().error("Liste 'servos' vide dans le JSON de calibration.")
            self._create_default_calibration()
            return

        self.servo_rest = np.zeros(num_servos)
        self.servo_min  = np.zeros(num_servos)
        self.servo_max  = np.zeros(num_servos)
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

            self.joint_names.append(name)
            self.index_from_name[name] = idx
            self.servo_rest[idx] = rest
            self.servo_min[idx] = low
            self.servo_max[idx] = high

        self.get_logger().info(f"Calibration chargée : {num_servos} servos trouvés.")

    def _create_default_calibration(self) -> None:
        """
        Calibration simple si le fichier JSON est absent :
        16 servos, 0–180°, repos=90°.
        """
        num_servos = 16
        self.servo_rest = np.full(num_servos, 90.0, dtype=float)
        self.servo_min = np.zeros(num_servos, dtype=float)
        self.servo_max = np.full(num_servos, 180.0, dtype=float)
        self.joint_names = [f"servo_{i}" for i in range(num_servos)]
        self.index_from_name = {name: i for i, name in enumerate(self.joint_names)}

    # ======================================================
    #  Callback /cmd_vel
    # ======================================================
    def cmd_vel_callback(self, msg: Twist) -> None:
        # On ne prend que la composante linéaire X et la rotation Yaw
        self.desired_linear_x = msg.linear.x
        self.desired_yaw_rate = msg.angular.z

        # Si la commande est quasi nulle -> on arrête la marche
        if abs(self.desired_linear_x) < 1e-3 and abs(self.desired_yaw_rate) < 1e-3:
            self.is_walking = False
        else:
            self.is_walking = True

    # ======================================================
    #  Génération de trajectoires pieds
    # ======================================================
    def foot_trajectory(self, phase: float, support_is_left: bool) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simple trajectoire de marche : 
        - un pied d'appui (support) reste au sol,
        - l'autre (swing) avance d'un demi-step puis revient.
        phase ∈ [0, 1[
        """
        # Position nominale des pieds par rapport au bassin
        # Décalage latéral approximatif
        hip_offset_y = 0.06  # 6 cm entre le COM et chaque hanche

        left_foot = np.array([0.0, +hip_offset_y, 0.0])
        right_foot = np.array([0.0, -hip_offset_y, 0.0])

        # On avance le pied swing sur X avec une trajectoire cycloïdale
        # phase_swing va de 0 à 1 pendant la moitié du cycle
        if support_is_left:
            # pied gauche au sol, pied droit en swing
            swing_foot = right_foot
            support_foot = left_foot
            swing_sign = +1.0
        else:
            # pied droit au sol, pied gauche en swing
            swing_foot = left_foot
            support_foot = right_foot
            swing_sign = -1.0

        # fraction du cycle où le pied swing se déplace
        # ici, moitié du cycle (0.0-0.5 montée/avancée, 0.5-1.0 retour)
        if phase < 0.5:
            s = phase / 0.5  # 0 -> 1
        else:
            s = (phase - 0.5) / 0.5  # 0 -> 1

        # déplacement avant / arrière
        # pied swing avance puis revient à sa position nominale
        step_x = swing_sign * STEP_LENGTH
        if phase < 0.5:
            # avance + montée
            dx = step_x * s
            dz = STEP_HEIGHT * math.sin(math.pi * s)
        else:
            # retour + descente
            dx = step_x * (1.0 - s)
            dz = STEP_HEIGHT * math.sin(math.pi * (1.0 - s))

        swing_foot_traj = swing_foot.copy()
        swing_foot_traj[0] += dx
        swing_foot_traj[2] += dz

        if support_is_left:
            left_traj = support_foot
            right_traj = swing_foot_traj
        else:
            left_traj = swing_foot_traj
            right_traj = support_foot

        return left_traj, right_traj

    # ======================================================
    #  Cinématique inverse jambe plan sagittal
    # ======================================================
    def ik_leg_sagittal(self, foot_pos: np.ndarray) -> Tuple[float, float, float]:
        """
        Cinématique inverse simplifiée dans le plan XZ.
        foot_pos : [x, y, z] dans le repère de la hanche.
        Retourne (hip, knee, ankle) en radians.
        """
        x = foot_pos[0]
        z = foot_pos[2]

        # distance hanche-pied
        d = math.sqrt(x * x + z * z)
        # évite les erreurs numériques
        d = max(min(d, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)

        # angle au genou (loi des cos)
        cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2)
        cos_knee = max(min(cos_knee, 1.0), -1.0)
        knee = math.acos(cos_knee)

        # angle entre cuisse et segment hanche-pied
        cos_alpha = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)
        cos_alpha = max(min(cos_alpha, 1.0), -1.0)
        alpha = math.acos(cos_alpha)

        # angle du segment hanche-pied
        phi = math.atan2(-z, x)

        hip = phi + alpha
        # cheville pour garder le pied à peu près horizontal
        ankle = -(hip + knee)

        return hip, knee, ankle

    # ======================================================
    #  Conversion joint -> servo (degrés)
    # ======================================================
    def joint_to_servo_deg(self, joint_name: str, joint_angle_rad: float) -> Tuple[float, int]:
        """
        Convertit un angle de joint URDF (en radians) en angle servo (en degrés)
        en utilisant :
        - la position de repos (servo_rest),
        - un signe par joint (servo_sign),
        - les contraintes mécaniques (servo_min / servo_max).
        Retourne (servo_deg_clampé, index_servo).
        """
        if joint_name not in self.index_from_name:
            # Joint non calibré -> servo fictif
            return 90.0, -1

        idx = self.index_from_name[joint_name]
        rest = self.servo_rest[idx]
        s = self.servo_sign.get(joint_name, 1.0)

        joint_deg = math.degrees(joint_angle_rad)
        servo_deg = rest + s * joint_deg

        # clamp mécaniques
        servo_deg = max(self.servo_min[idx], min(self.servo_max[idx], servo_deg))
        return servo_deg, idx

    # ======================================================
    #  Tick principal
    # ======================================================
    def timer_callback(self) -> None:
        self.t += DT

        # Phase de marche [0, 1[
        phase = (self.t % CYCLE_TIME) / CYCLE_TIME

        # Choix du pied d'appui : alterne toutes les 2 secondes (demi-cycle)
        support_is_left = (phase < 0.5)

        # Génération des trajectoires des pieds (dans le repère du bassin)
        left_foot_rel, right_foot_rel = self.foot_trajectory(phase, support_is_left)

        # Cinématique inverse pour chaque jambe
        if self.has_left_leg:
            hipL, kneeL, ankleL = self.ik_leg_sagittal(left_foot_rel)
        else:
            hipL = kneeL = ankleL = 0.0

        if self.has_right_leg:
            hipR, kneeR, ankleR = self.ik_leg_sagittal(right_foot_rel)
        else:
            hipR = kneeR = ankleR = 0.0

        # Construction du JointState pour l'URDF (radians)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = [0.0] * len(self.joint_names)

        # On part d'une pose neutre (0 rad sur tout)
        positions = np.zeros(len(self.joint_names), dtype=float)

        # Appliquer les angles jambes dans le vecteur positions (URDF)
        def set_joint(jname: str, value: float) -> None:
            if jname in self.index_from_name:
                jidx = self.index_from_name[jname]
                positions[jidx] = value

        if self.has_left_leg:
            set_joint("cuisse_gauche_joint", hipL)
            set_joint("genou_gauche_joint", kneeL)
            set_joint("cheville_gauche_joint", ankleL)

        if self.has_right_leg:
            set_joint("cuisse_droit_joint", hipR)
            set_joint("genou_droit_joint", kneeR)
            set_joint("cheville_droite_joint", ankleR)

        js.position = positions.tolist()

        # Publication pour RViz / autres noeuds
        self.joint_pub.publish(js)

        # Optionnel : TF du bassin (base_link)
        # self.publish_tf_base_link()

        # Optionnel : markers COM / ZMP / pieds (simplifiés)
        self.publish_debug_markers(left_foot_rel, right_foot_rel)

    # ======================================================
    #  Publication TF
    # ======================================================
    def publish_tf_base_link(self) -> None:
        t = TransformStamped()
        now = self.get_clock().now().to_msg()

        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self.base_x)
        t.transform.translation.y = float(self.base_y)
        t.transform.translation.z = float(COM_HEIGHT)

        cy = math.cos(self.base_yaw * 0.5)
        sy = math.sin(self.base_yaw * 0.5)
        # rotation uniquement autour de Z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

    # ======================================================
    #  Markers de debug
    # ======================================================
    def publish_debug_markers(self, left_foot_rel: np.ndarray, right_foot_rel: np.ndarray) -> None:
        now = self.get_clock().now().to_msg()

        # COM (ici fixé au-dessus du bassin)
        com = Marker()
        com.header.stamp = now
        com.header.frame_id = "base_link"
        com.ns = "com"
        com.id = 0
        com.type = Marker.SPHERE
        com.action = Marker.ADD
        com.pose.position.x = 0.0
        com.pose.position.y = 0.0
        com.pose.position.z = COM_HEIGHT
        com.scale.x = 0.02
        com.scale.y = 0.02
        com.scale.z = 0.02
        com.color.a = 1.0
        com.color.r = 0.0
        com.color.g = 1.0
        com.color.b = 0.0
        self.com_pub.publish(com)

        # Pied gauche
        lf = Marker()
        lf.header.stamp = now
        lf.header.frame_id = "base_link"
        lf.ns = "left_foot"
        lf.id = 1
        lf.type = Marker.SPHERE
        lf.action = Marker.ADD
        lf.pose.position.x = float(left_foot_rel[0])
        lf.pose.position.y = float(left_foot_rel[1])
        lf.pose.position.z = float(left_foot_rel[2])
        lf.scale.x = lf.scale.y = lf.scale.z = 0.02
        lf.color.a = 1.0
        lf.color.r = 0.0
        lf.color.g = 0.0
        lf.color.b = 1.0
        self.left_foot_pub.publish(lf)

        # Pied droit
        rf = Marker()
        rf.header.stamp = now
        rf.header.frame_id = "base_link"
        rf.ns = "right_foot"
        rf.id = 2
        rf.type = Marker.SPHERE
        rf.action = Marker.ADD
        rf.pose.position.x = float(right_foot_rel[0])
        rf.pose.position.y = float(right_foot_rel[1])
        rf.pose.position.z = float(right_foot_rel[2])
        rf.scale.x = rf.scale.y = rf.scale.z = 0.02
        rf.color.a = 1.0
        rf.color.r = 1.0
        rf.color.g = 0.0
        rf.color.b = 0.0
        self.right_foot_pub.publish(rf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ASTERGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

