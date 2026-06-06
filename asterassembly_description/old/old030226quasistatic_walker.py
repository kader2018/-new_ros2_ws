#!/usr/bin/env python3
import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ------------------------------
#  JOINTS (d'après ton JSON/URDF)
# ------------------------------
JOINT_NAMES: List[str] = [
    "bras_droit_de_ar_joint",
    "bras_droit_h_b_joint",
    "bras_droit_rot_joint",
    "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint",
    "bras_gauche_h_b_joint",
    "bras_gauche_rot_joint",
    "bras_gauche_coude_joint",
    "cuisse_droit_joint",
    "genou_droit_joint",
    "cheville_droite_joint",
    "cuisse_gauche_joint",
    "genou_gauche_joint",
    "cheville_gauche_joint",
    "tete_g_d_joint",
    "tete_h_b_joint",
]

# ------------------------------
#  PARAMÈTRES MARCHE CPG
# ------------------------------
STEP_FREQUENCY = 0.5        # Hz -> 2 s par pas complet
HIP_AMPLITUDE   = 0.25      # rad (~14°) amplitude hanche
KNEE_BASE       = 0.30      # rad flexion de base genou
KNEE_AMPLITUDE  = 0.25      # rad amplitude genou
ANKLE_AMPLITUDE = 0.15      # rad amplitude cheville
ARM_SWING_AMPL  = 0.20      # rad amplitude bras
DT              = 0.01      # 100 Hz

# signe pour obtenir un mouvement miroir gauche/droite
SIGN: Dict[str, float] = {
    "cuisse_gauche_joint":   +1.0,
    "genou_gauche_joint":    +1.0,
    "cheville_gauche_joint": +1.0,

    "cuisse_droit_joint":    +1.0,
    "genou_droit_joint":     +1.0,
    "cheville_droite_joint": +1.0,

    "bras_gauche_de_ar_joint": +1.0,
    "bras_droit_de_ar_joint":  +1.0,
}


class QuasiStaticWalker(Node):
    """
    Générateur de marche CPG en ESPACE DES JOINTS.
    - Aucun Arduino ici, uniquement /joint_states pour RViz.
    - Petite marche symétrique, pas de grand écart, pas de “vol”.
    """

    def __init__(self) -> None:
        super().__init__("quasistatic_walker")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.t = 0.0

        # index de chaque joint dans le vecteur
        self.index_from_name = {name: i for i, name in enumerate(JOINT_NAMES)}

        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info("QuasiStaticWalker CPG initialisé (simulation seule, marche lente).")

    # ---------- CPG jambe (hip, knee, ankle) ----------

    def leg_cpg(self, phase: float) -> (float, float, float):
        """
        CPG pour une jambe :
        phase en radians [0, 2π).
        Retourne (hip, knee, ankle) en radians.
        """
        # hanche : sinusoïde
        hip = HIP_AMPLITUDE * math.sin(phase)

        # genou : flexion de base + modulation
        knee = KNEE_BASE + KNEE_AMPLITUDE * (1.0 - math.cos(phase)) / 2.0

        # cheville : compense partiellement hip + genou
        ankle = -0.5 * hip - 0.3 * (knee - KNEE_BASE)

        # clamp sécurité
        hip   = max(-0.6, min(0.6, hip))
        knee  = max(0.0,  min(1.6, knee))
        ankle = max(-0.6, min(0.6, ankle))

        return hip, knee, ankle

    # ---------- CPG bras ----------

    def arm_cpg(self, phase: float) -> (float, float):
        """
        CPG pour bras :
        - seul le joint avant/arrière (de_ar) bouge,
        - en opposition de phase avec la jambe.
        """
        shoulder = ARM_SWING_AMPL * math.sin(phase + math.pi)  # opposition de phase
        elbow = 0.0
        return shoulder, elbow

    # ---------- mise à jour périodique ----------

    def update(self) -> None:
        self.t += DT
        omega = 2.0 * math.pi * STEP_FREQUENCY
        phase = omega * self.t

        # jambes en opposition de phase
        phase_left = phase
        phase_right = phase + math.pi

        # jambe gauche/droite
        hipL, kneeL, ankleL = self.leg_cpg(phase_left)
        hipR, kneeR, ankleR = self.leg_cpg(phase_right)

        # bras gauche/droite
        shoulderL, elbowL = self.arm_cpg(phase_left)
        shoulderR, elbowR = self.arm_cpg(phase_right)

        # vecteur complet de positions
        pos = [0.0] * len(JOINT_NAMES)

        def set_joint(name: str, value: float):
            if name in self.index_from_name:
                idx = self.index_from_name[name]
                s = SIGN.get(name, 1.0)
                pos[idx] = s * value

        # jambes
        set_joint("cuisse_gauche_joint",   hipL)
        set_joint("genou_gauche_joint",    kneeL)
        set_joint("cheville_gauche_joint", ankleL)

        set_joint("cuisse_droit_joint",    hipR)
        set_joint("genou_droit_joint",     kneeR)
        set_joint("cheville_droite_joint", ankleR)

        # bras
        set_joint("bras_gauche_de_ar_joint", shoulderL)
        set_joint("bras_gauche_coude_joint", elbowL)
        set_joint("bras_droit_de_ar_joint",  shoulderR)
        set_joint("bras_droit_coud_joint",   elbowR)

        # tête à 0 rad (statique pour l'instant)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = pos

        self.pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuasiStaticWalker()
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

