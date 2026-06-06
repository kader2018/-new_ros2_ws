#!/usr/bin/env python3
import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ------------------------------
#  CONFIG JOINTS & DT
# ------------------------------
JOINT_NAMES: List[str] = [
    "bras_droit_de_ar_joint", "bras_droit_h_b_joint", "bras_droit_rot_joint", "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint", "bras_gauche_h_b_joint", "bras_gauche_rot_joint", "bras_gauche_coude_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint",
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "tete_g_d_joint", "tete_h_b_joint",
]

HIP_AMPLITUDE   = 0.25      
KNEE_BASE       = 0.30      
KNEE_AMPLITUDE  = 0.25      
ANKLE_AMPLITUDE = 0.15      
ARM_SWING_AMPL  = 0.20      
DT              = 0.01      

SIGN: Dict[str, float] = {
    "cuisse_gauche_joint": +1.0, "genou_gauche_joint": +1.0, "cheville_gauche_joint": +1.0,
    "cuisse_droit_joint": +1.0, "genou_droit_joint": +1.0, "cheville_droite_joint": +1.0,
    "bras_gauche_de_ar_joint": +1.0, "bras_droit_de_ar_joint": +1.0,
}

class QuasiStaticWalker(Node):
    def __init__(self) -> None:
        super().__init__("quasistatic_walker")
        
        # --- DECLARATION DU PARAMETRE DE VITESSE ---
        self.declare_parameter('step_frequency', 0.5)
        
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.t = 0.0
        self.index_from_name = {name: i for i, name in enumerate(JOINT_NAMES)}
        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info("Marcheur Aster prêt (Vitesse dynamique activée).")

    def leg_cpg(self, phase: float):
        hip = HIP_AMPLITUDE * math.sin(phase)
        knee = KNEE_BASE + KNEE_AMPLITUDE * (1.0 - math.cos(phase)) / 2.0
        ankle = -0.5 * hip - 0.3 * (knee - KNEE_BASE)
        return hip, knee, ankle

    def arm_cpg(self, phase: float):
        shoulder = ARM_SWING_AMPL * math.sin(phase + math.pi)
        return shoulder, 0.0

    def update(self) -> None:
        # --- RECUPERATION DE LA VITESSE DEPUIS L'IHM ---
        current_freq = self.get_parameter('step_frequency').get_parameter_value().double_value
        
        self.t += DT
        omega = 2.0 * math.pi * current_freq
        phase = omega * self.t

        phase_left = phase
        phase_right = phase + math.pi

        hipL, kneeL, ankleL = self.leg_cpg(phase_left)
        hipR, kneeR, ankleR = self.leg_cpg(phase_right)
        shoulderL, elbowL = self.arm_cpg(phase_left)
        shoulderR, elbowR = self.arm_cpg(phase_right)

        pos = [0.0] * len(JOINT_NAMES)

        def set_joint(name: str, value: float):
            if name in self.index_from_name:
                idx = self.index_from_name[name]
                s = SIGN.get(name, 1.0)
                pos[idx] = s * value

        set_joint("cuisse_gauche_joint", hipL); set_joint("genou_gauche_joint", kneeL); set_joint("cheville_gauche_joint", ankleL)
        set_joint("cuisse_droit_joint", hipR); set_joint("genou_droit_joint", kneeR); set_joint("cheville_droite_joint", ankleR)
        set_joint("bras_gauche_de_ar_joint", shoulderL); set_joint("bras_gauche_coude_joint", elbowL)
        set_joint("bras_droit_de_ar_joint", shoulderR); set_joint("bras_droit_coud_joint", elbowR)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = pos
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = QuasiStaticWalker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # On ne shutdown que si rclpy est encore actif
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
