#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# --- METS ICI TOUS LES JOINTS DE TON URDF SANS EXCEPTION ---
ALL_JOINTS = [
    "bras_droit_de_ar_joint", "bras_droit_h_b_joint", "bras_droit_rot_joint", "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint", "bras_gauche_h_b_joint", "bras_gauche_rot_joint", "bras_gauche_coude_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint",
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "tete_g_d_joint", "tete_h_b_joint",
    # Ajoute les joints manquants ici pour éviter l'effet "tas de pièces"
]

class QuasiStaticWalker(Node):
    def __init__(self):
        super().__init__('quasistatic_walker')
        self.speed = 0.0
        # On s'abonne à l'IHM
        self.sub_speed = self.create_subscription(Float64, '/step_frequency', self.speed_cb, 10)
        
        # On publie sur /joint_states avec une priorité élevée (QoS)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(JointState, '/joint_states', qos)
        
        self.timer = self.create_timer(0.02, self.update)
        self.t = 0.0
        self.get_logger().info("Walker ASTER : Prêt à écraser le conflit.")

    def speed_cb(self, msg):
        self.speed = msg.data

    def update(self):
        if self.speed > 0:
            self.t += 0.02
        
        # Trajectoire simplifiée pour test
        freq = self.speed
        omega_t = 2.0 * math.pi * freq * self.t
        
        # On crée le dictionnaire de TOUS les joints (tous à 0.0 au départ)
        pos_dict = {name: 0.0 for name in ALL_JOINTS}

        if self.speed > 0:
            # Calcul mouvement
            swing = 0.3 * math.sin(omega_t)
            # Affectation
            pos_dict["cuisse_gauche_joint"] = swing
            pos_dict["cuisse_droit_joint"] = -swing
            pos_dict["bras_gauche_de_ar_joint"] = -swing
            pos_dict["bras_droit_de_ar_joint"] = -swing # car URDF inversé
            pos_dict["tete_g_d_joint"] = 0.1 * math.sin(omega_t)

        # On construit le message JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(pos_dict.keys())
        msg.position = list(pos_dict.values())
        
        # On publie
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = QuasiStaticWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
