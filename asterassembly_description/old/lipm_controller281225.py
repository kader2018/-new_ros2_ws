#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist # NOUVEAU: Import pour les commandes de vitesse
import math 

# ---------------- CONFIGURATION AVANCEE ----------------
# Paramètres Géométriques et de Marche
L1 = 0.18           
L2 = 0.18           
COM_HEIGHT = 0.36
STEP_WIDTH = 0.08 
STEP_LENGTH = 0.06 
STEP_HEIGHT = 0.03 
CYCLE_TIME = 1.0    
DT = 0.02           

JOINT_NAMES = [
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint"
]

# PARAMÈTRES MOTEURS (RVIZ PURE CINÉMATIQUE : REST = 0)
SERVO_PARAMS = {
    "cuisse_gauche_joint": {"rest": 0, "low":-90, "high":90, "sign": 1},
    "genou_gauche_joint": {"rest": 0, "low":0, "high":130, "sign": 1},
    "cheville_gauche_joint": {"rest": 0, "low":-60, "high":60, "sign": 1},
    "cuisse_droit_joint": {"rest": 0, "low":-90, "high":90, "sign": -1},
    "genou_droit_joint": {"rest": 0, "low":0, "high":130, "sign": 1},
    "cheville_droite_joint": {"rest": 0, "low":-60, "high":60, "sign": -1}
}

REST_ANGLES_DEG = np.array([SERVO_PARAMS[j]["rest"] for j in JOINT_NAMES], dtype=np.float64)
JOINT_IDX = {j: i for i,j in enumerate(JOINT_NAMES)}


# ---------------- FONCTION DE CONVERSION (CONSERVÉE) ----------------
def euler_to_quaternion(roll, pitch, yaw):
    """ Convertit les angles d'Euler (radians) en Quaternion. """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q

# ---------------- CINEMATIQUE ET TRAJECTOIRE ----------------
def leg_ik(x, z, L1, L2):
    """ Cinématique Inverse 2D (Pitch) avec compensation du tangage (cheville). """
    d_sq = x**2 + z**2
    d = np.sqrt(d_sq)
    d = np.clip(d, 1e-6, L1 + L2 - 1e-6)
    
    cos_knee = (L1**2 + L2**2 - d_sq) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle = np.pi - np.arccos(cos_knee)

    alpha = np.arctan2(x, z)
    cos_beta = (L1**2 + d_sq - L2**2) / (2 * L1 * d)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    
    hip_angle = alpha - beta
    ankle_angle = - (hip_angle + knee_angle)
    
    return hip_angle, knee_angle, ankle_angle

def foot_trajectory(step_length, step_height, phase):
    """ Génère la trajectoire cycloidale modifiée. """
    if phase < 0.5:
        s = phase / 0.5
        x = -step_length / 2 + step_length * (s - (1 / (2 * np.pi)) * np.sin(2 * np.pi * s))
        z = step_height * np.sin(np.pi * s)
    else:
        s = (phase - 0.5) / 0.5
        x = step_length / 2 - step_length * s
        z = 0.0
    return x, z

# ---------------- NODE CONTROLEUR ----------------
class ASTERGaitNode(Node):
    def __init__(self):
        super().__init__('lipm_controller_expert')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publishers de débogage (optionnels)
        self.zmp_pub    = self.create_publisher(Marker,'/zmp_marker',10)
        self.com_pub    = self.create_publisher(Marker,'/com_marker',10)
        self.left_foot_pub  = self.create_publisher(Marker,'/left_foot_traj',10)
        self.right_foot_pub = self.create_publisher(Marker,'/right_foot_traj',10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = COM_HEIGHT 
        
        self.t = 0.0
        
        # NOUVEAU: VARIABLES DE CONTRÔLE DE COMMANDE
        self.is_walking = False          # État de marche (True/False)
        self.robot_yaw = 0.0             # Accumulateur d'angle de Yaw (position)
        self.desired_yaw_rate = 0.0      # Vitesse angulaire souhaitée (vitesse)
        self.desired_linear_x = 0.0      # Vitesse linéaire souhaitée (vitesse)
        
        # NOUVEAU: ABONNEMENT À /CMD_VEL
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.timer = self.create_timer(DT, self.timer_callback)
        self.get_logger().info('Contrôleur de marche ASTER démarré.') 

    # NOUVELLE MÉTHODE: CALLBACK DE COMMANDE DE VITESSE
    def cmd_vel_callback(self, msg):
        self.desired_linear_x = msg.linear.x
        self.desired_yaw_rate = msg.angular.z
        
        # Démarre la marche si la vitesse linéaire OU angulaire est non nulle
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            if not self.is_walking:
                 self.get_logger().info('Commande reçue: Démarrage de la marche.')
            self.is_walking = True
        else:
            if self.is_walking:
                 self.get_logger().info('Commande reçue: Arrêt de la marche. Maintien de la posture.')
            self.is_walking = False


    def timer_callback(self):
        
        # ---------------- LOGIQUE D'ARRÊT ----------------
        if not self.is_walking:
            # 1. Publie la pose de repos (maintien de la posture)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = np.radians(REST_ANGLES_DEG).tolist()
            self.joint_pub.publish(msg)
            
            # 2. Réinitialise le temps de cycle pour un redémarrage propre
            self.t = 0.0
            
            # 3. Arrête l'exécution de la logique de marche
            return 
        
        # ---------------- LOGIQUE DE MARCHE (Si is_walking est True) ----------------
        
        # 1. Mise à jour de l'orientation du robot (virage)
        self.robot_yaw += self.desired_yaw_rate * DT # Intègre la vitesse angulaire
        
        
        phase = (self.t / CYCLE_TIME) % 1.0
        pl = phase
        pr = (phase + 0.5) % 1.0
        
        # Les calculs de trajectoire des pieds ne changent pas, seule la TF de la base changera.
        dxl, dzl = foot_trajectory(STEP_LENGTH, STEP_HEIGHT, pl)
        dxr, dzr = foot_trajectory(STEP_LENGTH, STEP_HEIGHT, pr)
        
        xl = dxl
        zl = COM_HEIGHT - dzl
        
        xr = dxr
        zr = COM_HEIGHT - dzr

        # Cinématique Inverse (IK)
        hipL, kneeL, ankleL = leg_ik(xl, zl, L1, L2)
        hipR, kneeR, ankleR = leg_ik(xr, zr, L1, L2)

        left_rad  = [hipL, kneeL, ankleL]
        right_rad = [hipR, kneeR, ankleR]
        
        # Mappage aux Servos ( inchangé )
        angles_deg = np.array(REST_ANGLES_DEG, dtype=np.float64)  
        
        left_deg = np.degrees(left_rad)
        right_deg = np.degrees(right_rad)
        
        # =========================================================================
        # >>> MAPPING DES ANGLES CALCULÉS VERS LE JOINTSTATE <<<
        # =========================================================================
        
        # 1. Jambe Gauche
        angles_deg[JOINT_IDX["cuisse_gauche_joint"]]  = left_deg[0] * SERVO_PARAMS["cuisse_gauche_joint"]["sign"]
        angles_deg[JOINT_IDX["genou_gauche_joint"]]    = left_deg[1] * SERVO_PARAMS["genou_gauche_joint"]["sign"]
        angles_deg[JOINT_IDX["cheville_gauche_joint"]] = left_deg[2] * SERVO_PARAMS["cheville_gauche_joint"]["sign"]

        # 2. Jambe Droite
        angles_deg[JOINT_IDX["cuisse_droit_joint"]]  = right_deg[0] * SERVO_PARAMS["cuisse_droit_joint"]["sign"]
        angles_deg[JOINT_IDX["genou_droit_joint"]]    = right_deg[1] * SERVO_PARAMS["genou_droit_joint"]["sign"]
        angles_deg[JOINT_IDX["cheville_droite_joint"]] = right_deg[2] * SERVO_PARAMS["cheville_droite_joint"]["sign"]
        
        # =========================================================================

        # Publication des commandes (en RADIANS)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = np.radians(angles_deg).tolist()
        self.joint_pub.publish(msg)

        # ---------------- LOGIQUE DE DÉPLACEMENT DE LA BASE (TF) ----------------
        # La TF est dynamique, elle est mise à jour uniquement si la marche est active.
        
        # 1. Mise à jour de la position X du robot
        # Utilise la vitesse désirée au lieu du pas fixe si vous voulez varier la vitesse:
        # velocity_x = self.desired_linear_x 
        # Pour conserver le pas fixe de 6cm:
        velocity_x = STEP_LENGTH / CYCLE_TIME
        
        self.robot_x += velocity_x * DT

        # 2. Création et envoi de la transformation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'      # Parent fixe (comme dans votre config RViz)
        t.child_frame_id = 'base_link'  # Enfant mobile
        
        # Translation
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = self.robot_z 
        
        # Rotation : AJOUT du pitch de 90 degrés (1.5707 rad) ET du Yaw dynamique
        roll = 0.0
        pitch = 1.5707 # Redressement à 90 degrés
        yaw = self.robot_yaw # L'angle de rotation accumulé
        
        # Utilise la fonction euler_to_quaternion
        q = euler_to_quaternion(roll, pitch, yaw) 

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Envoi de la transformation dynamique
        self.tf_broadcaster.sendTransform(t)
        
        # ---------------- FIN TF ----------------

        self.t += DT

# ---------------- CORRECTION DU MAIN POUR ÉVITER LE RCLError ----------------
def main(args=None):
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
