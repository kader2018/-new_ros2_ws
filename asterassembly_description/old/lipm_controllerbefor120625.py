import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import serial
import time

class AsterWalkNode(Node):
    def __init__(self):
        super().__init__('aster_walk_lipm_full')

        # ROS pubs/subs/services
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_service(SetBool, 'set_marche_active', self.handle_set_marche_active)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 50 Hz

        # Liste des joints
        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'tete_h_b_joint'
        ]

        # Offsets
        self.servo_offsets = {name: 0.0 for name in self.joint_names}

        # Paramètres LIPM
        self.zc = 0.3  # hauteur CoM (m)
        self.g = 9.81
        self.omega = math.sqrt(self.g / self.zc)
        self.x0 = 0.0
        self.vx0 = 0.1

        # État du robot
        self.is_walking = False
        self.start_time = time.time()

        # Couleurs pour RViz
        self.color_on = (0.0, 1.0, 0.0, 1.0)   # Vert
        self.color_off = (1.0, 0.0, 0.0, 1.0)  # Rouge
        self.marker_color = self.color_off
        self.foot_traj = []

        # Série Arduino
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Connexion série établie avec Arduino.")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Connexion série impossible : {e}")

    def radians_to_servo(self, rad):
        deg = math.degrees(rad) + 90
        return max(0, min(180, int(deg)))

    def send_to_arduino(self, angles):
        if not self.ser or not self.ser.is_open:
            return
        try:
            values = [str(self.radians_to_servo(a)) for a in angles]
            msg = ";".join(values) + "\n"
            self.ser.write(msg.encode())
            ack = self.ser.readline().decode().strip()
            if ack != "OK":
                self.get_logger().warn(f"Pas d'ACK : {ack}")
        except Exception as e:
            self.get_logger().error(f"Erreur envoi série : {e}")

    def handle_set_marche_active(self, request, response):
        self.is_walking = request.data
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.marker_color = self.color_on if self.is_walking else self.color_off
        response.success = True
        response.message = "Marche activée" if self.is_walking else "Repos"
        return response

    def timer_callback(self):
        if not self.is_walking:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        # 💡 LIPM: calcul du centre de masse en X
        x = self.x0 * math.cosh(self.omega * t) + (self.vx0 / self.omega) * math.sinh(self.omega * t)

        # 💡 Oscillation verticale simulée pour les pieds
        swing = 0.05 * math.sin(2 * math.pi * 1.5 * t)  # 5 cm d’oscillation
        z = max(0.0, swing)

        # 🦿 JAMBES
        walk_amp = math.radians(15) * math.sin(2 * math.pi * 1.5 * t)
        knee = math.radians(20) * max(0, math.sin(2 * math.pi * 1.5 * t))

        positions = [0.0] * 16

        # Hanches / Genoux / Chevilles
        positions[8] = walk_amp
        positions[9] = knee
        positions[10] = -walk_amp - knee

        positions[11] = -walk_amp
        positions[12] = knee
        positions[13] = walk_amp + knee

        # 💪 BRAS (balancement en opposition)
        #bras_amp_long = math.radians(30) * math.sin(2 * math.pi * 1.5 * t)
        #coude_amp = math.radians(15) * (1 + math.sin(2 * math.pi * 1.5 * t)) / 2
        # Bras : balancement opposé aux jambes
        arm_amp = math.radians(25) * math.sin(2 * math.pi * 1.5 * t + math.pi)  # bras opposé à la jambe
        elbow = math.radians(10) * (math.sin(2 * math.pi * 1.5 * t + math.pi) ** 2)  # coude plie légèrement


        positions[0] = arm_amp         # bras_droit_de_ar_joint (épaule avant/arrière)
        positions[1] = 0.0              # bras_droit_h_b_joint
        positions[2] = 0.0              # bras_droit_rot_joint
        positions[3] = -elbow            # bras_droit_coud_joint

        positions[4] = arm_amp          # bras_gauche_de_ar_joint
        positions[5] = 0.0              # bras_gauche_h_b_joint
        positions[6] = 0.0              # bras_gauche_rot_joint
        positions[7] = elbow            # bras_gauche_coude_joint

        # 🧠 TÊTE (mouvement de type "scanning")
        head_pan = math.radians(5) * math.sin(2 * math.pi * 0.5 * t)  # gauche/droite
        head_tilt = math.radians(3) * math.sin(2 * math.pi * 0.25 * t)  # haut/bas

        positions[14] = head_pan   # tete_g_d_joint
        positions[15] = head_tilt  # tete_h_b_joint

      
        # Publier les angles
        self.publish_joint_positions(positions)
        self.send_to_arduino(positions)

        # RViz marker
        pt = Point()
        pt.x = x
        pt.y = 0.0
        pt.z = z
        self.foot_traj.append(pt)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.marker_color
        marker.points = self.foot_traj
        marker.ns = "foot_lipm"
        marker.id = 1
        self.marker_pub.publish(marker)


    def publish_joint_positions(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AsterWalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

