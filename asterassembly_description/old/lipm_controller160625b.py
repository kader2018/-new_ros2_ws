import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterDescriptor
import math
import serial
import time

class AsterWalkNode(Node):
    def __init__(self):
        super().__init__('aster_walk_lipm_full')

        # === Publishers ===
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # === Services ===
        self.create_service(SetBool, 'set_marche_active', self.handle_set_marche_active)

        # === Paramètre dynamique ===
        freq_descriptor = ParameterDescriptor(description="Fréquence de la marche (Hz)")
        self.declare_parameter('marche_frequency', 1.5, freq_descriptor)
        self.marche_frequency = self.get_parameter('marche_frequency').value
        self.add_on_set_parameters_callback(self.parameter_callback)

        # === Timer ===
        self.timer = self.create_timer(1.0 / self.marche_frequency, self.timer_callback)

        # === Configuration robot ===
        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'tete_h_b_joint'
        ]

        self.servo_offsets = {name: 0.0 for name in self.joint_names}

        # === Modèle LIPM ===
        self.zc = 0.3
        self.g = 9.81
        self.omega = math.sqrt(self.g / self.zc)
        self.x0 = 0.0
        self.vx0 = 0.1

        self.is_walking = False
        self.start_time = time.time()
        self.foot_traj = []

        self.color_on = (0.0, 1.0, 0.0, 1.0)
        self.color_off = (1.0, 0.0, 0.0, 1.0)
        self.marker_color = self.color_off

        # === Connexion série Arduino ===
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
            self.ser.flushInput()
            self.ser.flushOutput()
            self.get_logger().info("Connexion série établie avec Arduino.")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Connexion série impossible : {e}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'marche_frequency' and param.type_ == param.Type.DOUBLE:
                self.marche_frequency = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.marche_frequency, self.timer_callback)
                self.get_logger().info(f"Fréquence de marche mise à jour : {self.marche_frequency} Hz")
        return rclpy.parameter.ParameterEventDescriptors()

    def radians_to_servo(self, rad):
        deg = math.degrees(rad) + 90
        return max(0, min(180, int(deg)))

    def send_to_arduino(self, angles):
        if not self.ser or not self.ser.is_open:
            return
        try:
            values = [str(self.radians_to_servo(a)) for a in angles]
            msg = ";".join(values) + "\n"
            self.ser.flushInput()
            self.ser.write(msg.encode())
            time.sleep(0.02)

            ack = self.ser.readline().decode().strip()
            retry = 0
            while ack != "OK" and retry < 3:
                self.get_logger().warn(f"Pas d'ACK, tentative {retry+1}, réponse : '{ack}'")
                self.ser.write(msg.encode())
                time.sleep(0.02)
                ack = self.ser.readline().decode().strip()
                retry += 1

            if ack != "OK":
                self.get_logger().error(f"Erreur persistante Arduino. Dernière réponse : '{ack}'")
        except Exception as e:
            self.get_logger().error(f"Erreur envoi série : {e}")

    def handle_set_marche_active(self, request, response):
        self.is_walking = request.data
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.marker_color = self.color_on if self.is_walking else self.color_off
        response.success = True
        response.message = "Marche activée" if self.is_walking else "Mode repos manuel activé"
        return response

    def timer_callback(self):
        if self.is_walking:
            now = self.get_clock().now().nanoseconds / 1e9
            t = now - self.start_time
            freq = self.marche_frequency

            # LIPM + marche alternée gauche/droite
            x = self.x0 * math.cosh(self.omega * t) + (self.vx0 / self.omega) * math.sinh(self.omega * t)
            swing = 0.05 * math.sin(2 * math.pi * freq * t)
            z = max(0.0, swing)

            leg_phase = math.sin(2 * math.pi * freq * t)
            walk_amp = math.radians(15) * leg_phase
            knee = math.radians(20) * max(0, leg_phase)

            positions = [0.0] * 16

            # Jambes
            positions[8] = walk_amp
            positions[9] = knee
            positions[10] = -walk_amp - knee

            positions[11] = -walk_amp
            positions[12] = knee
            positions[13] = walk_amp + knee

            # Bras (en opposition de phase avec jambes)
            arm_amp = math.radians(25) * math.sin(2 * math.pi * freq * t + math.pi)
            elbow = math.radians(10) * (math.sin(2 * math.pi * freq * t + math.pi) ** 2)

            positions[0] = arm_amp
            positions[1] = 0.0
            positions[2] = 0.0
            positions[3] = -elbow

            positions[4] = arm_amp
            positions[5] = 0.0
            positions[6] = 0.0
            positions[7] = elbow

            # Tête (oscillations lentes)
            positions[14] = math.radians(5) * math.sin(2 * math.pi * 0.5 * t)
            positions[15] = math.radians(3) * math.sin(2 * math.pi * 0.25 * t)

            # Envoi
            self.publish_joint_positions(positions)
            self.send_to_arduino(positions)

            # Marker
            pt = Point(x=x, y=0.0, z=z)
            self.foot_traj.append(pt)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "foot_lipm"
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.01
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.marker_color
            marker.points = self.foot_traj
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

