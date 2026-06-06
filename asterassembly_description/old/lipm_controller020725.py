import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import serial
import time

class AsterWalkNode(Node):
    def __init__(self):
        super().__init__('aster_walk_node')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'tete_h_b_joint'
        ]

        self.constraints = {
            'bras_droit_de_ar_joint':    {'min': 0,   'max': 180, 'zero': 90,  'sens': +1},
            'bras_droit_h_b_joint':      {'min': 20,  'max': 180, 'zero': 20,  'sens': +1},
            'bras_droit_rot_joint':      {'min': 0,   'max': 180, 'zero': 90,  'sens': +1},
            'bras_droit_coud_joint':     {'min': 0,   'max': 180, 'zero': 0,   'sens': +1},
            'bras_gauche_de_ar_joint':   {'min': 0,   'max': 180, 'zero': 100, 'sens': -1},
            'bras_gauche_h_b_joint':     {'min': 0,   'max': 170, 'zero': 160, 'sens': -1},
            'bras_gauche_rot_joint':     {'min': 0,   'max': 180, 'zero': 90,  'sens': -1},
            'bras_gauche_coude_joint':   {'min': 40,  'max': 160, 'zero': 160, 'sens': -1},
            'cuisse_droit_joint':        {'min': 0,   'max': 180, 'zero': 110, 'sens': +1},
            'genou_droit_joint':         {'min': 80,  'max': 180, 'zero': 180, 'sens': +1},
            'cheville_droite_joint':     {'min': 90,  'max': 150, 'zero': 130, 'sens': +1},
            'cuisse_gauche_joint':       {'min': 0,   'max': 180, 'zero': 70,  'sens': -1},
            'genou_gauche_joint':        {'min': 0,   'max': 100, 'zero': 0,   'sens': -1},
            'cheville_gauche_joint':     {'min': 0,   'max': 50,  'zero': 30,  'sens': -1},
            'tete_g_d_joint':            {'min': 0,   'max': 180, 'zero': 160, 'sens': +1},
            'tete_h_b_joint':            {'min': 0,   'max': 180, 'zero': 0,   'sens': +1},
        }

        self.zc = 0.3
        self.g = 9.81
        self.omega = math.sqrt(self.g / self.zc)
        self.x0 = 0.0
        self.vx0 = 0.1
        self.start_time = time.time()
        self.foot_traj = []

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 500000, timeout=0.1)
            self.get_logger().info("✅ Arduino connecté")
            time.sleep(5.0)
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"❌ Arduino non détecté : {e}")

        self.send_rest_pose()
        time.sleep(3)
        self.is_walking = True
        self.start_time = time.time()

    def radians_to_servo(self, rad, joint_name):
        joint = self.constraints[joint_name]
        angle_deg = math.degrees(rad) * joint['sens']
        servo_deg = joint['zero'] + angle_deg
        return int(max(joint['min'], min(joint['max'], servo_deg)))

    def send_to_arduino(self, positions):
        if not self.ser or not self.ser.is_open:
            return
        try:
            degrees = [self.radians_to_servo(pos, name) for pos, name in zip(positions, self.joint_names)]
            msg = ";".join(str(d) for d in degrees) + "\n"
            self.ser.write(msg.encode())
            self.ser.flush()  # ✅ assure l'envoi
            time.sleep(0.05)  # petite pause

            ack = self.ser.readline().decode(errors='ignore').strip()
            if ack != "OK":
                self.get_logger().warn(f"⚠️ Arduino n'a pas renvoyé OK : '{ack}'")
            self.get_logger().info(f"🛰️ Trame envoyée : {msg.strip()}")

        except Exception as e:
            self.get_logger().error(f"Erreur série : {e}")

    def timer_callback(self):
        if not self.is_walking:
            return

        t = time.time() - self.start_time
        freq = 0.1  # très lent

         # MARCHE LENTE
        swing = math.sin(2 * math.pi * freq * t)

        walk_amp = math.radians(25) * swing
        knee_droit = math.radians(40) * abs(swing)
        knee_gauche = math.radians(40) * abs(swing)
        ankle = -walk_amp - 0.5 * knee_droit

        # BRAS
        arm_swing = math.radians(30) * math.sin(2 * math.pi * freq * t + math.pi)
        elbow_flex = math.radians(20) * (math.sin(2 * math.pi * freq * t + math.pi) ** 2)

        # TETE
        head_pan = math.radians(5) * math.sin(2 * math.pi * freq * 0.5 * t)
        head_tilt = math.radians(3) * math.sin(2 * math.pi * freq * 0.25 * t)

        # POSITIONS
        positions = [
        arm_swing, 0.0, -elbow_flex, elbow_flex,       # bras droit
        -arm_swing, 0.0, elbow_flex, elbow_flex,       # bras gauche
        walk_amp, knee_droit, ankle,                   # jambe droite
        -walk_amp, knee_gauche, -ankle,                # jambe gauche
        head_pan, head_tilt                            # tête
        ]

        # ✅ debug angles
        self.get_logger().debug(f"rad: {positions}")
        self.get_logger().debug(f"deg: {[self.radians_to_servo(p, n) for p, n in zip(positions, self.joint_names)]}")

        self.publish_joint_positions(positions)
        self.send_to_arduino(positions)

    def send_rest_pose(self):
        rest_angles = [0.0] * len(self.joint_names)
        self.publish_joint_positions(rest_angles)
        self.send_to_arduino(rest_angles)

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

