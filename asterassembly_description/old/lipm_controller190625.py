import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import serial
import threading
import time
from queue import Queue, Full, Empty

class AsterWalkNode(Node):
    def __init__(self):
        super().__init__('aster_walk_lipm_full')

        self.zc = 0.3
        self.g = 9.81
        self.omega = math.sqrt(self.g / self.zc) if self.zc > 0 else None
        if self.omega is None:
            self.get_logger().error("zc doit être strictement positif !")
            raise ValueError("zc doit être strictement positif")

        self.x0 = 0.0
        self.vx0 = 0.1

        self.is_walking = False
        self.start_time = None

        self.color_on = (0.0, 1.0, 0.0, 1.0)
        self.color_off = (1.0, 0.0, 0.0, 1.0)
        self.marker_color = self.color_off
        self.foot_traj = []

        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'tete_h_b_joint'
        ]
        self.servo_offsets = {name: 0.0 for name in self.joint_names}

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 50)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 50)
        self.create_service(SetBool, 'set_marche_active', self.handle_set_marche_active)

        self.declare_parameter('vitesse_marche', 0.1)
        self.declare_parameter('amplitude_mouvement', 1.0)
        self.declare_parameter('timer_frequency', 3.0)

        self.timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.timer_frequency, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.ser = None
        self.serial_lock = threading.Lock()

        self.last_send_failed = False

        # Queue max 50 commandes
        self.command_queue = Queue(maxsize=50)

        self.connect_serial()

        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

        # Temps minimum entre deux envois en secondes
        self.min_send_interval = 0.5
        self.last_send_time = 0

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.7)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connexion série établie avec Arduino sur {self.serial_port}")
            self.last_send_failed = False
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Impossible de connecter à l'Arduino: {e}")

    def radians_to_servo(self, rad):
        deg = math.degrees(rad) + 90
        return max(0, min(180, int(deg)))

    def _send_to_arduino_blocking(self, angles):
        now = time.time()
        if now - self.last_send_time < self.min_send_interval:
            # Trop rapide, on skip pour éviter saturation
            return
        self.last_send_time = now

        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Port série non ouvert, tentative de reconnexion...")
            self.connect_serial()
            if self.ser is None:
                return

        values = [str(self.radians_to_servo(a)) for a in angles]
        msg = ";".join(values) + "\n"

        with self.serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(msg.encode())
                self.ser.flush()

                start_time = time.time()
                ack_received = False
                while time.time() - start_time < 0.1:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode(errors='ignore').strip()
                        if line == "OK":
                            ack_received = True
                            break

                if ack_received:
                    if self.last_send_failed:
                        self.get_logger().info("Réception ACK rétablie.")
                    self.last_send_failed = False
                else:
                    if not self.last_send_failed:
                        self.get_logger().warn("Pas d'ACK reçu après envoi commande.")
                    self.last_send_failed = True

            except Exception as e:
                if not self.last_send_failed:
                    self.get_logger().error(f"Erreur en communication série : {e}")
                self.last_send_failed = True

    def _serial_worker(self):
        while rclpy.ok():
            try:
                angles = self.command_queue.get(timeout=0.1)
                self._send_to_arduino_blocking(angles)
                self.command_queue.task_done()
            except Empty:
                # Pas de commande en attente, on attend la prochaine
                pass
            except Exception as e:
                self.get_logger().error(f"Erreur thread série : {e}")

    def handle_set_marche_active(self, request, response):
        self.is_walking = request.data
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.marker_color = self.color_on if self.is_walking else self.color_off
        response.success = True
        response.message = "Marche activée" if self.is_walking else "Repos"
        self.get_logger().info(response.message)
        return response

    def parameter_callback(self, params):
        successful = True
        for param in params:
            if param.name == "timer_frequency" and param.type_ == param.Type.DOUBLE:
                new_freq = param.value
                if new_freq <= 0.0:
                    self.get_logger().warn("La fréquence doit être > 0.0")
                    successful = False
                else:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / new_freq, self.timer_callback)
                    self.timer_frequency = new_freq
                    self.get_logger().info(f"Timer mis à jour : {new_freq} Hz")
            elif param.name in ['vitesse_marche', 'amplitude_mouvement']:
                self.get_logger().debug(f"Paramètre '{param.name}' mis à jour : {param.value}")
        return SetParametersResult(successful=successful)

    def timer_callback(self):
        if not self.is_walking:
            return

        vitesse = self.get_parameter('vitesse_marche').get_parameter_value().double_value
        amplitude = self.get_parameter('amplitude_mouvement').get_parameter_value().double_value
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time
        t = min(t, 10.0)

        try:
            x = self.x0 * math.cosh(self.omega * t) + (self.vx0 / self.omega) * math.sinh(self.omega * t)
        except OverflowError as e:
            self.get_logger().error(f"Overflow dans les calculs LIPM : {e}")
            self.start_time = now
            return

        swing = max(0.0, 0.05 * math.sin(2 * math.pi * vitesse * t))
        base_amp = math.radians(15)
        base_knee = math.radians(20)
        walk_amp = base_amp * amplitude * math.sin(2 * math.pi * vitesse * t)
        knee = base_knee * amplitude * max(0, math.sin(2 * math.pi * vitesse * t))

        positions = [0.0] * 16

        # Jambes
        positions[8] = walk_amp
        positions[9] = knee
        positions[10] = -walk_amp - knee
        positions[11] = -walk_amp
        positions[12] = knee
        positions[13] = walk_amp + knee

        # Bras
        arm_amp = math.radians(25) * amplitude * math.sin(2 * math.pi * vitesse * t + math.pi)
        elbow = math.radians(10) * (math.sin(2 * math.pi * vitesse * t + math.pi) ** 2)

        positions[0] = arm_amp
        positions[1] = 0.0
        positions[2] = 0.0
        positions[3] = -elbow

        positions[4] = arm_amp
        positions[5] = 0.0
        positions[6] = 0.0
        positions[7] = -elbow

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = positions
        self.joint_pub.publish(joint_msg)

        # Envoi en évitant la saturation de la queue
        if not self.command_queue.full():
            try:
                self.command_queue.put_nowait(positions)
            except Full:
                self.get_logger().warn("Queue de commandes pleine, commande ignorée.")
        else:
            self.get_logger().warn("Queue de commandes pleine, envoi abandonné.")

        # Publication d'un marker (exemple simple)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "foot_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.marker_color
        marker.points.append(Point(x=float(x), y=0.0, z=0.0))
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = AsterWalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur")
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

