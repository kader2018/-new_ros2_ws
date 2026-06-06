import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
import math
import serial
import time


class AsterNode(Node):
    def __init__(self):
        super().__init__('aster_node')
        self.srv = self.create_service(SetBool, 'set_marche_active', self.handle_set_marche_active)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'foot_trajectory', 10)
        self.led_marker_pub = self.create_publisher(Marker, 'led_status', 10)

        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.is_walking = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.freq = 1.5

        self.servo_offsets = {
            'bras_droit_de_ar_joint': math.radians(0),
            'bras_droit_h_b_joint': math.radians(-5),
            'bras_droit_rot_joint': math.radians(0),
            'bras_droit_coud_joint': math.radians(0),
            'bras_gauche_de_ar_joint': math.radians(0),
            'bras_gauche_h_b_joint': math.radians(-5),
            'bras_gauche_rot_joint': math.radians(0),
            'bras_gauche_coude_joint': math.radians(0),
            'cuisse_droit_joint': math.radians(0),
            'genou_droit_joint': math.radians(0),
            'cheville_droite_joint': math.radians(0),
            'cuisse_gauche_joint': math.radians(0),
            'genou_gauche_joint': math.radians(0),
            'cheville_gauche_joint': math.radians(0),
            'tete_g_d_joint': math.radians(0),
            'tete_h_b_joint': math.radians(0)
        }

        self.joint_names = list(self.servo_offsets.keys())

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("Connexion série avec Arduino établie")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur connexion série : {e}")
            self.ser = None

        self.base_x = 0.0
        self.foot_traj = []
        self.marker_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        self.activate_walking()

    def activate_walking(self):
        self.is_walking = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.marker_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

    def stop_walking(self):
        self.is_walking = False
        self.marker_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    def radians_to_servo_degrees(self, rad):
        deg = math.degrees(rad) + 90
        return max(0, min(180, int(deg)))

    def publish_joint_positions(self, positions):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)
    def handle_set_marche_active(self, request, response):
        self.is_walking = request.data
        if self.is_walking:
            self.activate_walking()
            self.get_logger().info("Mode MARCHE activé via service")
        else:
            self.stop_walking()
            self.get_logger().info("Mode REPOS activé via service")

        response.success = True
        response.message = f"Marche activée: {self.is_walking}"
        return response

    def timer_callback(self):
        if not self.is_walking:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time
        phase = 2 * math.pi * self.freq * t

        swing = math.sin(phase)

        walk_amp = math.radians(10)
        knee_amp = math.radians(15)
        arm_amp = math.radians(15)
        elbow_amp = math.radians(10)
        lift_amp = math.radians(5)
        lat_rot_amp = math.radians(5)

        hip_g = walk_amp * swing
        hip_d = -walk_amp * swing
        knee_g = knee_amp * max(0.0, -swing)
        knee_d = knee_amp * max(0.0, swing)

        arm_d = arm_amp * swing
        arm_g = arm_amp * swing
        elbow_d = elbow_amp * math.sin(phase + math.pi)
        elbow_g = -elbow_amp * math.sin(phase)
        lift_d = lift_amp * math.sin(phase + math.pi)
        lift_g = lift_amp * math.sin(phase)
        rot_d = -lat_rot_amp * math.sin(phase)
        rot_g = lat_rot_amp * math.sin(phase)

        base_positions = [
            arm_d, lift_d, rot_d, elbow_d,
            arm_g, lift_g, rot_g, elbow_g,
            hip_d, knee_d, -hip_d - knee_d,
            hip_g, knee_g, -hip_g - knee_g,
            math.radians(10) * swing,
            math.radians(5) * swing
        ]

        # Ajout des offsets
        pos = [
            base + self.servo_offsets.get(name, 0.0)
            for base, name in zip(base_positions, self.joint_names)
        ]

        # Publication ROS
        self.publish_joint_positions(pos)

        # Envoi à l’Arduino
        if self.ser and self.ser.is_open:
            try:
                degrees = [self.radians_to_servo_degrees(angle) for angle in pos]
                data_str = ";".join(str(d) for d in degrees) + "\n"
                self.ser.write(data_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Erreur envoi série: {e}")

        # Trajectoire du pied
        self.base_x += 0.002
        foot_pos = Point()
        foot_pos.x = self.base_x
        foot_pos.y = 0.0
        foot_pos.z = 0.0
        self.foot_traj.append(foot_pos)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "foot_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color = self.marker_color
        marker.points = self.foot_traj
        self.marker_pub.publish(marker)

        # LED RViz
        led_marker = Marker()
        led_marker.header.frame_id = "base_link"
        led_marker.header.stamp = self.get_clock().now().to_msg()
        led_marker.ns = "led_status"
        led_marker.id = 1
        led_marker.type = Marker.SPHERE
        led_marker.action = Marker.ADD
        led_marker.scale.x = 0.05
        led_marker.scale.y = 0.05
        led_marker.scale.z = 0.05
        led_marker.color = self.marker_color
        led_marker.pose.position.x = 0.0
        led_marker.pose.position.y = 0.0
        led_marker.pose.position.z = 1.0
        self.led_marker_pub.publish(led_marker)

def main(args=None):
    rclpy.init(args=args)
    node = AsterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_walking()
        node.get_logger().info("Arrêt du robot ASTER")
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

