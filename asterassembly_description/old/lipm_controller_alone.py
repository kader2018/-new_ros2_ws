import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point, AccelStamped
from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
import math
import serial
import time

class HumanoidWalker(Node):
    def __init__(self):
        super().__init__('humanoid_walker')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/foot_trajectory_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_accel = self.create_subscription(
            AccelStamped,
            '/accel_control',
            self.accel_callback,
            10
        )

        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.marche_active = True  # par défaut actif

        self.srv = self.create_service(SetBool, '/set_marche_active', self.set_marche_active_callback)

        self.freq = 0.2  # fréquence initiale (Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'tete_h_b_joint'
        ]

        self.servo_offsets = {name: 0.0 for name in self.joint_names}
        self.servo_offsets['bras_droit_h_b_joint'] = math.radians(-5)
        self.servo_offsets['bras_gauche_h_b_joint'] = math.radians(-5)
        self.base_x = 0.0
        self.foot_traj = []
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("Connexion série avec Arduino établie")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur connexion série : {e}")
            self.ser = None

    def radians_to_servo_degrees(self, rad):
        deg = math.degrees(rad) + 90
        return max(0, min(180, round(deg)))

    def accel_callback(self, msg: AccelStamped):
        val = msg.accel.linear.x
        self.freq = max(0.05, min(1.0, val))
        self.get_logger().info(f"Fréquence mise à jour: {self.freq:.2f} Hz")

    def set_marche_active_callback(self, request, response):
        self.marche_active = request.data
        etat = "Marche" if self.marche_active else "Repos"
        self.get_logger().info(f"Mode changé : {etat}")
        response.success = True
        response.message = etat
        return response

    def joint_states_callback(self, msg: JointState):
        if self.marche_active:
            return  # On ignore le contrôle manuel si marche active

        if self.ser and self.ser.is_open:
            try:
                degrees = [self.radians_to_servo_degrees(angle) for angle in msg.position]
                data_str = ";".join(str(d) for d in degrees) + "\n"
                self.ser.write(data_str.encode('utf-8'))
                self.get_logger().info(f"[MANUEL RViz] Sent: {data_str.strip()}")
            except Exception as e:
                self.get_logger().error(f"[Erreur série - manuel] : {e}")

    def timer_callback(self):
        if not self.marche_active:
            return  # Pas de marche si désactivée

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

        # Jambes
        hip_g = walk_amp * swing
        hip_d = -walk_amp * swing
        knee_g = knee_amp * max(0.0, -swing)
        knee_d = knee_amp * max(0.0, swing)

        # Bras
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

        pos = [
            base + self.servo_offsets.get(name, 0.0)
            for base, name in zip(base_positions, self.joint_names)
        ]

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = pos
        self.joint_pub.publish(msg)

        if self.ser and self.ser.is_open:
            try:
                degrees = [self.radians_to_servo_degrees(angle) for angle in pos]
                data_str = ";".join(str(d) for d in degrees) + "\n"
                self.ser.write(data_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Erreur envoi série: {e}")

        # TF
        self.base_x += 0.002 * self.freq
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.base_x
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        angle_rad = math.radians(90)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(angle_rad / 2)
        tf.transform.rotation.w = math.cos(angle_rad / 2)
        self.tf_broadcaster.sendTransform(tf)

        self.foot_traj.append(Point(x=self.base_x, y=0.0, z=0.0))
        if len(self.foot_traj) > 100:
            self.foot_traj.pop(0)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "foot_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.points = self.foot_traj
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

