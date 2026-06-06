import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math

class HumanoidWalker(Node):
    def __init__(self):
        super().__init__('humanoid_walker')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.joint_names = [
            # Bras droit
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            # Bras gauche
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            # Jambes droite
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            # Jambes gauche
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            # Tête
            'tete_g_d_joint', 'nuque_1_joint', 'tete_h_b_joint'
        ]

        self.base_x = 0.0  # position du robot dans RViz

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        freq = 0.5  # Hz
        swing = math.sin(2 * math.pi * freq * t)
        rot = math.sin(2 * math.pi * freq * t + math.pi / 2)

        # Cycle de marche (jambes alternées)
        walk_amp = math.radians(20)
        knee_amp = math.radians(25)
        hip_d = walk_amp * swing
        hip_g = -walk_amp * swing
        knee_d = knee_amp * max(0.0, -swing)
        knee_g = knee_amp * max(0.0, swing)

        # Bras humanoïde synchronisés avec jambes opposées
        amp = math.radians(25)
        elbow_amp = math.radians(15)
        arm_swing = -swing  # inversion pour synchronisation bras opposé jambe
        pos = [
            amp * arm_swing, 0.0, amp * math.sin(2 * math.pi * freq * t + math.pi / 2) * 0.3, -elbow_amp * arm_swing,  # bras droit
            -amp * arm_swing, 0.0, -amp * math.sin(2 * math.pi * freq * t + math.pi / 2) * 0.3, elbow_amp * arm_swing   # bras gauche
        ]

        pos += [
            hip_d, knee_d, -hip_d - knee_d,   # jambe droite
            hip_g, knee_g, -hip_g - knee_g    # jambe gauche
        ]

        # Tête : mouvement léger
        pos += [
            math.radians(10) * swing,
            math.radians(5) * swing,
            math.radians(8) * swing
        ]

        # Publier les joints
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = pos
        self.joint_pub.publish(msg)

        # Déplacement du robot dans RViz (translation du base_link)
        self.base_x += 0.002  # avancer à chaque cycle (~4 cm/s)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.base_x
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

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

