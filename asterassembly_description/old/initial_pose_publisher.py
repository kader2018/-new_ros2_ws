from rclpy.node import Node
from sensor_msgs.msg import JointState
import rclpy
import time

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if self.published:
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_state.name = [
            'Pied gauche', 'Genou gauche', 'Hanche droite', 'Hanche gauche',
            'Pied droit', 'Genou droit',
            'Avant-bras gauche Axe transverse', 'Bras gauche Axe longitudinal',
            'Epaule gauche 1 axe sagittal', 'Epaule gauche 2 axe transverse',
            'Avant-bras droit Axe transverse', 'Bras droit Axe longitudinal',
            'Epaule droite 1 axe sagittal', 'Epaule droite 2 axe transverse'
        ]

        # Position de repos (ex : les valeurs que tu m’as données précédemment)
        joint_state.position = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ]

        self.publisher_.publish(joint_state)
        self.get_logger().info("✅ Position initiale envoyée")
        self.published = True

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
