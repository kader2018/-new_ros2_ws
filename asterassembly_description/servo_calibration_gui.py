import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from PyQt5 import QtWidgets, QtCore
import sys
import math

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('servo_calibration_gui')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [
            'bras_droit_de_ar_joint', 'bras_droit_h_b_joint', 'bras_droit_rot_joint', 'bras_droit_coud_joint',
            'bras_gauche_de_ar_joint', 'bras_gauche_h_b_joint', 'bras_gauche_rot_joint', 'bras_gauche_coude_joint',
            'cuisse_droit_joint', 'genou_droit_joint', 'cheville_droite_joint',
            'cuisse_gauche_joint', 'genou_gauche_joint', 'cheville_gauche_joint',
            'tete_g_d_joint', 'nuque_1_joint', 'tete_h_b_joint'
        ]
        self.values = [0.0] * len(self.joint_names)
        self.limits = [(-2.0, 2.0)] * len(self.joint_names)  # Limites en radians

        # Timer pour publier périodiquement
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def set_value(self, index, val):
        # Conversion en radians et limitation
        val_rad = math.radians(val)
        min_val, max_val = self.limits[index]
        self.values[index] = max(min(val_rad, max_val), min_val)

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.values
        self.publisher.publish(msg)

class CalibrationGUI(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Calibration des servos - ASTER')
        layout = QtWidgets.QVBoxLayout()

        self.sliders = []
        for i, name in enumerate(node.joint_names):
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(lambda val, i=i: self.update_joint(i, val))
            self.sliders.append(slider)

            row = QtWidgets.QHBoxLayout()
            row.addWidget(label)
            row.addWidget(slider)
            layout.addLayout(row)

        self.setLayout(layout)

    def update_joint(self, index, value):
        self.node.set_value(index, value)

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    node = CalibrationNode()
    gui = CalibrationGUI(node)
    gui.show()

    # Exécution parallèle GUI + ROS
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

