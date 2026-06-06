import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QSlider, QLabel
)
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class WalkControlNode(Node):
    def __init__(self):
        super().__init__('walk_control_gui_node')
        self.cli = self.create_client(SetBool, '/set_marche_active')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_marche_active non disponible, attente...')
        self.req = SetBool.Request()

        # Déclare les paramètres dynamiques
        self.declare_parameter('vitesse_marche', 0.03)
        self.declare_parameter('amplitude_mouvement', 1.0)
        self.declare_parameter('timer_frequency', 5.0)

    def call_service(self, activate: bool):
        self.req.data = activate
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Erreur lors de l'appel du service")
            return None

    def set_vitesse(self, vitesse):
        self.set_parameters([rclpy.parameter.Parameter(
            'vitesse_marche', rclpy.Parameter.Type.DOUBLE, vitesse)])

    def set_amplitude(self, amplitude):
        self.set_parameters([rclpy.parameter.Parameter(
            'amplitude_mouvement', rclpy.Parameter.Type.DOUBLE, amplitude)])

    def set_frequence(self, frequence):
        self.set_parameters([rclpy.parameter.Parameter(
            'timer_frequency', rclpy.Parameter.Type.DOUBLE, frequence)])


class WalkControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Contrôle de la Marche - ASTER')
        self.setFixedSize(350, 300)
        layout = QVBoxLayout()

        # Boutons de commande
        self.btn_start = QPushButton('▶️ Démarrer la marche')
        self.btn_stop = QPushButton('⏹️ Arrêter la marche')
        self.btn_start.clicked.connect(self.start_walk)
        self.btn_stop.clicked.connect(self.stop_walk)

        layout.addWidget(self.btn_start)
        layout.addWidget(self.btn_stop)

        # Slider Vitesse
        self.vitesse_slider = QSlider(Qt.Horizontal)
        self.vitesse_slider.setRange(1, 20)
        self.vitesse_slider.setValue(3)  # = 0.03
        self.vitesse_slider.valueChanged.connect(self.update_vitesse)
        self.vitesse_label = QLabel('Vitesse : 0.03 m/s')

        layout.addWidget(self.vitesse_label)
        layout.addWidget(self.vitesse_slider)

        # Slider Amplitude
        self.amplitude_slider = QSlider(Qt.Horizontal)
        self.amplitude_slider.setRange(1, 20)
        self.amplitude_slider.setValue(10)  # = 1.0
        self.amplitude_slider.valueChanged.connect(self.update_amplitude)
        self.amplitude_label = QLabel('Amplitude : 1.00x')

        layout.addWidget(self.amplitude_label)
        layout.addWidget(self.amplitude_slider)

        # Slider Fréquence
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setRange(1, 50)
        self.freq_slider.setValue(5)  # = 5 Hz
        self.freq_slider.valueChanged.connect(self.update_frequence)
        self.freq_label = QLabel('Fréquence Timer : 5.0 Hz')

        layout.addWidget(self.freq_label)
        layout.addWidget(self.freq_slider)

        self.setLayout(layout)

    def start_walk(self):
        result = self.node.call_service(True)
        if result:
            print(f"✅ {result.message}")

    def stop_walk(self):
        result = self.node.call_service(False)
        if result:
            print(f"⛔️ {result.message}")

    def update_vitesse(self, value):
        vitesse = round(value * 0.01, 3)
        self.vitesse_label.setText(f'Vitesse : {vitesse:.2f} m/s')
        self.node.set_vitesse(vitesse)

    def update_amplitude(self, value):
        amplitude = round(value * 0.1, 2)
        self.amplitude_label.setText(f'Amplitude : {amplitude:.2f}x')
        self.node.set_amplitude(amplitude)

    def update_frequence(self, value):
        frequence = float(value)
        self.freq_label.setText(f'Fréquence Timer : {frequence:.1f} Hz')
        self.node.set_frequence(frequence)


def main(args=None):
    rclpy.init(args=args)
    node = WalkControlNode()
    app = QApplication(sys.argv)
    gui = WalkControlGUI(node)
    gui.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

