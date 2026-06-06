#!/usr/bin/env python3
import sys
import subprocess
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QSlider, QTabWidget, QGroupBox)
from PyQt5.QtCore import Qt, QTimer

JOINT_NAMES = [
    "bras_droit_de_ar_joint", "bras_droit_h_b_joint", "bras_droit_rot_joint", "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint", "bras_gauche_h_b_joint", "bras_gauche_rot_joint", "bras_gauche_coude_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint",
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "tete_g_d_joint", "tete_h_b_joint"
]

class AsterProEngine(Node):
    def __init__(self):
        super().__init__('aster_pro_engine')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.t = 0.0
        self.mode = "WALK"
        
        # Flags d'inversion (1.0 ou -1.0)
        self.inv_global = 1.0
        self.inv_arm_r = 1.0
        self.inv_arm_l = -1.0 # Par défaut opposé
        self.inv_elbow_l = 1.0
        
        self.configs = {
            "WALK": {"speed": 0.0, "fwd": 45.0, "bwd": -15.0, "knee": 35.0, "arm_amp": 0.5, "elbow_flex": 0.4},
            "RUN":  {"speed": 0.0, "fwd": 110.0, "bwd": -25.0, "knee": 100.0, "arm_amp": 1.2, "elbow_flex": 1.1}
        }

    def update_motion(self):
        cfg = self.configs[self.mode]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        
        if cfg["speed"] > 0:
            self.t += 0.02
            phase = (2.0 * math.pi * cfg["speed"] * self.t) % (2.0 * math.pi)
            
            # --- JAMBES (Logique V26) ---
            f_rad, b_rad, k_rad = math.radians(cfg["fwd"]), math.radians(cfg["bwd"]), math.radians(cfg["knee"])
            def compute_leg(p):
                if p < math.pi: # Swing
                    n = p / math.pi
                    hip = b_rad + (f_rad - b_rad) * n
                    knee = k_rad * math.sin(n * math.pi)
                else: # Stance
                    hip = f_rad - (f_rad - b_rad) * ((p - math.pi) / math.pi)
                    knee = 0.0
                return hip * self.inv_global, knee, -hip * self.inv_global

            hL, kL, aL = compute_leg(phase)
            hR, kR, aR = compute_leg((phase + math.pi) % (2.0 * math.pi))
            
            # --- BRAS (Full Calibration) ---
            # Épaules
            sL = cfg["arm_amp"] * math.sin(phase) * self.inv_arm_l
            sR = cfg["arm_amp"] * math.sin(phase) * self.inv_arm_r
            
            # Coudes
            eL = cfg["elbow_flex"] * self.inv_elbow_l
            eR = cfg["elbow_flex"]

            d = {n: 0.0 for n in JOINT_NAMES}
            d.update({
                "cuisse_gauche_joint": hL, "genou_gauche_joint": kL, "cheville_gauche_joint": aL,
                "cuisse_droit_joint": hR, "genou_droit_joint": kR, "cheville_droite_joint": aR,
                "bras_gauche_de_ar_joint": sL, "bras_droit_de_ar_joint": sR,
                "bras_gauche_coude_joint": eL, "bras_droit_coud_joint": eR
            })
            msg.position = [d[n] for n in JOINT_NAMES]
        else:
            msg.position = [0.0] * len(JOINT_NAMES)
        self.pub.publish(msg)

class AsterIHM(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ASTER V28 - CALIBRATION EXPERT")
        self.setStyleSheet("background-color: #0f172a; color: white;")
        layout = QVBoxLayout()

        # --- SECTION INVERSIONS ---
        g_inv = QGroupBox("Inversions Cinématiques")
        il = QHBoxLayout()
        
        btn_glob = QPushButton("INV. GLOBAL"); btn_glob.setCheckable(True)
        btn_glob.clicked.connect(lambda c: setattr(self.node, 'inv_global', -1.0 if c else 1.0))
        
        btn_arm_r = QPushButton("INV. BRAS D."); btn_arm_r.setCheckable(True)
        btn_arm_r.clicked.connect(lambda c: setattr(self.node, 'inv_arm_r', -1.0 if c else 1.0))
        
        btn_arm_l = QPushButton("INV. BRAS G."); btn_arm_l.setCheckable(True)
        btn_arm_l.clicked.connect(lambda c: setattr(self.node, 'inv_arm_l', 1.0 if c else -1.0))

        btn_elb_l = QPushButton("INV. COUDE G."); btn_elb_l.setCheckable(True)
        btn_elb_l.clicked.connect(lambda c: setattr(self.node, 'inv_elbow_l', -1.0 if c else 1.0))

        for b in [btn_glob, btn_arm_r, btn_arm_l, btn_elb_l]: il.addWidget(b)
        g_inv.setLayout(il); layout.addWidget(g_inv)

        # --- ONGLETS REGLAGES ---
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_tab("WALK"), "MARCHE")
        self.tabs.addTab(self.create_tab("RUN"), "COURSE")
        self.tabs.currentChanged.connect(lambda i: setattr(self.node, 'mode', "WALK" if i == 0 else "RUN"))
        layout.addWidget(self.tabs)

        # --- RVIZ ---
        btn_rviz = QPushButton("LANCER RVIZ"); btn_rviz.clicked.connect(self.launch_rviz)
        btn_rviz.setStyleSheet("background-color: #1e293b; height: 40px; font-weight: bold;")
        layout.addWidget(btn_rviz)

        self.setLayout(layout); self.resize(650, 700)

    def create_tab(self, mode):
        w = QWidget(); l = QVBoxLayout(); cfg = self.node.configs[mode]
        self.add_s(l, "Vitesse (Hz)", 0, 40, int(cfg["speed"]*10), mode, "speed", 10.0)
        self.add_s(l, "Hanche Avant", 0, 150, int(cfg["fwd"]), mode, "fwd")
        self.add_s(l, "Hanche Arrière", -90, 30, int(cfg["bwd"]), mode, "bwd")
        self.add_s(l, "Flexion Genou", 0, 150, int(cfg["knee"]), mode, "knee")
        self.add_s(l, "Amplitude Épaule", 0, 20, int(cfg["arm_amp"]*10), mode, "arm_amp", 10.0)
        self.add_s(l, "Flexion Coude", 0, 25, int(cfg["elbow_flex"]*10), mode, "elbow_flex", 10.0)
        w.setLayout(l); return w

    def add_s(self, layout, name, mn, mx, init, mode, key, div=1.0):
        lbl = QLabel(f"{name}: {init/div}")
        s = QSlider(Qt.Horizontal); s.setRange(mn, mx); s.setValue(init)
        s.valueChanged.connect(lambda v: [lbl.setText(f"{name}: {v/div}"), self.node.configs[mode].update({key: v/div})])
        layout.addWidget(lbl); layout.addWidget(s)

    def launch_rviz(self):
        subprocess.Popen("sudo pkill -9 -f robot_state_publisher", shell=True)
        subprocess.Popen(["ros2", "launch", "asterassembly_description", "display.launch.py"])

def main():
    rclpy.init(); node = AsterProEngine(); app = QApplication(sys.argv); win = AsterIHM(node)
    win.show(); t = QTimer(); t.timeout.connect(lambda: [rclpy.spin_once(node, timeout_sec=0), node.update_motion()])
    t.start(20); sys.exit(app.exec_())

if __name__ == "__main__": main()
