#!/usr/bin/env python3
import sys, subprocess, math, json, os, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer

CONFIG_FILE = "aster_config.json"
JOINT_NAMES = [
    "bras_droit_de_ar_joint", "bras_droit_h_b_joint", "bras_droit_rot_joint", "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint", "bras_gauche_h_b_joint", "bras_gauche_rot_joint", "bras_gauche_coude_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint",
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "tete_g_d_joint", "tete_h_b_joint"
]

# ==========================================================
# 1. MOTEUR CINÉMATIQUE
# ==========================================================
class AsterKinematics:
    @staticmethod
    def compute_leg(phase, cfg, dyn):
        f_rad, b_rad, k_rad = math.radians(cfg["fwd"]), math.radians(cfg["bwd"]), math.radians(cfg["knee"])
        clearance = math.radians(dyn['clearance'])
        
        if phase < math.pi: # SWING
            n = phase / math.pi
            n_c = math.pow(n, 1.0 / max(0.1, dyn['swing_ratio']))
            hip = b_rad + (f_rad - b_rad) * n_c
            knee = k_rad * math.sin(n * math.pi)
            ankle = -hip + clearance * math.sin(n * math.pi)
        else: # STANCE
            n = (phase - math.pi) / math.pi
            n_c = math.pow(n, dyn['stance_power'])
            hip = f_rad - (f_rad - b_rad) * n_c
            knee = 0.0
            ankle = -hip
        return hip, knee, ankle

# ==========================================================
# 2. NOEUD ROS AVEC SAUVEGARDE JSON
# ==========================================================
class AsterNode(Node):
    def __init__(self):
        super().__init__('aster_controller')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.t = 0.0
        self.active_tab = "WALK"
        
        # Valeurs par défaut (si pas de fichier config)
        self.inv = {'glob': 1.0, 'br_r': 1.0, 'cd_r': 1.0, 'br_l': -1.0, 'cd_l': 1.0}
        self.dyn = {'clearance': 27.0, 'swing_ratio': 1.0, 'stance_power': 4.0}
        self.configs = {
            "WALK": {"speed": 0.0, "fwd": 13.0, "bwd": -55.0, "knee": 61.0, "arm_amp": 0.3, "elbow_flex": 0.3},
            "RUN":  {"speed": 0.0, "fwd": 110.0, "bwd": -25.0, "knee": 100.0, "arm_amp": 1.2, "elbow_flex": 1.1}
        }
        
        self.load_config() # Charger la sauvegarde si elle existe
        self.last_pos = [0.0] * len(JOINT_NAMES)
        self.sent_deg = [999] * len(JOINT_NAMES)

    def save_config(self):
        data = {'inv': self.inv, 'dyn': self.dyn, 'configs': self.configs}
        with open(CONFIG_FILE, 'w') as f:
            json.dump(data, f)

    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    data = json.load(f)
                    self.inv.update(data.get('inv', {}))
                    self.dyn.update(data.get('dyn', {}))
                    self.configs.update(data.get('configs', {}))
            except: pass

    def update_engine(self):
        cfg = self.configs[self.active_tab]
        d = {n: 0.0 for n in JOINT_NAMES}
        if cfg["speed"] > 0.001:
            self.t += 0.02
            p = (2.0 * math.pi * cfg["speed"] * self.t) % (2.0 * math.pi)
            hR, kR, aR = AsterKinematics.compute_leg(p, cfg, self.dyn)
            hL, kL, aL = AsterKinematics.compute_leg((p + math.pi) % (2.0 * math.pi), cfg, self.dyn)
            d.update({
                "cuisse_droit_joint": hR * self.inv['glob'], "genou_droit_joint": kR, "cheville_droite_joint": aR,
                "cuisse_gauche_joint": hL * self.inv['glob'], "genou_gauche_joint": kL, "cheville_gauche_joint": aL,
                "bras_droit_de_ar_joint": cfg["arm_amp"] * math.sin(p) * self.inv['br_r'],
                "bras_gauche_de_ar_joint": cfg["arm_amp"] * math.sin(p + math.pi) * self.inv['br_l'],
                "bras_droit_coud_joint": cfg["elbow_flex"] * self.inv['cd_r'],
                "bras_gauche_coude_joint": cfg["elbow_flex"] * self.inv['cd_l']
            })
        self.last_pos = [d[n] for n in JOINT_NAMES]

    def publish(self):
        curr_deg = [int(round(math.degrees(p))) for p in self.last_pos]
        if any(abs(curr_deg[i] - self.sent_deg[i]) >= 1 for i in range(len(curr_deg))):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [math.radians(deg) for deg in curr_deg]
            self.pub.publish(msg)
            self.sent_deg = curr_deg

# ==========================================================
# 3. INTERFACE UTILISATEUR
# ==========================================================
class AsterUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ASTER V57 - PERSISTENT ARCHI")
        self.setStyleSheet("background-color: #0f172a; color: white;")
        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()

        # 1. Dynamique
        g_dyn = QGroupBox("Dynamique (Auto-Save)")
        l_dyn = QVBoxLayout()
        self.add_slider(l_dyn, "Pointe Pied (Deg)", 0, 45, 'clearance', 1.0, True)
        self.add_slider(l_dyn, "Swing Ratio", 10, 100, 'swing_ratio', 10.0, True)
        self.add_slider(l_dyn, "Stance Power", 5, 50, 'stance_power', 10.0, True)
        g_dyn.setLayout(l_dyn); left_panel.addWidget(g_dyn)

        # 2. Inversions
        g_inv = QGroupBox("Inversions Manuelles")
        l_inv = QHBoxLayout()
        for label, key in [("GLOB", 'glob'), ("BR R", 'br_r'), ("CD R", 'cd_r'), ("BR L", 'br_l'), ("CD L", 'cd_l')]:
            btn = QPushButton(label); btn.setCheckable(True)
            if self.node.inv[key] == -1.0: btn.setChecked(True)
            btn.clicked.connect(lambda c, k=key: [self.node.inv.update({k: -1.0 if c else 1.0}), self.node.save_config()])
            l_inv.addWidget(btn)
        g_inv.setLayout(l_inv); left_panel.addWidget(g_inv)

        # 3. Onglets
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_tab("WALK"), "MARCHE")
        self.tabs.addTab(self.create_tab("RUN"), "COURSE")
        self.tabs.currentChanged.connect(lambda i: setattr(self.node, 'active_tab', "WALK" if i==0 else "RUN"))
        left_panel.addWidget(self.tabs)

        # 4. Actions Shell
        l_act = QHBoxLayout()
        for t, f in [("RVIZ", self.launch_rviz), ("CONNECT", self.launch_phys)]:
            b = QPushButton(t); b.clicked.connect(f); l_act.addWidget(b)
        left_panel.addLayout(l_act)

        # 5. Monitor
        self.monitor = QTextEdit(); self.monitor.setReadOnly(True)
        self.monitor.setStyleSheet("background-color: #020617; color: #10b981; font-family: monospace;")
        
        main_layout.addLayout(left_panel, 1); main_layout.addWidget(self.monitor, 1)
        self.setLayout(main_layout); self.resize(1150, 850)

    def create_tab(self, mode):
        w = QWidget(); l = QVBoxLayout()
        cfg = self.node.configs[mode]
        controls = [("Vitesse", 0, 40, "speed", 10.0), ("Hanche Av", 0, 150, "fwd", 1.0),
                    ("Hanche Ar", -90, 30, "bwd", 1.0), ("Genou", 0, 150, "knee", 1.0),
                    ("Amp. Épaule", 0, 20, "arm_amp", 10.0), ("Flex. Coude", 0, 25, "elbow_flex", 10.0)]
        for lbl, mn, mx, key, div in controls:
            self.add_slider(l, lbl, mn, mx, key, div, False, mode)
        w.setLayout(l); return w

    def add_slider(self, layout, name, mn, mx, key, div, is_dyn, mode=None):
        val = self.node.dyn[key] if is_dyn else self.node.configs[mode][key]
        lbl = QLabel(f"{name}: {val}"); s = QSlider(Qt.Horizontal)
        s.setRange(mn, mx); s.setValue(int(val * div))
        def change(v):
            nv = v/div; lbl.setText(f"{name}: {nv}")
            if is_dyn: self.node.dyn[key] = nv
            else: self.node.configs[mode][key] = nv
            self.node.save_config() # Sauvegarde à chaque mouvement
        s.valueChanged.connect(change); layout.addWidget(lbl); layout.addWidget(s)

    def refresh_ui(self):
        txt = "".join([f"{n:<22}: {int(round(math.degrees(self.node.last_pos[i]))):>4}°\n" for i, n in enumerate(JOINT_NAMES)])
        self.monitor.setText(txt)

    def launch_rviz(self):
        subprocess.Popen("sudo pkill -9 -f robot_state_publisher", shell=True)
        subprocess.Popen(["ros2", "launch", "asterassembly_description", "display.launch.py"])

    def launch_phys(self):
        subprocess.Popen(["ros2", "run", "asterassembly_description", "servo_bridge"])

def main():
    rclpy.init(); node = AsterNode(); app = QApplication(sys.argv); ui = AsterUI(node); ui.show()
    t_f = QTimer(); t_f.timeout.connect(lambda: [rclpy.spin_once(node, timeout_sec=0), node.update_engine(), ui.refresh_ui()]); t_f.start(20)
    t_s = QTimer(); t_s.timeout.connect(node.publish); t_s.start(40)
    sys.exit(app.exec_())

if __name__ == "__main__": main()
