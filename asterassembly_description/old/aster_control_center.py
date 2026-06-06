#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, json, math, subprocess, rclpy, serial
import numpy as np
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range, Temperature
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor

# --- CONFIGURATION DES CHEMINS & PACKAGES ---
SCRIPT_DIR = Path(__file__).parent.absolute()
CALIB_PATH = SCRIPT_DIR / "servo_calibration.json"
PACKAGE_MOVEIT = "moveit_aster_config"

JOINT_NAMES = [
    "bras_droit_de_ar_joint", "bras_droit_h_b_joint", "bras_droit_rot_joint", "bras_droit_coud_joint",
    "bras_gauche_de_ar_joint", "bras_gauche_h_b_joint", "bras_gauche_rot_joint", "bras_gauche_coude_joint",
    "cuisse_droit_joint", "genou_droit_joint", "cheville_droite_joint",
    "cuisse_gauche_joint", "genou_gauche_joint", "cheville_gauche_joint",
    "tete_g_d_joint", "tete_h_b_joint"
]

# --- POSES SRDF (DÉFINITIONS EXACTES) ---
ASTER_POSES = {
    "REST": [90]*16,
    "READY": [90, 110, 90, 45, 90, 110, 90, 45, 90, 90, 90, 90, 90, 90, 90, 90],
    "WALK_PREP": [90, 90, 90, 90, 90, 90, 90, 90, 75, 45, 105, 75, 45, 105, 90, 90],
    "SIT": [90, 90, 90, 90, 90, 90, 90, 90, 45, 10, 45, 45, 10, 45, 90, 90],
    "SALUTE": [160, 120, 90, 10, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 110, 90]
}

class AsterCoreNode(Node):
    def __init__(self, calib_data):
        super().__init__('aster_core_node')
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        start_pos = [float(calib_data['servos'][str(i)]['rest_position']) for i in range(16)]
        self.current_cmd = np.array(start_pos)
        self.target_cmd = self.current_cmd.copy()
        self.max_delta = 15.0 * 0.05 

    def publish_state(self):
        delta = np.clip(self.target_cmd - self.current_cmd, -self.max_delta, self.max_delta)
        self.current_cmd += delta
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(a - 90) for a in self.current_cmd]
        self.js_pub.publish(msg)
        return self.current_cmd.astype(int)

class AsterUltimateStation(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.ser = None
        self.walk_t = 0.0
        self.inversions = [False] * 16 # Pour les boutons INV individuels
        
        # Chargement JSON complet
        if not CALIB_PATH.exists():
            self.full_cfg = {"servos": {str(i): {"rest_position": 90, "low_mech_constraint": 0, "high_mech_constraint": 180} for i in range(16)}}
        else:
            with open(CALIB_PATH, 'r') as f: self.full_cfg = json.load(f)

        self.init_ui()
        
        # Timers de synchronisation (Bridge + Télémétrie)
        self.timer_main = QTimer(); self.timer_main.timeout.connect(self.main_loop); self.timer_main.start(50)
        self.timer_ser = QTimer(); self.timer_ser.timeout.connect(self.read_serial); self.timer_ser.start(10)
        self.connect_serial()

    def init_ui(self):
        self.setWindowTitle("ASTER STATION v4.0 - FULL LABORATORY INTEGRATION")
        self.resize(1800, 1000)
        self.setStyleSheet("QWidget { background-color: #050505; color: #00ffcc; font-family: 'Consolas'; }")
        
        self.tabs = QTabWidget(); self.setCentralWidget(self.tabs)

        # --- ONGLET 1 : HARDWARE (Calibration complète + Boutons de fonction) ---
        self.setup_hw_tab()
        # --- ONGLET 2 : PILOTAGE DYNAMIQUE (Marche, Poses, Courbe & Niveaux) ---
        self.setup_pilot_tab()

    def setup_hw_tab(self):
        tab = QWidget(); layout = QHBoxLayout(tab)
        
        # Panneau des 16 Sliders
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        content = QWidget(); grid = QGridLayout(content)
        self.sliders, self.pbars = [], []
        
        for i in range(16):
            s_cfg = self.full_cfg['servos'][str(i)]
            grid.addWidget(QLabel(f"ID{i:02d} - {JOINT_NAMES[i]}"), i, 0)
            
            sld = QSlider(Qt.Horizontal); sld.setRange(0, 180)
            sld.setValue(int(self.node.target_cmd[i]))
            sld.valueChanged.connect(lambda v, idx=i: self.update_target(idx, v))
            grid.addWidget(sld, i, 1); self.sliders.append(sld)
            
            pb = QProgressBar(); pb.setRange(0, 180); pb.setFixedWidth(100)
            grid.addWidget(pb, i, 2); self.pbars.append(pb)
            
            # Bouton d'inversion par axe (Demande spécifique)
            inv = QPushButton("INV"); inv.setCheckable(True); inv.setFixedWidth(45)
            inv.clicked.connect(lambda c, idx=i: self.set_inv(idx, c))
            grid.addWidget(inv, i, 3)

        scroll.setWidget(content); layout.addWidget(scroll, 3)
        
        # Panneau latéral de fonctions (calibration_assembly)
        side = QVBoxLayout()
        side.addWidget(QLabel("🛠 OUTILS DE CALIBRATION"))
        
        btn_reconnect = QPushButton("🔌 RECONNEXION SÉRIE"); btn_reconnect.clicked.connect(self.connect_serial)
        btn_save = QPushButton("💾 SAUVEGARDER (JSON)"); btn_save.clicked.connect(self.save_calib)
        btn_reset = QPushButton("🔄 RESET (VALEURS REPOS)"); btn_reset.clicked.connect(self.reset_calib)
        btn_test_s0 = QPushButton("🔎 ENVOYER TEST S0:90"); btn_test_s0.clicked.connect(lambda: self.send_manual("S0:90"))
        btn_test_all = QPushButton("📏 TEST ALL 90°"); btn_test_all.clicked.connect(lambda: [s.setValue(90) for s in self.sliders])
        
        for b in [btn_reconnect, btn_save, btn_reset, btn_test_s0, btn_test_all]: side.addWidget(b)
        
        self.lbl_constraints = QLabel("Contraintes : --")
        side.addWidget(self.lbl_constraints); side.addStretch()
        layout.addLayout(side, 1)
        self.tabs.addTab(tab, "⚙ HARDWARE")

    def setup_pilot_tab(self):
        tab = QWidget(); layout = QGridLayout(tab)
        
        # Section Poses (MoveIt SRDF)
        pose_box = QGroupBox("📍 POSES ENREGISTRÉES")
        pose_l = QHBoxLayout(pose_box)
        for name in ASTER_POSES.keys():
            btn = QPushButton(name); btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, n=name: self.apply_pose(n))
            pose_l.addWidget(btn)
        layout.addWidget(pose_box, 0, 0, 1, 3)

        # Moniteur de Télémétrie (Sonar, Temp, IA)
        self.monitor = QTextEdit(); self.monitor.setReadOnly(True)
        self.monitor.setStyleSheet("background: black; color: #00ff00; font-family: 'Consolas';")
        layout.addWidget(self.monitor, 1, 0, 1, 3)
        
        # Ajustements de Niveaux de Marche (Niveau, Foulée, Clearance)
        adj_box = QGroupBox("📏 PARAMÈTRES D'AJUSTEMENT")
        al = QFormLayout(adj_box)
        self.sld_speed = QSlider(Qt.Horizontal); self.sld_speed.setRange(0, 100); self.sld_speed.setValue(40)
        self.sld_cl = QSlider(Qt.Horizontal); self.sld_cl.setRange(10, 80); self.sld_cl.setValue(30)
        self.sld_fwd = QSlider(Qt.Horizontal); self.sld_fwd.setRange(45, 90); self.sld_fwd.setValue(75)
        self.sld_swing = QSlider(Qt.Horizontal); self.sld_swing.setRange(10, 50); self.sld_swing.setValue(20) # Nouvel ajustement
        
        al.addRow("Vitesse (Cycle):", self.sld_speed)
        al.addRow("Hauteur Pas (Clearance):", self.sld_cl)
        al.addRow("Amplitude Pas (Fwd/Bwd):", self.sld_fwd)
        al.addRow("Niveau Swing (Ajust):", self.sld_swing)
        layout.addWidget(adj_box, 2, 0)
        
        # Commandes de Dynamique
        cmd_box = QGroupBox("🏃 DYNAMIQUE")
        cl = QVBoxLayout(cmd_box)
        self.btn_walk = QPushButton("▶ MARCHE (WALK)"); self.btn_walk.setCheckable(True)
        self.btn_run = QPushButton("⚡ COURSE (RUN)"); self.btn_run.setCheckable(True)
        self.btn_rviz = QPushButton("🚀 LANCER MOVEIT / RVIZ"); self.btn_rviz.clicked.connect(self.launch_moveit)
        
        for b in [self.btn_walk, self.btn_run, self.btn_rviz]: cl.addWidget(b)
        layout.addWidget(cmd_box, 2, 1)

        # Inversion Globale (Optionnelle pour le pilotage)
        inv_box = QGroupBox("🔄 INVERSION GLOBALE")
        il = QVBoxLayout(inv_box)
        self.btn_inv_all = QPushButton("INVERSER TOUT"); self.btn_inv_all.setCheckable(True)
        il.addWidget(self.btn_inv_all); il.addStretch()
        layout.addWidget(inv_box, 2, 2)

        self.tabs.addTab(tab, "🚶 PILOTAGE & AJUSTEMENTS")

    # --- LOGIQUE FONCTIONNELLE INTÉGRALE ---

    def main_loop(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        
        # Moteur de Marche (Logique aster_control_center)
        if self.btn_walk.isChecked() or self.btn_run.isChecked():
            is_run = self.btn_run.isChecked()
            spd = self.sld_speed.value() / (500.0 if not is_run else 250.0)
            self.walk_t += spd
            p = self.walk_t % (2 * math.pi)
            
            cfg = {"fwd": self.sld_fwd.value(), "bwd": 180 - self.sld_fwd.value(), "knee": 45}
            cl = math.radians(self.sld_cl.value())
            
            # Jambe D (8, 9)
            f, k = self.compute_gait(p, cfg, cl)
            self.node.target_cmd[8], self.node.target_cmd[9] = math.degrees(f), math.degrees(k)
            # Jambe G (11, 12) avec déphasage PI
            fg, kg = self.compute_gait((p + math.pi) % (2*math.pi), cfg, cl)
            self.node.target_cmd[11], self.node.target_cmd[12] = math.degrees(fg), math.degrees(kg)

        # Update Progress bars & MoveIt
        cmds = self.node.publish_state()
        for i in range(16): self.pbars[i].setValue(int(cmds[i]))
        
        # Envoi Série avec gestion des inversions (Individuelles + Globale)
        if self.ser and self.ser.is_open:
            out = []
            for i, c in enumerate(cmds):
                val = c
                if self.inversions[i] or self.btn_inv_all.isChecked():
                    val = 180 - c
                out.append(val)
            self.ser.write((";".join(map(str, out)) + "\n").encode())

    def compute_gait(self, phase, cfg, cl):
        # La courbe labo exacte : math.pow(n, 1.0/1.2)
        f_rad, b_rad, k_rad = math.radians(cfg["fwd"]), math.radians(cfg["bwd"]), math.radians(cfg["knee"])
        if phase < math.pi: # Phase de Swing
            n = phase / math.pi
            n_c = math.pow(n, 1.0/1.2)
            h = math.sin(phase) * cl
            return f_rad + (b_rad - f_rad) * n_c, k_rad - h
        else: # Phase d'appui (Stance)
            return b_rad + (f_rad - b_rad) * ((phase - math.pi) / math.pi), k_rad

    def read_serial(self):
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line.startswith("S;"):
                    p = line.split(";")
                    # Parsing intégral : Sonar, Temp, IA
                    txt = f"--- TÉLÉMÉTRIE LABO ---\n"
                    txt += f"📏 SONAR : {p[1]} mm\n"
                    txt += f"🌡 TEMP  : {p[6]} °C\n"
                    txt += f"👁 IA ID : {p[3]} (X:{p[4]}, Y:{p[5]})\n"
                    self.monitor.setText(txt)
            except: pass

    def apply_pose(self, name):
        angles = ASTER_POSES[name]
        for i in range(16): self.sliders[i].setValue(angles[i])
        self.log(f"POSE : {name} appliquée.")

    def update_target(self, idx, val):
        s = self.full_cfg['servos'][str(idx)]
        val = np.clip(val, s['low_mech_constraint'], s['high_mech_constraint'])
        self.node.target_cmd[idx] = val
        self.lbl_constraints.setText(f"ID{idx} : {s['low_mech_constraint']}° | {s['high_mech_constraint']}°")

    def connect_serial(self):
        try:
            if self.ser: self.ser.close()
            p = "/dev/ttyACM0" if os.path.exists("/dev/ttyACM0") else "/dev/ttyACM1"
            self.ser = serial.Serial(p, 115200, timeout=0.01)
            self.log(f"SÉRIE : Connecté sur {p}")
        except: self.log("SÉRIE : Échec de connexion port ACM")

    def save_calib(self):
        for i in range(16): self.full_cfg['servos'][str(i)]['rest_position'] = int(self.node.current_cmd[i])
        with open(CALIB_PATH, 'w') as f: json.dump(self.full_cfg, f, indent=4)
        self.log("SYSTÈME : JSON mis à jour.")

    def reset_calib(self):
        for i in range(16): self.sliders[i].setValue(self.full_cfg['servos'][str(i)]['rest_position'])

    def set_inv(self, i, checked): self.inversions[i] = checked

    def launch_moveit(self):
        subprocess.Popen(f"ros2 launch {PACKAGE_MOVEIT} demo.launch.py use_gui:=false &", shell=True)

    def send_manual(self, cmd):
        if self.ser: self.ser.write((cmd + "\n").encode())

    def log(self, m): self.monitor.append(f"> {m}")

def main(args=None):
    rclpy.init(args=args)
    if not CALIB_PATH.exists():
        calib = {"servos": {str(i): {"rest_position": 90, "low_mech_constraint":0, "high_mech_constraint":180} for i in range(16)}}
    else:
        with open(CALIB_PATH, 'r') as f: calib = json.load(f)
    
    node = AsterCoreNode(calib)
    app = QApplication(sys.argv)
    ihm = AsterUltimateStation(node)
    ihm.show()
    try: sys.exit(app.exec_())
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
