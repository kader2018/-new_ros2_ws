#!/usr/bin/env python3
import sys, subprocess, math, json, rclpy, os, threading, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

if "--cli" not in sys.argv:
    from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QTabWidget, QLabel, QSlider, QPushButton, QTextEdit
    from PyQt5.QtCore import Qt, QTimer

JN = ["bras_droit_de_ar_joint","bras_droit_h_b_joint","bras_droit_rot_joint","bras_droit_coud_joint","bras_gauche_de_ar_joint","bras_gauche_h_b_joint","bras_gauche_rot_joint","bras_gauche_coude_joint","cuisse_droit_joint","genou_droit_joint","cheville_droite_joint","cuisse_gauche_joint","genou_gauche_joint","cheville_gauche_joint","tete_g_d_joint","tete_h_b_joint"]

def bezier3(t,p0,p1,p2,p3):
    u=1.-t
    return u*u*u*p0+3*u*u*t*p1+3*u*t*t*p2+t*t*t*p3

def smooth_step(t): return t*t*(3.-2.*t)
def ease_out_quad(t): return 1.-(1.-t)*(1.-t)

class AsterKinematics:
    @staticmethod
    def compute_leg(ph, c, d, isl=False):
        f, b, k = math.radians(c["fwd"]), math.radians(c["bwd"]), math.radians(c["knee"])
        cl, ks = math.radians(d['clearance']), math.radians(d.get('knee_safety', 5.))
        
        if ph < math.pi:
            n = ph/math.pi
            hip = b + (f - b) * ease_out_quad(math.pow(n, 1.0 / d['swing_ratio']))
            knee = bezier3(n, ks, k*1.2, k*0.9, ks)
            ankle = -hip * 0.45 + bezier3(n, 0., cl*1.1, cl*0.8, 0.)
            hl = math.radians(d['hip_swing_lat']) * math.sin(n*math.pi)
        else:
            n = (ph-math.pi)/math.pi
            hip = f - (f - b) * smooth_step(math.pow(n, 1.0 / d['stance_power']))
            rs = math.radians(d.get('knee_residual', 4.0)) + ks
            knee = bezier3(n, ks, rs*1.2, rs*0.6, ks)
            ps = math.radians(d.get('ankle_push', 6.0))
            ankle = -hip + bezier3(n, 0.02, 0., -ps*0.4, -ps)
            hl = 0.
            
        final_lat = (math.radians(d['straddle_width']) + hl) * (1 if isl else -1)
        return hip, knee, ankle, final_lat

class AsterNode(Node):
    def __init__(self):
        super().__init__('aster_ctrl')
        self.p = self.create_publisher(JointState, '/joint_states', 10)
        self.pc = self.create_publisher(String, '/aster/cmd', 10)
        self.t, self.at, self.imu = 0., "WALK", {"roll":0.,"pitch":0.}
        self.create_subscription(String, '/aster/imu', self._cb, 10)
        self.dyn = {'clearance':18., 'swing_ratio':1.2, 'stance_power':3., 'knee_residual':4., 'ankle_push':6., 'hip_swing_lat':8., 'straddle_width':5., 'knee_safety':5.}
        
        # S'applique à 1.5 Hz d'allure immédiate s'il tourne via Flask en mode --cli (ordre 'go marche')
        self.cf = {
            "WALK":{"speed":1.5 if "--cli" in sys.argv else 0.0, "fwd":25.,"bwd":-8.,"knee":35.,"arm_amp":0.25,"elbow_flex":0.25},
            "RUN":{"speed":2.5 if "--cli" in sys.argv else 0.0, "fwd":45.,"bwd":-15.,"knee":65.,"arm_amp":0.9,"elbow_flex":0.8}
        }
        
        self.lp = [0.] * len(JN)
        try:
            json_path = os.path.expanduser("~/ros2_moveit_ws/src/asterassembly_description/asterassembly_description/servo_calibration.json")
            if os.path.exists(json_path):
                with open(json_path, 'r') as f:
                    calib_data = json.load(f)
                servos_dict = calib_data.get("servos", {})
                name_to_angle = {}
                for idx_str, s_info in servos_dict.items():
                    name_to_angle[s_info["name"]] = math.radians(s_info.get("rest_position", 90.0))
                self.lp = [name_to_angle.get(name, 0.0) for name in JN]
                self.get_logger().info("✅ Positions de repos chargées en radians.")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur chargement JSON : {str(e)}")

    def _cb(self, m):
        try: self.imu = json.loads(m.data)
        except: pass

    def update(self):
        c = self.cf[self.at]
        if c["speed"] > 0.01:
            self.t += 0.02; p = (2.*math.pi*c["speed"]*self.t)%(2.*math.pi)
            hR, kR, aR, lR = AsterKinematics.compute_leg(p, c, self.dyn, False)
            hL, kL, aL, lL = AsterKinematics.compute_leg((p+math.pi)%(2.*math.pi), c, self.dyn, True)
            asw = c["arm_amp"]*math.sin(p+math.pi)
            
            d = {n:0. for n in JN}
            d.update({
                "cuisse_droit_joint":hR,"genou_droit_joint":kR,"cheville_droite_joint":aR,
                "cuisse_gauche_joint":hL,"genou_gauche_joint":kL,"cheville_gauche_joint":aL,
                "bras_droit_h_b_joint":lR,"bras_gauche_h_b_joint":lL,
                "bras_droit_de_ar_joint":asw,"bras_gauche_de_ar_joint":-asw,
                "bras_droit_coud_joint":c["elbow_flex"],"bras_gauche_coude_joint":c["elbow_flex"]
            })
            self.lp = [d[n] for n in JN]

    def pub(self): 
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg(); m.name, m.position = JN, self.lp; self.p.publish(m)

if "--cli" not in sys.argv:
    class AsterUI(QWidget):
        def __init__(self, node): super().__init__(); self.n = node; self.init_ui()
        def init_ui(self):
            self.setWindowTitle("ASTER V61 - Console d'ajustement"); self.setStyleSheet("background:#0f172a;color:white;"); l = QHBoxLayout(self); left = QVBoxLayout(); self.ts = QTabWidget()
            self.ts.addTab(self.mk_tab("WALK"),"MARCHE"); self.ts.addTab(self.mk_tab("RUN"),"COURSE")
            bw = QWidget(); bl = QVBoxLayout(bw)
            for n,k in [("Déhanchement",'hip_swing_lat'),("Écartement",'straddle_width'),("Sécurité Genou",'knee_safety'),("Pointe Pied",'clearance')]: self.asl(bl,n,0,30,k,1.,True)
            self.ts.addTab(bw,"BIO"); left.addWidget(self.ts); r = QHBoxLayout()
            b1 = QPushButton("RVIZ"); b1.clicked.connect(self.rvz); r.addWidget(b1)
            b2 = QPushButton("RESET IMU"); b2.clicked.connect(lambda: self.n.pc.publish(String(data="RESET_IMU"))); r.addWidget(b2)
            left.addLayout(r); self.m = QTextEdit(); self.m.setReadOnly(True); self.m.setStyleSheet("background:#020617;color:#10b981;font:9pt monospace;"); l.addLayout(left,1); l.addWidget(self.m,1); self.resize(1100,800)
        def mk_tab(self, m):
            w = QWidget(); l = QVBoxLayout(w)
            for n,k,mi,ma,d in [("Vitesse","speed",0,40,10.),("H.Av","fwd",0,60,1.),("H.Ar","bwd",-30,5,1.),("Gen","knee",0,80,1.)]: self.asl(l,n,mi,ma,k,d,False,m)
            return w
        def asl(self, lay, name, mn, mx, key, div, isd, m=None):
            h = QHBoxLayout(); lb = QLabel(name); s = QSlider(Qt.Horizontal); s.setRange(int(mn*div), int(mx*div))
            v = self.n.dyn[key] if isd else self.n.cf[m][key]
            s.setValue(int(v*div)); lb.setText(f"{name}: {v}")
            def ch(val):
                nv = val/div; lb.setText(f"{name}: {nv}")
                target = self.n.dyn if isd else self.n.cf[m]
                target[key] = nv
            s.valueChanged.connect(ch); lay.addWidget(lb); lay.addWidget(s)
        def ref(self):
            self.n.at = "WALK" if self.ts.currentIndex() == 0 else "RUN"
            t = "".join([f"{n:<22}: {math.degrees(self.n.lp[i]):>5.1f}°\n" for i, n in enumerate(JN)])
            self.m.setText(f"{t}\nRoll: {self.n.imu.get('roll',0):.1f} {self._tb(self.n.imu.get('roll',0))}\nPitch: {self.n.imu.get('pitch',0):.1f} {self._tb(self.n.imu.get('pitch',0))}")
        def _tb(self, a, w=20):
            m=w//2; s=int(min(abs(a)/15.*m, m)); b=[' ']*w; b[m]='|'
            idx = range(m,min(m+s,w)) if a>=0 else range(max(m-s,0),m+1)
            for i in idx: b[i]='█'
            return '['+''.join(b)+']'
        def rvz(self):
            subprocess.Popen("sudo pkill -9 -f robot_state_publisher",shell=True)
            subprocess.Popen(["ros2","launch","asterassembly_description","display.launch.py"])

def main():
    rclpy.init()
    n = AsterNode()
    
    if "--cli" in sys.argv:
        print("[Marche] Mode CLI : publication /joint_states active.")
        try:
            while rclpy.ok():
                n.update()
                n.pub()
                rclpy.spin_once(n, timeout_sec=0.0)
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            n.destroy_node()
            rclpy.shutdown()
    else:
        app = QApplication(sys.argv)
        ui = AsterUI(n)
        ui.show()
        t = QTimer()
        t.timeout.connect(lambda:[rclpy.spin_once(n,timeout_sec=0),n.update(),n.pub(),ui.ref()])
        t.start(20)
        sys.exit(app.exec_())

if __name__ == "__main__":
    main()
