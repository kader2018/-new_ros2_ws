#!/usr/bin/env python3
import tkinter as tk
import subprocess, os, signal, time

class AsterIHM:
    def __init__(self, root):
        self.root = root
        self.root.title("ASTEЯ CONTROL")
        self.root.geometry("350x600")
        self.root.configure(bg="#1e1e2e")
        self.procs = {}

        # --- TITRE ---
        tk.Label(root, text="ASTEЯ SYSTEM", font=("Arial", 16, "bold"), bg="#1e1e2e", fg="#cba6f7").pack(pady=20)

        # --- BOUTONS DE BASE ---
        self.btn_rviz = tk.Button(root, text="1. VOIR (RViz)", command=self.run_rviz, 
                                 bg="#89b4fa", width=25, height=2)
        self.btn_rviz.pack(pady=5)

        self.btn_bridge = tk.Button(root, text="2. CONNECTER (Physique)", command=self.run_bridge, 
                                   bg="#fab387", width=25, height=2)
        self.btn_bridge.pack(pady=5)

        # --- REGLAGE VITESSE ---
        tk.Label(root, text="VITESSE DE MARCHE (Hz)", bg="#1e1e2e", fg="white", font=("Arial", 10, "bold")).pack(pady=(20, 0))
        self.s_speed = tk.Scale(root, from_=0.1, to=2.0, resolution=0.1, orient="horizontal", 
                                bg="#1e1e2e", fg="white", highlightthickness=0, length=200)
        self.s_speed.set(0.5) # Valeur par défaut
        self.s_speed.pack(pady=5)

        # --- BOUTON MARCHE ---
        self.btn_walk = tk.Button(root, text="3. DÉMARRER MARCHE", command=self.run_walk, 
                                 bg="#a6e3a1", font=("Arial", 12, "bold"), width=25, height=2)
        self.btn_walk.pack(pady=20)

        # --- STOP ---
        tk.Button(root, text="🛑 STOP TOTAL", command=self.stop, 
                  bg="#f38ba8", fg="white", width=25, height=2, font=("Arial", 10, "bold")).pack(pady=20)

    def run_cmd(self, tag, cmd):
        """Lance ou arrête un processus proprement"""
        if tag in self.procs:
            try:
                os.killpg(os.getpgid(self.procs[tag].pid), signal.SIGTERM)
            except:
                pass
            del self.procs[tag]
            return False
        else:
            full_cmd = f"source ~/ros2_moveit_ws/install/setup.bash && {cmd}"
            self.procs[tag] = subprocess.Popen(["bash", "-c", full_cmd], preexec_fn=os.setsid)
            return True

    def run_rviz(self):
        if self.run_cmd("rviz", "ros2 launch asterassembly_description display.launch.py"):
            self.btn_rviz.config(bg="#f9e2af", text="⏹️ FERMER RVIZ")
        else:
            self.btn_rviz.config(bg="#89b4fa", text="1. VOIR (RViz)")

    def run_bridge(self):
        if self.run_cmd("bridge", "ros2 run asterassembly_description servo_bridge"):
            self.btn_bridge.config(bg="#f9e2af", text="⏹️ DÉCONNECTER")
        else:
            self.btn_bridge.config(bg="#fab387", text="2. CONNECTER (Physique)")

    def run_walk(self):
        if "walk" in self.procs:
            self.run_cmd("walk", "")
            self.btn_walk.config(bg="#a6e3a1", text="3. DÉMARRER MARCHE")
        else:
            # Sécurité : si le bridge n'est pas lancé, on le lance
            if "bridge" not in self.procs:
                self.run_bridge()
                time.sleep(1.5)
            
            # On récupère la valeur actuelle du slider
            v = self.s_speed.get()
            cmd = f"ros2 run asterassembly_description quasistatic_walker --ros-args -p step_frequency:={v}"
            
            if self.run_cmd("walk", cmd):
                self.btn_walk.config(bg="#fab387", text="⏹️ ARRÊTER MARCHE")

    def stop(self):
        for t in list(self.procs.keys()):
            try:
                os.killpg(os.getpgid(self.procs[t].pid), signal.SIGTERM)
            except:
                pass
            del self.procs[t]
        self.btn_rviz.config(bg="#89b4fa", text="1. VOIR (RViz)")
        self.btn_bridge.config(bg="#fab387", text="2. CONNECTER (Physique)")
        self.btn_walk.config(bg="#a6e3a1", text="3. DÉMARRER MARCHE")

def main():
    root = tk.Tk()
    app = AsterIHM(root)
    root.mainloop()

if __name__ == "__main__":
    main()
