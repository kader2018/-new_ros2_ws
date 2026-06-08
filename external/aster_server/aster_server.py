import os
import requests
import json
import threading
import subprocess
import time
import serial
import re
from serial import SerialException
from flask import Flask, request, send_from_directory, jsonify
from flask_cors import CORS

API_KEY = os.environ.get("GEMINI_API_KEY", "")
GEMINI_URL = "https://generativelanguage.googleapis.com/v1/models/gemini-2.5-flash:generateContent"

app = Flask(__name__)
CORS(app)

request_lock = threading.Lock()
mode_lock = threading.Lock()
robot_mode = "discussion"

PORT_SERIE = "/dev/ttyACM0"
BAUDRATE = 115200
WS_DIR = "/home/addala/ros2_moveit_ws/"
FOLDER_SRC_ROS = "/home/addala/ros2_moveit_ws/src/asterassembly_description/asterassembly_description/"
SOURCE_COMMAND = "source /opt/ros/jazzy/setup.bash && source install/setup.bash"

ros_processes = {
    "ihm": None,
    "bridge": None,
    "control": None
}

CALIBRATION_ROS2 = "/home/addala/ros2_moveit_ws/src/asterassembly_description/asterassembly_description/servo_calibration.json"

# Codes de statut : 0 = En attente, 1 = En cours de calibration, 2 = Calibration terminée avec succès
init_status_code = 0

def set_robot_mode(mode):
    global robot_mode
    with mode_lock:
        robot_mode = mode

def get_robot_mode():
    with mode_lock:
        return robot_mode

def run_ros_shell(command, timeout=None):
    full_cmd = f"cd {WS_DIR} && {SOURCE_COMMAND} && {command}"
    return subprocess.run(["bash", "-lc", full_cmd], timeout=timeout)

def is_bridge_running():
    proc = ros_processes.get("bridge")
    if proc is not None and proc.poll() is None:
        return True
    result = subprocess.run(["pgrep", "-f", "ros2_serial_bridge.py"], stdout=subprocess.DEVNULL)
    return result.returncode == 0

def ensure_bridge_running():
    if is_bridge_running():
        return True
    env = os.environ.copy()
    env["DISPLAY"] = ":0"
    cmd_bridge = f"cd {WS_DIR} && {SOURCE_COMMAND} && python3 {FOLDER_SRC_ROS}ros2_serial_bridge.py"
    ros_processes["bridge"] = subprocess.Popen(["bash", "-lc", cmd_bridge], env=env)
    time.sleep(1.5)
    return is_bridge_running()

def stop_walk_controller():
    proc = ros_processes.get("control")
    if proc is not None and proc.poll() is None:
        try:
            proc.terminate()
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
    ros_processes["control"] = None
    subprocess.run(["pkill", "-f", "aster_control_center.py --cli"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def publish_aster_cmd(command):
    msg = "{data: '" + command.replace("'", "") + "'}"
    result = run_ros_shell(f'ros2 topic pub --once /aster/cmd std_msgs/msg/String "{msg}"', timeout=6)
    return result.returncode == 0

def release_mode_later(mode, delay_s):
    def _release():
        if get_robot_mode() == mode:
            set_robot_mode("discussion")
    timer = threading.Timer(delay_s, _release)
    timer.daemon = True
    timer.start()

def get_rest_positions_list():
    if not os.path.exists(CALIBRATION_ROS2):
        print("❌ CRITIQUE : Le fichier JSON de calibration est introuvable !")
        return None
    try:
        with open(CALIBRATION_ROS2, 'r', encoding='utf-8') as f:
            config = json.load(f)
        servos_config = config.get("servos", {})
        rest_positions = []
        for i in range(16):
            if isinstance(servos_config, dict):
                rest_pos = servos_config.get(str(i), {}).get("rest_position")
            else:
                rest_pos = servos_config[i].get("rest_position") if i < len(servos_config) else None
                
            if rest_pos is None:
                return None
            rest_positions.append(int(rest_pos))
        return rest_positions
    except:
        return None

def run_tactile_calibration_sequence():
    """
    Parcourt chaque servo l'un après l'autre de manière ULTRA-LENTE.
    Délègue l'interpolation à l'Arduino via l'ordre 'I;id;min;max;rest'
    et attend le signal 'INIT_DONE' de l'Arduino avant de passer au suivant.
    """
    global init_status_code
    if is_bridge_running() or get_robot_mode() in ("marche_ros2", "direct_go"):
        print("Calibration refusée : le bridge ou un mode moteur est actif.")
        init_status_code = 0
        set_robot_mode("discussion")
        return
    set_robot_mode("calibration")
    init_status_code = 1
    print("🔄 [Calibration] Démarrage de la séquence d'étalonnage ULTRA-LENTE...")
    
    base_positions = get_rest_positions_list()
    if base_positions is None:
        print("🛑 Séquence annulée : Valeurs de repos introuvables.")
        init_status_code = 0
        set_robot_mode("discussion")
        return

    try:
        with open(CALIBRATION_ROS2, 'r', encoding='utf-8') as f:
            config = json.load(f)
        servos_config = config.get("servos", {})
        
        # Connexion au port série
        ser = serial.Serial(PORT_SERIE, BAUDRATE, timeout=1.0)
        time.sleep(1.5)

        # Alignement initial global de tous les servos au repos
        trame_init = "M;" + ";".join(map(str, base_positions)) + "\n"
        ser.write(trame_init.encode('utf-8'))
        ser.flush()
        time.sleep(1.0)

        for i in range(16):
            try:
                if isinstance(servos_config, dict):
                    servo_data = servos_config.get(str(i)) or servos_config.get(i) or {}
                else:
                    servo_data = servos_config[i] if i < len(servos_config) else {}

                min_angle = servo_data.get("low_mech_constraint") or servo_data.get("min_angle", 0)
                max_angle = servo_data.get("high_mech_constraint") or servo_data.get("max_angle", 180)
                rest_angle = servo_data.get("rest_position") or base_positions[i]

                min_angle, max_angle, rest_angle = int(min_angle), int(max_angle), int(rest_angle)
                
                if min_angle == max_angle:
                    print(f"⏩ Servo {i:02d}/15 sauté (Angles identiques)")
                    continue

                print(f"🔩 Alignement matériel Servo {i:02d}/15 (Min: {min_angle}°, Max: {max_angle}°, Repos: {rest_angle}°)")
                
                # Nettoyage du buffer avant l'envoi
                ser.reset_input_buffer()
                
                # Envoi de la commande d'initialisation dédiée
                cmd_init = f"I;{i};{min_angle};{max_angle};{rest_angle}\n"
                ser.write(cmd_init.encode('utf-8'))
                ser.flush()

                # Attente bloquante de la fin du mouvement géré par l'Arduino
                start_time = time.time()
                while True:
                    if ser.in_waiting:
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if "INIT_DONE" in response:
                            break
                    # Sécurité de sortie (Timeout de 15 secondes max par servo)
                    if time.time() - start_time > 15.0:
                        print(f"⚠️ Timeout de sécurité atteint sur le servo {i:02d}")
                        break
                    time.sleep(0.02)

            except Exception as e_servo:
                print(f"⚠️ Erreur servo {i}: {e_servo}")
                continue

        print("✅ [Calibration] Tous les servos ont fini. Passage du statut à la FIN (2).")
        init_status_code = 2  # Déclenche la fin sur l'IHM (Retour des yeux actifs + Parole)
        
    except Exception as e:
        print(f"💥 Erreur critique calibration : {e}")
        init_status_code = 0
    finally:
        try:
            ser.close()
        except:
            pass
        set_robot_mode("discussion")

def pass_serial_pose_sequence(pose_base_name):
    print(f"Execution de la pose via ROS : {pose_base_name}")
    return send_serial_pose_sequence(pose_base_name)

def send_serial_pose_sequence(pose_base_name):
    # Les gestes DIRECT_GO passent toujours par ros2_serial_bridge.py.
    if not ensure_bridge_running():
        print("Bridge indisponible : DIRECT_GO annule.")
        return False
    return publish_aster_cmd(f"DIRECT_GO:{pose_base_name}")

def execute_direct_go(pose_name):
    if get_robot_mode() == "calibration":
        return "Calibration en cours, j'attends la fin."
    if not os.path.exists(CALIBRATION_ROS2):
        return "Je ne trouve pas le fichier de calibration."
    stop_walk_controller()
    set_robot_mode("direct_go")
    if not send_serial_pose_sequence(pose_name):
        set_robot_mode("discussion")
        return "Je n'arrive pas a joindre le pont serie."
    release_mode_later("direct_go", 8.0)
    return {
        "bonjour": "Bonjour !",
        "oui": "Oui tout a fait.",
        "non": "Non je ne veux pas.",
        "pense": "Je reflechis."
    }.get(pose_name, "D'accord.")

def execute_robot_command(user_text):
    text_lower = user_text.lower()
    env = os.environ.copy()
    env["DISPLAY"] = ":0"

    direct_commands = {
        "go bonjour": "bonjour",
        "go hello": "bonjour",
        "go oui": "oui",
        "go non": "non",
        "go pense": "pense",
    }
    for key, pose_name in direct_commands.items():
        if key in text_lower:
            return execute_direct_go(pose_name)

    if "go marche" in text_lower or "montre-moi comment tu marches" in text_lower:
        if get_robot_mode() in ("direct_go", "calibration"):
            return "J'attends la fin du mouvement en cours."
        if not os.path.exists(CALIBRATION_ROS2):
            return "Je ne trouve pas la position de repos car le fichier de configuration est absent."
        set_robot_mode("marche_ros2")
        if not ensure_bridge_running():
            set_robot_mode("discussion")
            return "Je n'arrive pas a demarrer le pont serie."
        if ros_processes["control"] is None or ros_processes["control"].poll() is not None:
            print("[Marche] Lancement du Control Center CLI...")
            full_cmd = f"cd {WS_DIR} && {SOURCE_COMMAND} && python3 {FOLDER_SRC_ROS}aster_control_center.py --cli"
            ros_processes["control"] = subprocess.Popen(["bash", "-lc", full_cmd], env=env)
        return "Je te montre comment je marche."

    if "simulation marche" in text_lower:
        if ensure_bridge_running():
            return "Pont serie operationnel."
        return "Je n'arrive pas a demarrer le pont serie."

    if re.search(r"(stop|arrêt|arret|repo)", text_lower):
        stop_walk_controller()
        publish_aster_cmd("REST") if ensure_bridge_running() else None
        set_robot_mode("discussion")
        return "Tout est arrete."

    return None

@app.route("/")
def index(): return send_from_directory(".", "aster_face.html")

@app.route("/trigger_init", methods=["POST"])
def trigger_init():
    global init_status_code
    if init_status_code == 0:
        threading.Thread(target=run_tactile_calibration_sequence).start()
        return jsonify({"status": "started"})
    return jsonify({"status": "busy"})

@app.route("/check_init", methods=["GET"])
def check_init():
    global init_status_code
    return jsonify({"code": init_status_code})

@app.route("/clear_history", methods=["POST"])
def clear_history_route():
    return jsonify({"status": "cleared"})

@app.route("/ask", methods=["POST"])
def ask_route():
    data = request.get_json(force=True)
    user_text = data.get("text", "")
    lock_acquired = request_lock.acquire(blocking=False)
    if not lock_acquired: return jsonify({'error': "Requête en cours."}), 429
    try:
        text_clean = user_text.strip()
        robot_reply = execute_robot_command(text_clean)
        if robot_reply: return jsonify({'text': robot_reply})
        if not API_KEY:
            return jsonify({'error': "GEMINI_API_KEY manquante"}), 500
        payload = {"contents": [{"parts": [{"text": text_clean + " (Réponds en une phrase très courte de moins de 15 mots)"}]}]}
        response = requests.post(f"{GEMINI_URL}?key={API_KEY}", json=payload, timeout=15)
        if response.status_code == 200:
            return jsonify({'text': response.json()['candidates'][0]['content']['parts'][0]['text']})
        return jsonify({'error': "Erreur API"}), 500
    except Exception as e: return jsonify({'error': str(e)}), 500
    finally: request_lock.release()

def launch_march_simulation():
    """
    1. Ferme toute connexion série active dans ce script.
    2. Tue les processus ROS2/Bridge existants qui verrouillent le port.
    3. Lance la simulation et le bridge.
    """
    print("🤖 [Marche] Simulation activée - Libération du port série...")
    
    # 1. Fermeture du port série utilisé par le serveur si ouvert
    # (On utilise un bloc try/except car 'ser' n'est peut-être pas défini dans tous les cas)
    try:
        # Si tu as une instance globale appelée 'ser'
        if 'ser' in globals() and ser and ser.is_open:
            ser.close()
            print("🔌 Port série local fermé.")
    except Exception as e:
        print(f"⚠️ Note lors de la fermeture locale : {e}")

    # 2. Kill brutal des processus ROS2 pour s'assurer qu'aucun ne garde le port
    # 'pkill' sur le nom des fichiers Python ou des nodes ROS
    subprocess.run(["pkill", "-f", "ros2_serial_bridge.py"])
    subprocess.run(["pkill", "-f", "robot_state_publisher"])
    subprocess.run(["pkill", "-f", "rviz2"])
    
    # Temps de pause pour laisser le noyau Linux libérer le périphérique /dev/ttyACM0
    time.sleep(2.5) 

    # 3. Lancement de la simulation et du bridge
    print("🚀 Simulation Marche activée - Initialisation ROS2...")
    try:
        # Lancement de Rviz
        subprocess.Popen(["ros2", "launch", "asterassembly_description", "display.launch.py"])
        # Lancement du bridge après une courte attente
        time.sleep(1.0)
        subprocess.Popen(["ros2", "run", "aster_pkg", "ros2_serial_bridge.py"])
        print("✅ Processus de marche lancés.")
    except Exception as e:
        print(f"💥 Erreur lors du lancement des processus ROS2 : {e}")
if __name__ == "__main__":
    # RETRAIT DE LA SÉQUENCE SACCADÉE AUTOMATIQUE ICI : C'est l'IHM tactile qui pilote maintenant la calibration
    app.run(host="0.0.0.0", port=8443, ssl_context=('cert.pem', 'key.pem'), debug=False, use_reloader=False)
