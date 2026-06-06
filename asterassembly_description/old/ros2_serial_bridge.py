import math, json, rclpy, serial, os
from pathlib import Path
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range, Temperature
from geometry_msgs.msg import Point

SCRIPT_DIR = Path(__file__).parent.absolute()
CALIB_PATH = SCRIPT_DIR / "servo_calibration.json"
SERIAL_PORT = "/dev/ttyACM0" if os.path.exists("/dev/ttyACM0") else "/dev/ttyACM1"
BAUDRATE, BRIDGE_DT, MAX_SPEED_DEG_S = 115200, 0.05, 15.0

class AsterSerialBridge(Node):
    def __init__(self):
        super().__init__("aster_serial_bridge")
        self.joint_names, self.index_from_name = [], {}
        self.servo_rest, self.servo_min, self.servo_max = np.array([]), np.array([]), np.array([])
        self._load_calib()
        self.current_cmd = self.servo_rest.copy().astype(np.float64)
        self.target_cmd = self.servo_rest.copy().astype(np.float64)
        self.last_sent_cmd = self.servo_rest.copy().astype(np.int64)
        self.max_delta = MAX_SPEED_DEG_S * BRIDGE_DT
        self.p_laser = self.create_publisher(Range, "/aster/sensor/sonar", 10)
        self.p_temp = self.create_publisher(Temperature, "/aster/sensor/temp", 10)
        self.p_vision = self.create_publisher(Point, "/aster/sensor/eye", 10)
        self.p_ir = self.create_publisher(Point, "/aster/sensor/proximite", 10)
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
            self.get_logger().info(f"Connecté sur {SERIAL_PORT}")
        except: self.ser = None
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.create_timer(BRIDGE_DT, self.timer_cb)

    def _load_calib(self):
        if not CALIB_PATH.exists(): return
        with open(CALIB_PATH, 'r') as f:
            servos = json.load(f).get("servos", {})
            keys = sorted(servos.keys(), key=lambda x: int(x))
            self.servo_rest, self.servo_min, self.servo_max = np.zeros(len(keys)), np.zeros(len(keys)), np.zeros(len(keys))
            for i, k in enumerate(keys):
                s = servos[k]; name = s.get("name", f"s_{k}")
                self.joint_names.append(name); self.index_from_name[name] = i
                self.servo_rest[i] = float(s.get("rest_position", 90))
                self.servo_min[i] = float(s.get("low_mech_constraint", 0))
                self.servo_max[i] = float(s.get("high_mech_constraint", 180))

    def joint_cb(self, msg):
        target = self.servo_rest.copy()
        n2i = {n: i for i, n in enumerate(msg.name)}
        for jn, si in self.index_from_name.items():
            if jn in n2i:
                # Inversion logique jambe gauche / jambe droite
                s = -1.0 if "gauche" in jn.lower() else 1.0
                val = self.servo_rest[si] + (s * math.degrees(msg.position[n2i[jn]]))
                target[si] = np.clip(val, self.servo_min[si], self.servo_max[si])
        self.target_cmd = target

    def timer_cb(self):
        if self.ser:
            while self.ser.in_waiting:
                try:
                    l = self.ser.readline().decode('ascii', errors='replace').strip()
                    if l.startswith("S;"):
                        p = l.split(";")
                        if len(p) >= 7:
                            self.p_laser.publish(Range(range=float(p[1])/1000.0))
                            ir = int(p[2]); self.p_ir.publish(Point(x=float(ir >> 2), y=float(ir & 0x03)))
                            self.p_vision.publish(Point(z=float(p[3]), x=float(p[4]), y=float(p[5])))
                            self.p_temp.publish(Temperature(temperature=float(p[6])))
                except: pass
            delta = np.clip(self.target_cmd - self.current_cmd, -self.max_delta, self.max_delta)
            self.current_cmd += delta
            cmd_int = np.round(self.current_cmd).astype(np.int64)
            if not np.array_equal(cmd_int, self.last_sent_cmd):
                try:
                    self.ser.write((";".join(map(str, cmd_int)) + "\n").encode("ascii"))
                    self.last_sent_cmd = cmd_int.copy()
                except: pass

def main():
    rclpy.init(); node = AsterSerialBridge()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
