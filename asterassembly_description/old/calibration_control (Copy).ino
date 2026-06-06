/****************************************************
 * ASTER - Firmware de CALIBRATION Servo (PCA9685)
 * 
 * Protocole série (depuis asterassembly_controller.py) :
 *  - "S<ID>:<ANGLE>\n"
 *    ex : "S0:150\n"  -> servo 0 à 150°
 * 
 *  Réponse Arduino (pour l'UI) :
 *    "Servo <ID> -> <ANGLE>"
 * 
 *  - 16 servos pilotés via PCA9685 (addr 0x40)
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// --- CONFIG ---
#define SERVO_COUNT   16
#define SERIAL_BAUD   115200

// Microsecondes min/max pour les servos MG9xx (à adapter si besoin)
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

// État des servos
int currentPos[SERVO_COUNT];
int targetPos[SERVO_COUNT];

// Vitesse max de déplacement pour lisser (en degrés par update)
float max_speed_deg = 5.0;   // calibration = un peu de douceur mais réactif


// Convertit un angle en degrés [0..180] en microsecondes pour le PCA9685
int degToUS(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}


// ===================== SETUP =====================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz pour servos

  // Position initiale : 90° partout (ou ce que tu veux)
  for (int i = 0; i < SERVO_COUNT; i++) {
    currentPos[i] = 90;
    targetPos[i]  = 90;
    int us = degToUS(90);
    pwm.writeMicroseconds(i, us);
  }

  Serial.println("ASTER CALIBRATION FIRMWARE READY");
}


// ===================== LECTURE SERIE =====================
void handleSerial() {
  if (!Serial.available()) return;

  // On lit une ligne complète
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // On s'attend à "S<ID>:<ANGLE>"
  // ex: "S3:150"
  if (line.charAt(0) != 'S') {
    // Commande inconnue -> ignore, mais on pourrait debug
    // Serial.print("Unknown cmd: "); Serial.println(line);
    return;
  }

  // On retire le 'S'
  String rest = line.substring(1); // ex: "3:150"
  int sepIdx = rest.indexOf(':');
  if (sepIdx < 0) {
    // format invalide
    return;
  }

  String idStr    = rest.substring(0, sepIdx);
  String angleStr = rest.substring(sepIdx + 1);

  int id    = idStr.toInt();
  int angle = angleStr.toInt();

  if (id < 0 || id >= SERVO_COUNT) {
    // id invalide
    return;
  }

  // Sécurité : clamp 0..180°
  angle = constrain(angle, 0, 180);

  targetPos[id] = angle;

  // Réponse pour le GUI Python (il cherche les lignes qui commencent par "Servo")
  Serial.print("Servo ");
  Serial.print(id);
  Serial.print(" -> ");
  Serial.println(angle);
}


// ===================== MISE A JOUR SERVOS =====================
void updateServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {

    // calibration = réaction immédiate
    currentPos[i] = targetPos[i];

    int us = degToUS(currentPos[i]);
    pwm.writeMicroseconds(i, us);
  }
}

// ===================== LOOP =====================
void loop() {
  handleSerial();    // traite les commandes venant du GUI
  updateServos();    // bouge les servos vers les cibles
  delay(20);         // ~50 Hz de mise à jour
}
