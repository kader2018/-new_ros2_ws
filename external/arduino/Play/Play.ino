#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include "HUSKYLENS.h"

// --- CONFIGURATION DU MATÉRIEL ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HUSKYLENS huskylens;

// --- CAPTEURS INFRAROUGES DES MAINS ---
#define PIN_IR_G  22
#define PIN_IR_G2 23
#define PIN_IR_D  24
#define PIN_IR_D2 25

// --- CONFIGURATION SERVOS (Bornes microsecondes de ta Calibration) ---
#define SERVO_COUNT   16
#define SERVO_MIN_US  500     
#define SERVO_MAX_US  2500   

int currentPos[SERVO_COUNT];
int targetPos[SERVO_COUNT];

// Suivi de l'état réel de santé des composants I2C
bool bnoConnected = false;
bool loxConnected = false;
bool huskyConnected = false;
bool telemetryEnabled = true;

// --- TIMING ---
unsigned long lastSensorMs = 0;
const int sensorInterval = 100;

// Conversion d'angle (0-180) en Microsecondes (Calibration précise)
int degToUS(int deg) {
  return map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}

// Lecture sécurisée du capteur thermique Omron D6T
float readOmron() {
  Wire.beginTransmission(0x0A);
  Wire.write(0x4C);
  if (Wire.endTransmission() != 0) return 0.0;
  Wire.requestFrom(0x0A, 35);
  if (Wire.available() < 35) return 0.0;

  uint8_t buf[35];
  for (int i = 0; i < 35; i++) buf[i] = Wire.read();
  return ((buf[1] << 8) | buf[0]) * 0.1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000); // Vitesse I2C standard (100kHz) plus stable en cas de parasites moteurs

  // Initialisation du shield Servo PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initialisation des capteurs infrarouges (Pins physiques directes, hors I2C)
  pinMode(PIN_IR_G, INPUT);  pinMode(PIN_IR_G2, INPUT);
  pinMode(PIN_IR_D, INPUT);  pinMode(PIN_IR_D2, INPUT);

  // Initialisation BNO055 sans blocage
  if (bno.begin()) {
    bnoConnected = true;
  } else {
    Serial.println("S;ERR_BNO055");
  }

  // Initialisation VL53L0X sans blocage
  if (lox.begin()) {
    loxConnected = true;
  } else {
    Serial.println("S;ERR_VL53L0X");
  }

  // Initialisation Huskylens NON-BLOQUANTE (Suppression du 'while')
  if (huskylens.begin(Wire)) {
    huskyConnected = true;
  } else {
    Serial.println("S;ERR_HUSKYLENS");
  }

  // Positions initiales de repos (90° partout au démarrage pour des raisons de sécurité)
  for (int i = 0; i < SERVO_COUNT; i++) {
    currentPos[i] = 90;
    targetPos[i] = 90;
    pwm.writeMicroseconds(i, degToUS(currentPos[i]));
  }
  
  Serial.println("ASTER_READY");
}

// Décodage intelligent capable de lire "S[id]:[angle]" ET "M;v0;v1;...;v15" ET "I;id;min;max;rest"
void handleSerialCommands() {
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line == "TELEMETRY_OFF") {
      telemetryEnabled = false;
      continue;
    }

    if (line == "TELEMETRY_ON") {
      telemetryEnabled = true;
      continue;
    }

    // --- NOUVEAU FORMAT : SÉQUENCE INITIALISATION PROGRESSIVE INDIVIDUELLE (I;id;min;max;rest) ---
    if (line.charAt(0) == 'I') {
      int p1 = line.indexOf(';', 2);
      int p2 = line.indexOf(';', p1 + 1);
      int p3 = line.indexOf(';', p2 + 1);
      
      if (p1 > 0 && p2 > 0 && p3 > 0) {
        int id = line.substring(2, p1).toInt();
        int minAngle = line.substring(p1 + 1, p2).toInt();
        int maxAngle = line.substring(p2 + 1, p3).toInt();
        int restAngle = line.substring(p3 + 1).toInt();

        if (id >= 0 && id < SERVO_COUNT && minAngle != maxAngle) {
          int steps = 80;
          int delayMs = 30; // Vitesse contrôlée et ultra-fluide

          // Mouvement 1 : Repos -> Max
          for (int step = 0; step <= steps; step++) {
            int current = restAngle + (maxAngle - restAngle) * step / steps;
            pwm.writeMicroseconds(id, degToUS(constrain(current, 0, 180)));
            delay(delayMs);
          }
          delay(150);

          // Mouvement 2 : Max -> Min
          for (int step = 0; step <= steps; step++) {
            int current = maxAngle + (minAngle - maxAngle) * step / steps;
            pwm.writeMicroseconds(id, degToUS(constrain(current, 0, 180)));
            delay(delayMs);
          }
          delay(150);

          // Mouvement 3 : Min -> Repos
          for (int step = 0; step <= steps; step++) {
            int current = minAngle + (restAngle - minAngle) * step / steps;
            pwm.writeMicroseconds(id, degToUS(constrain(current, 0, 180)));
            delay(delayMs);
          }
          
          // Mise à jour de l'état pour éviter les sursauts ultérieurs
          targetPos[id] = restAngle;
          currentPos[id] = restAngle;
          pwm.writeMicroseconds(id, degToUS(restAngle));
          delay(200);
        }
        // Envoi du signal d'acquittement à Python pour passer au servo suivant
        Serial.println("INIT_DONE");
      }
    }

    // --- FORMAT MULTI-SERVO (M;v0;v1;...;v15) --- Utilisé par l'IHM vocale et le bridge ROS2
    else if (line.charAt(0) == 'M') {
      int servoIdx = 0;
      int startIdx = 2; // Saute le "M;"
      
      while (startIdx < line.length() && servoIdx < SERVO_COUNT) {
        int nextSemiColon = line.indexOf(';', startIdx);
        String valStr;
        
        if (nextSemiColon == -1) {
          valStr = line.substring(startIdx);
          startIdx = line.length(); // Fin de la ligne
        } else {
          valStr = line.substring(startIdx, nextSemiColon);
          startIdx = nextSemiColon + 1;
        }
        
        if (valStr.length() > 0) {
          int angle = valStr.toInt();
          targetPos[servoIdx] = constrain(angle, 0, 180);
        }
        servoIdx++;
      }
    }
    
    // --- ANCIEN FORMAT SINGLE SERVO (S[id]:[angle]) --- Retenu pour la rétrocompatibilité
    else if (line.charAt(0) == 'S') {
      int sepIdx = line.indexOf(':');
      if (sepIdx > 0) {
        int id = line.substring(1, sepIdx).toInt();
        int angle = line.substring(sepIdx + 1).toInt();
        if (id >= 0 && id < SERVO_COUNT) {
          targetPos[id] = constrain(angle, 0, 180);
        }
      }
    }
  }
}

// Mise à jour instantanée des positions sans réécrire le PWM si rien ne change
void updateServoMovements() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (currentPos[i] != targetPos[i]) {
      currentPos[i] = targetPos[i];
      pwm.writeMicroseconds(i, degToUS(currentPos[i]));
    }
  }
}

// Envoi de la télémétrie à ROS (avec contournement si un capteur a échoué)
void sendSensorsToROS() {
  int dist = 999;
  if (loxConnected) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) dist = measure.RangeMilliMeter;
  }

  int irState = (digitalRead(PIN_IR_G) << 3) |
                (digitalRead(PIN_IR_G2) << 2) |
                (digitalRead(PIN_IR_D) << 1) | digitalRead(PIN_IR_D2);

  int iaID = 0, iaX = -1, iaY = -1;
  if (huskyConnected && huskylens.request(20)) {
    if (huskylens.available()) {
      HUSKYLENSResult res = huskylens.read();
      iaID = res.ID; iaX = res.xCenter; iaY = res.yCenter;
    }
  }

  float tempOmron = readOmron();
  float pitch = 0.0, roll = 0.0, yaw = 0.0;
  if (bnoConnected) {
    sensors_event_t imuEvent;
    bno.getEvent(&imuEvent);
    pitch = imuEvent.orientation.y;
    roll  = imuEvent.orientation.z;
    yaw   = imuEvent.orientation.x;
  }

  Serial.print("S;");
  Serial.print(dist);       Serial.print(";");
  Serial.print(irState);    Serial.print(";");
  Serial.print(iaID);       Serial.print(";");
  Serial.print(iaX);        Serial.print(";");
  Serial.print(iaY);        Serial.print(";");
  Serial.print(tempOmron);  Serial.print(";");
  Serial.print(pitch);      Serial.print(";");
  Serial.print(roll);       Serial.print(";");
  Serial.println(yaw);
}

void loop() {
  handleSerialCommands();
  updateServoMovements();
  
  unsigned long currentMs = millis();
  if (telemetryEnabled && currentMs - lastSensorMs >= sensorInterval) {
    lastSensorMs = currentMs;
    sendSensorsToROS();
  }
  delay(5);
}
