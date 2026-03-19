/**
 * @file main.ino
 * @brief Arduino Mega firmware for 380Robot motor and claw control.
 *
 * Serial Protocol (115200 baud, ASCII, newline terminated):
 *
 * Commands (ROS -> Arduino):
 *   M,<left_pwm>,<right_pwm>\n   Motor command, PWM in [-255, 255]
 *   C,<servo>,<angle>\n          Claw command
 *                                  servo 1 = rotation (pin 10), servo 2 = gripper (pin 11)
 *                                  angle in [10, 170]
 *
 * Telemetry (Arduino -> ROS):
 *   T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
 *   E,<code>,<message>\n
 */

#include <Servo.h>

// ── Motor pins (Elegoo Smart Car Shield / TB6612) ──────────────────────────
const int RIGHT_MOTOR_EN  = 6;   // PWMB
const int RIGHT_MOTOR_IN1 = 7;   // BIN1
const int RIGHT_MOTOR_IN2 = 8;   // BIN2
const int LEFT_MOTOR_EN   = 5;   // PWMA
const int LEFT_MOTOR_IN1  = 12;  // AIN1
const int LEFT_MOTOR_IN2  = 13;  // AIN2
const int MOTOR_STBY      = 3;   // must be HIGH to enable motors

// ── Servo pins ─────────────────────────────────────────────────────────────
const int SERVO1_PIN = 10;  // rotation servo
const int SERVO2_PIN = 11;  // gripper servo

// ── Servo start positions ──────────────────────────────────────────────────
const int SERVO1_START = 90;   // rotation level
const int SERVO2_START = 70;   // gripper open

// ── Servo angle limits ─────────────────────────────────────────────────────
const int MIN_ANGLE = 10;
const int MAX_ANGLE = 170;

// ── Misc ───────────────────────────────────────────────────────────────────
const int          BATTERY_PIN       = A0;
const unsigned long WATCHDOG_TIMEOUT = 250;   // ms — motors only
const unsigned long TELEMETRY_PERIOD = 50;    // ms

// ── Global state ───────────────────────────────────────────────────────────
Servo servo1;
Servo servo2;
int   currentLeftPWM  = 0;
int   currentRightPWM = 0;
unsigned long lastMotorCmdTime  = 0;
unsigned long lastTelemetryTime = 0;
bool  estopActive = false;
String inputBuffer = "";

// ──────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_STBY,      OUTPUT);
  pinMode(LEFT_MOTOR_EN,   OUTPUT);
  pinMode(LEFT_MOTOR_IN1,  OUTPUT);
  pinMode(LEFT_MOTOR_IN2,  OUTPUT);
  pinMode(RIGHT_MOTOR_EN,  OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
  stopMotors();

  // Servos — start in open/home position
  servo1.attach(SERVO1_PIN);
  servo1.write(SERVO1_START);
  delay(300);
  servo2.attach(SERVO2_PIN);
  servo2.write(SERVO2_START);

  pinMode(BATTERY_PIN, INPUT);
  lastMotorCmdTime  = millis();
  lastTelemetryTime = millis();

  Serial.println("E,0,Arduino ready");
}

// ──────────────────────────────────────────────────────────────────────────
void loop() {
  // Read serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  estopActive = false;

  // Motor watchdog
  unsigned long now = millis();
  if (now - lastMotorCmdTime > WATCHDOG_TIMEOUT) {
    if (currentLeftPWM != 0 || currentRightPWM != 0) {
      stopMotors();
    }
  }

  // Telemetry
  if (now - lastTelemetryTime >= TELEMETRY_PERIOD) {
    sendTelemetry();
    lastTelemetryTime = now;
  }
}

// ──────────────────────────────────────────────────────────────────────────
void processCommand(String cmd) {
  if (cmd.length() < 3) return;
  char cmdType = cmd.charAt(0);
  if (cmd.charAt(1) != ',') return;
  String params = cmd.substring(2);

  switch (cmdType) {
    case 'M': processMotorCommand(params); break;
    case 'C': processClawCommand(params);  break;
    default:  Serial.println("E,1,Unknown command");
  }
}

// ──────────────────────────────────────────────────────────────────────────
void processMotorCommand(String params) {
  int commaIdx = params.indexOf(',');
  if (commaIdx < 0) { Serial.println("E,2,Invalid motor command"); return; }

  int leftPWM  = constrain(params.substring(0, commaIdx).toInt(), -255, 255);
  int rightPWM = constrain(params.substring(commaIdx + 1).toInt(), -255, 255);

  if (!estopActive) {
    setMotors(leftPWM, rightPWM);
    lastMotorCmdTime = millis();
  }
}

// ──────────────────────────────────────────────────────────────────────────
void processClawCommand(String params) {
  // Format: C,<servo>,<angle>
  //   servo 1 = rotation (pin 10), servo 2 = gripper (pin 11)
  //   angle clamped to [MIN_ANGLE, MAX_ANGLE]
  int commaIdx = params.indexOf(',');
  if (commaIdx < 0) { Serial.println("E,3,Invalid claw command"); return; }

  int servoNum = params.substring(0, commaIdx).toInt();
  int angle    = constrain(params.substring(commaIdx + 1).toInt(), MIN_ANGLE, MAX_ANGLE);

  if (servoNum == 1) {
    servo1.write(angle);
    Serial.print("Servo1 -> "); Serial.println(angle);
  } else if (servoNum == 2) {
    servo2.write(angle);
    Serial.print("Servo2 -> "); Serial.println(angle);
  } else {
    Serial.println("E,3,Invalid servo number (use 1 or 2)");
  }
}

// ──────────────────────────────────────────────────────────────────────────
void setMotors(int leftPWM, int rightPWM) {
  currentLeftPWM  = leftPWM;
  currentRightPWM = rightPWM;

  if (leftPWM >= 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH); digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, leftPWM);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW); digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, -leftPWM);
  }

  if (rightPWM >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH); digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, rightPWM);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW); digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, -rightPWM);
  }
}

void stopMotors() {
  currentLeftPWM = currentRightPWM = 0;
  digitalWrite(LEFT_MOTOR_IN1, LOW); digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW); digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void sendTelemetry() {
  int rawADC    = analogRead(BATTERY_PIN);
  int batteryMV = map(rawADC, 0, 1023, 0, 5000) * 2;
  Serial.print("T,"); Serial.print(batteryMV);
  Serial.print(",0,0,"); Serial.println(estopActive ? "1" : "0");
}
