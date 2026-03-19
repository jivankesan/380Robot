/**
 * @file main.ino
 * @brief Arduino Mega 2560 firmware for 380Robot motor and claw control.
 *
 * Serial Protocol (115200 baud, ASCII, newline terminated):
 *
 * Commands (ROS -> Arduino):
 *   M,<left_pwm>,<right_pwm>\n   Motor command, PWM in [-255, 255]
 *   C,<mode>,<pos>\n             Claw command, mode 0=open/1=close/2=hold, pos 0-1000
 *
 * Telemetry (Arduino -> ROS):
 *   T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
 *   E,<code>,<message>\n         Error message
 */

#include <Servo.h>

// Pin definitions for Elegoo Smart Car Shield (TB6612 driver)
const int RIGHT_MOTOR_EN = 6;   // PWMB
const int RIGHT_MOTOR_IN1 = 7;  // BIN1
const int RIGHT_MOTOR_IN2 = 8;  // BIN2
const int LEFT_MOTOR_EN = 5;    // PWMA
const int LEFT_MOTOR_IN1 = 12;  // AIN1
const int LEFT_MOTOR_IN2 = 13;  // AIN2
const int MOTOR_STBY = 3;       // Standby - must be HIGH to enable motors

// Claw servo pins
const int CLAW_ROTATION_SERVO_PIN = 10;  // Servo 1: rotation axis
const int CLAW_GRIPPER_SERVO_PIN = 9;    // Servo 2: open/close gripper

// Battery voltage divider pin (analog)
const int BATTERY_PIN = A0;

// E-stop not used on Uno (pin 52 is Mega-only)
const int ESTOP_PIN = -1;

// Servo 2 (gripper) angles
const int CLAW_OPEN_POS = 70;
const int CLAW_CLOSED_POS = 130;

// Servo 1 (rotation) angles
const int CLAW_HORIZONTAL_POS = 20;
const int CLAW_ROTATED_POS = 90;

// Watchdog timeout (ms)
const unsigned long WATCHDOG_TIMEOUT = 250;

// Telemetry rate (ms)
const unsigned long TELEMETRY_PERIOD = 50;

// Global state
Servo clawRotationServo;
Servo clawGripperServo;
int currentLeftPWM = 0;
int currentRightPWM = 0;
int currentClawMode = 0;
int currentClawPos = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
bool estopActive = false;
String inputBuffer = "";

void setup() {
  // Initialize serial
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Enable TB6612 (take out of standby)
  digitalWrite(MOTOR_STBY, HIGH);

  // Stop motors initially
  stopMotors();

  // Claw servos — start in open/horizontal position
  clawRotationServo.attach(CLAW_ROTATION_SERVO_PIN);
  clawRotationServo.write(CLAW_HORIZONTAL_POS);
  clawGripperServo.attach(CLAW_GRIPPER_SERVO_PIN);
  clawGripperServo.write(CLAW_OPEN_POS);

  // Encoders not used with Elegoo Smart Car Shield
  // (encoder pins conflict with motor driver pins on this shield)

  // E-stop not used

  // Battery monitoring
  pinMode(BATTERY_PIN, INPUT);

  lastCommandTime = millis();
  lastTelemetryTime = millis();

  Serial.println("E,0,Arduino ready");
}

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

  // E-stop not used on Uno
  estopActive = false;

  // Watchdog - stop motors if no commands received
  unsigned long now = millis();
  if (now - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (currentLeftPWM != 0 || currentRightPWM != 0) {
      stopMotors();
    }
  }

  // E-stop override
  if (estopActive) {
    stopMotors();
  }

  // Send telemetry
  if (now - lastTelemetryTime >= TELEMETRY_PERIOD) {
    sendTelemetry();
    lastTelemetryTime = now;
  }
}

void processCommand(String cmd) {
  if (cmd.length() < 3)
    return;

  char cmdType = cmd.charAt(0);
  if (cmd.charAt(1) != ',')
    return;

  String params = cmd.substring(2);

  switch (cmdType) {
    case 'M':
      processMotorCommand(params);
      break;
    case 'C':
      processClawCommand(params);
      break;
    default:
      Serial.println("E,1,Unknown command");
  }
}

void processMotorCommand(String params) {
  // Parse: <left_pwm>,<right_pwm>
  int commaIdx = params.indexOf(',');
  if (commaIdx < 0) {
    Serial.println("E,2,Invalid motor command");
    return;
  }

  int leftPWM = params.substring(0, commaIdx).toInt();
  int rightPWM = params.substring(commaIdx + 1).toInt();

  // Clamp values
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (!estopActive) {
    setMotors(leftPWM, rightPWM);
    lastCommandTime = millis();
  }
}

void processClawCommand(String params) {
  // Parse: <mode>,<pos>
  int commaIdx = params.indexOf(',');
  if (commaIdx < 0) {
    Serial.println("E,3,Invalid claw command");
    return;
  }

  int mode = params.substring(0, commaIdx).toInt();
  int pos = params.substring(commaIdx + 1).toInt();

  currentClawMode = mode;
  currentClawPos = constrain(pos, 0, 1000);

  // Drive both servos based on mode
  int gripperAngle;
  int rotationAngle;
  switch (mode) {
    case 0:  // Open — gripper open, rotation horizontal
      gripperAngle = CLAW_OPEN_POS;
      rotationAngle = CLAW_HORIZONTAL_POS;
      break;
    case 1:  // Close — gripper closed, rotation rotated (carry position)
      gripperAngle = CLAW_CLOSED_POS;
      rotationAngle = CLAW_ROTATED_POS;
      break;
    case 2:  // Hold — interpolate gripper, keep rotation at carry position
      gripperAngle = map(currentClawPos, 0, 1000, CLAW_OPEN_POS, CLAW_CLOSED_POS);
      rotationAngle = CLAW_ROTATED_POS;
      break;
    default:
      gripperAngle = CLAW_OPEN_POS;
      rotationAngle = CLAW_HORIZONTAL_POS;
  }

  clawGripperServo.write(gripperAngle);
  clawRotationServo.write(rotationAngle);
  lastCommandTime = millis();
}

void setMotors(int leftPWM, int rightPWM) {
  currentLeftPWM = leftPWM;
  currentRightPWM = rightPWM;

  // Left motor
  if (leftPWM >= 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, leftPWM);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, -leftPWM);
  }

  // Right motor
  if (rightPWM >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, rightPWM);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, -rightPWM);
  }
}

void stopMotors() {
  currentLeftPWM = 0;
  currentRightPWM = 0;
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void sendTelemetry() {
  // Read battery voltage
  // Assuming voltage divider: Vbat -> 10k -> A0 -> 10k -> GND
  // So A0 reads Vbat/2
  int rawADC = analogRead(BATTERY_PIN);
  int batteryMV = map(rawADC, 0, 1023, 0, 5000) * 2;  // x2 for voltage divider

  // Format: T,<battery_mv>,<left_enc>,<right_enc>,<estop>
  Serial.print("T,");
  Serial.print(batteryMV);
  Serial.print(",0,0,");
  Serial.println(estopActive ? "1" : "0");
}
