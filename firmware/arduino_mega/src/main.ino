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

// Pin definitions - adjust for your wiring
// Motor driver pins (L298N or similar)
const int LEFT_MOTOR_EN = 5;   // PWM pin for left motor enable
const int LEFT_MOTOR_IN1 = 22; // Direction pin 1
const int LEFT_MOTOR_IN2 = 23; // Direction pin 2
const int RIGHT_MOTOR_EN = 6;  // PWM pin for right motor enable
const int RIGHT_MOTOR_IN1 = 24;
const int RIGHT_MOTOR_IN2 = 25;

// Claw servo pin
const int CLAW_SERVO_PIN = 9;

// Encoder pins (optional)
const int LEFT_ENC_A = 2;  // Interrupt pin
const int LEFT_ENC_B = 3;
const int RIGHT_ENC_A = 18; // Interrupt pin
const int RIGHT_ENC_B = 19;

// Battery voltage divider pin (analog)
const int BATTERY_PIN = A0;

// E-stop button pin
const int ESTOP_PIN = 52;

// Claw servo positions
const int CLAW_OPEN_POS = 30;
const int CLAW_CLOSED_POS = 120;

// Watchdog timeout (ms)
const unsigned long WATCHDOG_TIMEOUT = 250;

// Telemetry rate (ms)
const unsigned long TELEMETRY_PERIOD = 50;

// Global state
Servo clawServo;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
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
  while (!Serial) {
    ; // Wait for serial connection
  }

  // Motor pins
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Stop motors initially
  stopMotors();

  // Claw servo
  clawServo.attach(CLAW_SERVO_PIN);
  clawServo.write(CLAW_OPEN_POS);

  // Encoder pins with pullup
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // E-stop pin
  pinMode(ESTOP_PIN, INPUT_PULLUP);

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

  // Check E-stop
  estopActive = (digitalRead(ESTOP_PIN) == LOW);

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
  if (cmd.length() < 3) return;

  char cmdType = cmd.charAt(0);
  if (cmd.charAt(1) != ',') return;

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

  // Map position to servo angle
  int angle;
  switch (mode) {
    case 0: // Open
      angle = CLAW_OPEN_POS;
      break;
    case 1: // Close
      angle = CLAW_CLOSED_POS;
      break;
    case 2: // Hold (use position)
      angle = map(currentClawPos, 0, 1000, CLAW_OPEN_POS, CLAW_CLOSED_POS);
      break;
    default:
      angle = CLAW_OPEN_POS;
  }

  clawServo.write(angle);
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
  int batteryMV = map(rawADC, 0, 1023, 0, 5000) * 2; // x2 for voltage divider

  // Format: T,<battery_mv>,<left_enc>,<right_enc>,<estop>
  Serial.print("T,");
  Serial.print(batteryMV);
  Serial.print(",");
  Serial.print(leftEncoderCount);
  Serial.print(",");
  Serial.print(rightEncoderCount);
  Serial.print(",");
  Serial.println(estopActive ? "1" : "0");
}

// Encoder interrupt handlers
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}
