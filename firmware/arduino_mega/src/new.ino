/**
 * @file new.ino
 * @brief Arduino Mega 2560 firmware for 380Robot motor, encoder, and claw control.
 *
 * Serial Protocol (115200 baud, ASCII, newline terminated):
 *
 * Commands (ROS -> Arduino):
 *   M,<left_pwm>,<right_pwm>\n    Motor command, PWM in [-255, 255]
 *                                  Positive = forward (encoder count increases)
 *                                  Negative = reverse (encoder count decreases)
 *   C,<servo>,<angle>\n            Claw command: servo 1=rotation, 2=gripper; angle 0-180 deg
 *   Z\n                            Zero encoder counts
 *
 * Telemetry (Arduino -> ROS):
 *   T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
 *   E,<code>,<message>\n           Error message
 *
 * Notes:
 *   - If forward motion decreases a count, set that wheel's FLIP constant to -1
 */

#include <Servo.h>

// ---- Encoder direction correction ----
// Set to -1 if forward motion gives decreasing counts for that wheel
const int FLIP_LEFT_ENCODER = 1;
const int FLIP_RIGHT_ENCODER = 1;

// ---- Motor pins ----
const int RIGHT_MOTOR_PWM = 3;  // M1 PWM
const int RIGHT_MOTOR_IN1 = 5;
const int RIGHT_MOTOR_IN2 = 4;
const int LEFT_MOTOR_PWM = 11;  // M2 PWM
const int LEFT_MOTOR_IN1 = 7;
const int LEFT_MOTOR_IN2 = 6;

// ---- Encoder pins (A/B channels) ----
const int RIGHT_ENCODER_A = 8;
const int RIGHT_ENCODER_B = 9;
const int LEFT_ENCODER_B = 10;
const int LEFT_ENCODER_A = 2;

// ---- Claw servo pins ----
const int GRIPPER_PIN = 12;   // Servo 2: open/close gripper
const int ROTATION_PIN = 13;  // Servo 1: rotation axis

// ---- Battery voltage divider pin (analog) ----
const int BATTERY_PIN = A0;

// ---- Timing constants ----
const unsigned long WATCHDOG_TIMEOUT = 250;  // ms — stop motors if no command received
const unsigned long TELEMETRY_PERIOD = 50;   // ms — 20 Hz telemetry

// ---- Global state ----
Servo clawRotation;
Servo clawGripper;

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount = 0;

int prevRightState = 0;
int prevLeftState = 0;

int currentLeftPWM = 0;
int currentRightPWM = 0;

unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;

bool estopActive = false;

String inputBuffer = "";

// ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);

  // Encoder pins
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

  // Battery
  pinMode(BATTERY_PIN, INPUT);

  // Claw servos — start in safe position
  clawRotation.attach(ROTATION_PIN);
  clawRotation.write(20);  // horizontal
  clawGripper.attach(GRIPPER_PIN);
  clawGripper.write(50);  // open

  // Seed encoder states
  prevRightState = (digitalRead(RIGHT_ENCODER_A) << 1) | digitalRead(RIGHT_ENCODER_B);
  prevLeftState = (digitalRead(LEFT_ENCODER_A) << 1) | digitalRead(LEFT_ENCODER_B);

  stopMotors();

  lastCommandTime = millis();
  lastTelemetryTime = millis();

  Serial.println("E,0,Arduino ready");
}

// ------------------------------------------------------------------

void loop() {
  // Software quadrature decoding — call as fast as possible
  updateRightEncoder();
  updateLeftEncoder();

  // Serial command parsing
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  unsigned long now = millis();

  // Watchdog — stop motors if no command within timeout
  if (now - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (currentLeftPWM != 0 || currentRightPWM != 0) {
      stopMotors();
    }
  }

  // E-stop override
  if (estopActive) {
    stopMotors();
  }

  // Telemetry
  if (now - lastTelemetryTime >= TELEMETRY_PERIOD) {
    sendTelemetry();
    lastTelemetryTime = now;
  }
}

// ------------------------------------------------------------------

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() < 1)
    return;

  char cmdType = cmd.charAt(0);

  // Commands without a comma (single-char)
  if (cmd.length() == 1 || cmd.charAt(1) != ',') {
    if (cmdType == 'Z') {
      noInterrupts();
      rightEncoderCount = 0;
      leftEncoderCount = 0;
      interrupts();
      Serial.println("E,0,Encoders zeroed");
      return;
    }
    Serial.println("E,1,Unknown command");
    return;
  }

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

  int leftPWM = constrain(params.substring(0, commaIdx).toInt(), -255, 255);
  int rightPWM = constrain(params.substring(commaIdx + 1).toInt(), -255, 255);

  if (!estopActive) {
    setMotors(leftPWM, rightPWM);
    lastCommandTime = millis();
  }
}

void processClawCommand(String params) {
  // Parse: <servo_num>,<angle>  (1=rotation, 2=gripper)
  int commaIdx = params.indexOf(',');
  if (commaIdx < 0) {
    Serial.println("E,3,Invalid claw command");
    return;
  }

  int servoNum = params.substring(0, commaIdx).toInt();
  int angle = constrain(params.substring(commaIdx + 1).toInt(), 0, 180);

  if (servoNum == 1) {
    clawRotation.write(angle);
  } else if (servoNum == 2) {
    clawGripper.write(angle);
  } else {
    Serial.println("E,3,Invalid servo number");
    return;
  }

  lastCommandTime = millis();
}

// ------------------------------------------------------------------

void setMotors(int leftPWM, int rightPWM) {
  currentLeftPWM = leftPWM;
  currentRightPWM = rightPWM;

  // Left motor (M2)
  if (leftPWM >= 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_PWM, leftPWM);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_PWM, -leftPWM);
  }

  // Right motor (M1)
  if (rightPWM >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_PWM, rightPWM);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, -rightPWM);
  }
}

void stopMotors() {
  currentLeftPWM = 0;
  currentRightPWM = 0;
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

// ------------------------------------------------------------------

void sendTelemetry() {
  // Battery: voltage divider Vbat -> 10k -> A0 -> 10k -> GND  =>  A0 = Vbat/2
  int rawADC = analogRead(BATTERY_PIN);
  int batteryMV = map(rawADC, 0, 1023, 0, 5000) * 2;

  long l, r;
  noInterrupts();
  l = leftEncoderCount;
  r = rightEncoderCount;
  interrupts();

  // T,<battery_mv>,<left_enc>,<right_enc>,<estop>
  Serial.print("T,");
  Serial.print(batteryMV);
  Serial.print(",");
  Serial.print(l);
  Serial.print(",");
  Serial.print(r);
  Serial.print(",");
  Serial.println(estopActive ? "1" : "0");
}

// ------------------------------------------------------------------

void updateRightEncoder() {
  int a = digitalRead(RIGHT_ENCODER_A);
  int b = digitalRead(RIGHT_ENCODER_B);
  int currentState = (a << 1) | b;

  int delta = quadratureDelta(prevRightState, currentState);
  if (delta != 0) {
    rightEncoderCount += delta * FLIP_RIGHT_ENCODER;
    prevRightState = currentState;
  }
}

void updateLeftEncoder() {
  int a = digitalRead(LEFT_ENCODER_A);
  int b = digitalRead(LEFT_ENCODER_B);
  int currentState = (a << 1) | b;

  int delta = quadratureDelta(prevLeftState, currentState);
  if (delta != 0) {
    leftEncoderCount += delta * FLIP_LEFT_ENCODER;
    prevLeftState = currentState;
  }
}

// Returns +1, -1, or 0 based on Gray-code quadrature transition
int quadratureDelta(int prevState, int currentState) {
  // Forward:  00->01->11->10->00  (+1)
  if (
    (prevState == 0 && currentState == 1) || (prevState == 1 && currentState == 3) ||
    (prevState == 3 && currentState == 2) || (prevState == 2 && currentState == 0))
    return +1;

  // Reverse:  00->10->11->01->00  (-1)
  if (
    (prevState == 0 && currentState == 2) || (prevState == 2 && currentState == 3) ||
    (prevState == 3 && currentState == 1) || (prevState == 1 && currentState == 0))
    return -1;

  return 0;
}
