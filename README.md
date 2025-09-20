# Bot-with-3-modes-LFR-Obstacle-Bluetooth-

// ==================== Mode Selection Pins ====================
#define MODE0 0  // D0
#define MODE1 1  // D1

// ==================== Shared Motor Driver Pins ====================
#define IN1 4
#define IN2 5
#define IN3 2
#define IN4 7
#define EN1 6
#define EN2 11

// ==================== Libraries ====================
#include <Servo.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

// ==================== Bluetooth Setup (Mode 3) ====================
#define BT_RX 12
#define BT_TX 13
SoftwareSerial bt(BT_RX, BT_TX);
char btData;
int motorSpeed = 120;

// ==================== Obstacle Avoiding Setup (Mode 2) ====================
#define SERVO_PIN 3
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 200
#define SAFE_DISTANCE 15
#define MOTOR_SPEED 150       // Motor speed for obstacle avoiding
Servo myServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// ==================== LFR Setup (Mode 1) ====================
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4

const int baseSpeed = 130;
const int sharpTurnSpeed = 120;
const int curveSpeed = 110;
const int reverseSpeed = 120;

enum TrackState { NORMAL, L_TURN_LEFT, L_TURN_RIGHT, T_SECTION, CURVE, LOST, FINISH };
TrackState currentState = NORMAL;

unsigned long lastIntersectionTime = 0;
unsigned long finishLineStartTime = 0;
bool finishLineDetected = false;
bool isTurning = false;

// ==================== Setup ====================
void setup() {
  pinMode(MODE0, INPUT_PULLUP);
  pinMode(MODE1, INPUT_PULLUP);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT); pinMode(EN2, OUTPUT);

  // Servo for obstacle avoider
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  delay(500);

  // Initialize Bluetooth
  bt.begin(9600);
}

// ==================== Main Loop ====================
void loop() {
  int m0 = digitalRead(MODE0);
  int m1 = digitalRead(MODE1);

  if (m0 == LOW && m1 == LOW) {
    runLFR();
  }
  else if (m0 == LOW && m1 == HIGH) {
    runObstacleAvoiding();
  }
  else if (m0 == HIGH && m1 == HIGH) {
    runBluetooth();
  }
  else {
    stopAll();  // HIGH, LOW → unused
  }
}

// ======================================================
// =============== MODE 1: LFR FUNCTIONS ================
// ======================================================
void runLFR() {
int s[5] = {
    !digitalRead(IR1),  // Leftmost
    !digitalRead(IR2),  // Left mid
    !digitalRead(IR3),  // Center
    !digitalRead(IR4),  // Right mid
    !digitalRead(IR5)   // Rightmost
  };

  // === Finish Line Detection ===
  if (s[0] && s[1] && s[2] && s[3] && s[4]) {
    if (finishLineStartTime == 0) {
      finishLineStartTime = millis();  // Start timing
    } else if (millis() - finishLineStartTime >= 2000 && !finishLineDetected) {
      finishLineDetected = true;
      currentState = FINISH;
    }
  } else {
    finishLineStartTime = 0;  // Reset timer if condition breaks
  }

  // === Determine Track State ===
  if (!finishLineDetected) {
    if (s[0] && s[4] && millis() - lastIntersectionTime > 500) {
      currentState = T_SECTION;
      lastIntersectionTime = millis();
    } 
    else if ((s[0] || s[1]) && !s[2] && !s[3] && !s[4]) {
      currentState = L_TURN_LEFT;
    } 
    else if ((s[3] || s[4]) && !s[0] && !s[1] && !s[2]) {
      currentState = L_TURN_RIGHT;
    } 
    else if (s[1] || s[3]) {
      currentState = CURVE;
    } 
    else if (s[2]) {
      currentState = NORMAL;
    } 
    else {
      currentState = LOST;
    }
  }

  // === Execute State Action ===
  switch (currentState) {
    case T_SECTION:
      handleTSection(s);
      break;
    case L_TURN_LEFT:
      followLeftL(s);
      break;
    case L_TURN_RIGHT:
      followRightL(s);
      break;
    case CURVE:
      followCurve(s[1], s[3]);
      break;
    case NORMAL:
      forward();
      break;
    case LOST:
      searchLine();
      break;
    case FINISH:
      stopAtFinish();
      break;
  }
}

void forward() {
  analogWrite(EN1, baseSpeed);
  analogWrite(EN2, baseSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void followLeftL(int s[5]) {
  if (!isTurning) {
    isTurning = true;
    
    analogWrite(EN1, sharpTurnSpeed);
    analogWrite(EN2, sharpTurnSpeed);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left reverse
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right forward

    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (!digitalRead(IR3)) break;
    }
    
    forward();
    delay(100);
    isTurning = false;
  }
}

void followRightL(int s[5]) {
  if (!isTurning) {
    isTurning = true;
    
    analogWrite(EN1, sharpTurnSpeed);
    analogWrite(EN2, sharpTurnSpeed);
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left forward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right reverse

    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (!digitalRead(IR3)) break;
    }
    
    forward();
    delay(100);
    isTurning = false;
  }
}

void handleTSection(int s[5]) {
  // Default right turn
  followRightL(s);
  delay(300);
}

void followCurve(bool leftActive, bool rightActive) {
  int speedLeft = curveSpeed;
  int speedRight = curveSpeed;

  if (leftActive)  speedRight -= 30;
  if (rightActive) speedLeft -= 30;

  analogWrite(EN1, speedLeft);
  analogWrite(EN2, speedRight);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// ✅ Improved Search Line function
void searchLine() {
  static unsigned long spinStartTime = 0;
  static bool spinLeft = true;

  int s[5] = {
    !digitalRead(IR1),
    !digitalRead(IR2),
    !digitalRead(IR3),
    !digitalRead(IR4),
    !digitalRead(IR5)
  };

  // ✅ If line is found → exit search mode
  if (s[0] || s[1] || s[2] || s[3] || s[4]) {
    forward();
    spinStartTime = 0;   // reset timer
    return;
  }

  // === Line not found → continue spinning ===
  if (spinStartTime == 0) {
    spinStartTime = millis();  // start timing
  }

  if (spinLeft) {
    // spin left
    analogWrite(EN1, 100);
    analogWrite(EN2, 80);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left backward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right forward
  } else {
    // spin right
    analogWrite(EN1, 100);
    analogWrite(EN2, 80);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left forward
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right backward
  }

  // After 1.5s → change direction
  if (millis() - spinStartTime >= 1500) {
    spinLeft = !spinLeft;       // flip direction
    spinStartTime = millis();   // reset timer
  }
}

void stopAtFinish() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

  Serial.println("Finish line detected. Robot stopped.");

  while (true);  // Stop permanently
}

// ======================================================
// =========== MODE 2: OBSTACLE AVOIDING ================
// ======================================================
void runObstacleAvoiding() {
  int distance = getDistance();

  if (distance > SAFE_DISTANCE) {
    // No obstacle → move forward
    moveForward(MOTOR_SPEED);
  } else {
    // Obstacle detected → stop and move backward briefly
    stopMotors();
    delay(100);
    moveBackward(MOTOR_SPEED);
    delay(300);
    stopMotors();
    delay(100);

    // Scan left
    myServo.write(150);
    delay(400);
    int distanceLeft = getDistance();

    // Scan right
    myServo.write(30);
    delay(400);
    int distanceRight = getDistance();

    // Return to center
    myServo.write(90);
    delay(200);

    // Decide direction
    if (distanceLeft > SAFE_DISTANCE || distanceRight > SAFE_DISTANCE) {
      if (distanceLeft >= distanceRight) {
        turnLeft();
      } else {
        turnRight();
      }
    } else {
      // Both sides blocked → move backward again
      moveBackward(MOTOR_SPEED);
      delay(500);
      stopMotors();
    }
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE;
  return cm;
}


void moveForward(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}

// ======================================================
// =============== MODE 3: BLUETOOTH ====================
// ======================================================
void runBluetooth() {
  if (bt.available()) {
    btData = bt.read();
    switch (btData) {
      case 'F': moveForward(motorSpeed); break;
      case 'B': moveBackward(motorSpeed); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopAll(); break;
      case '1': motorSpeed = 90; break;
      case '2': motorSpeed = 92; break;
      case '3': motorSpeed = 94; break;
      case '4': motorSpeed = 96; break;
    }
  }
}

// ======================================================
// =============== Shared Stop Function =================
// ======================================================
void stopAll() {
  analogWrite(EN1, 0); analogWrite(EN2, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
