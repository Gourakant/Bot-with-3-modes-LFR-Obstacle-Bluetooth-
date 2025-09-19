# Bot-with-3-modes-LFR-Obstacle-Bluetooth-
// ==================== Mode Selection ====================
#define MODE0 A5
#define MODE1 A6

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
int motorSpeed = 200;

// ==================== Obstacle Avoiding Setup (Mode 2) ====================
#define SERVO_PIN 3
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 200
#define SAFE_DISTANCE 15
#define MOTOR_SPEED 150       // <--- Move this here globally
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

  Serial.begin(9600);
  bt.begin(9600);

  // Servo for obstacle avoider
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  delay(500);
}

// ==================== Loop ====================
void loop() {
  int m0 = digitalRead(MODE0);
  int m1 = digitalRead(MODE1);

  if (m0 == LOW && m1 == LOW) {
    runLFR();
  }
  else if (m0 == LOW && m1 == HIGH) {
    runObstacleAvoiding();
  }
  else if (m0 == HIGH && m1 == LOW) {
    runBluetooth();
  }
  else {
    stopAll();
  }
}

// ======================================================
// =============== MODE 1: LFR FUNCTIONS ================
// ======================================================
void runLFR() {
  int s[5] = {
    !digitalRead(IR1), !digitalRead(IR2),
    !digitalRead(IR3), !digitalRead(IR4),
    !digitalRead(IR5)
  };

  // Finish Line
  if (s[0] && s[1] && s[2] && s[3] && s[4]) {
    if (finishLineStartTime == 0) {
      finishLineStartTime = millis();
    } else if (millis() - finishLineStartTime >= 2000 && !finishLineDetected) {
      finishLineDetected = true;
      currentState = FINISH;
    }
  } else {
    finishLineStartTime = 0;
  }

  if (!finishLineDetected) {
    if (s[0] && s[4] && millis() - lastIntersectionTime > 500) {
      currentState = T_SECTION;
      lastIntersectionTime = millis();
    } 
    else if ((s[0] || s[1]) && !s[2] && !s[3] && !s[4]) currentState = L_TURN_LEFT;
    else if ((s[3] || s[4]) && !s[0] && !s[1] && !s[2]) currentState = L_TURN_RIGHT;
    else if (s[1] || s[3]) currentState = CURVE;
    else if (s[2]) currentState = NORMAL;
    else currentState = LOST;
  }

  switch (currentState) {
    case T_SECTION: followRightL(s); break;
    case L_TURN_LEFT: followLeftL(s); break;
    case L_TURN_RIGHT: followRightL(s); break;
    case CURVE: followCurve(s[1], s[3]); break;
    case NORMAL: forwardLFR(); break;
    case LOST: searchLine(); break;
    case FINISH: stopAtFinish(); break;
  }
}

void forwardLFR() {
  analogWrite(EN1, baseSpeed);
  analogWrite(EN2, baseSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void followLeftL(int s[5]) {
  analogWrite(EN1, sharpTurnSpeed);
  analogWrite(EN2, sharpTurnSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  delay(400);
}

void followRightL(int s[5]) {
  analogWrite(EN1, sharpTurnSpeed);
  analogWrite(EN2, sharpTurnSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(400);
}

void followCurve(bool leftActive, bool rightActive) {
  int speedLeft = curveSpeed, speedRight = curveSpeed;
  if (leftActive) speedRight -= 30;
  if (rightActive) speedLeft -= 30;
  analogWrite(EN1, speedLeft);
  analogWrite(EN2, speedRight);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void searchLine() {
  analogWrite(EN1, 100);
  analogWrite(EN2, 80);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopAtFinish() {
  stopAll();
  while (true);
}

// ======================================================
// =========== MODE 2: OBSTACLE AVOIDING ================
// ======================================================
void runObstacleAvoiding() {
  int distance = getDistance();

  if (distance > SAFE_DISTANCE) {
    moveForward(MOTOR_SPEED);
  } else {
    stopAll();
    delay(100);
    moveBackward(MOTOR_SPEED);
    delay(300);
    stopAll();
    delay(100);

    myServo.write(150); delay(400);
    int leftD = getDistance();

    myServo.write(30); delay(400);
    int rightD = getDistance();

    myServo.write(90); delay(200);

    if (leftD > SAFE_DISTANCE || rightD > SAFE_DISTANCE) {
      if (leftD >= rightD) turnLeft();
      else turnRight();
    } else {
      moveBackward(MOTOR_SPEED);
      delay(500);
      stopAll();
    }
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE;
  return cm;
}

void moveForward(int spd) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN1, spd); analogWrite(EN2, spd);
}

void moveBackward(int spd) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN1, spd); analogWrite(EN2, spd);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopAll();
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopAll();
}

// ======================================================
// =============== MODE 3: BLUETOOTH ====================
// ======================================================
void runBluetooth() {
  if (bt.available()) {
    btData = bt.read();
    Serial.println(btData);

    switch (btData) {
      case 'F': moveForward(motorSpeed); break;
      case 'B': moveBackward(motorSpeed); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopAll(); break;
      case '1': motorSpeed = 100; break;
      case '2': motorSpeed = 150; break;
      case '3': motorSpeed = 200; break;
      case '4': motorSpeed = 255; break;
    }
  }
}

// ======================================================
// =============== Shared Stop Function =================
// ======================================================
void stopAll() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
