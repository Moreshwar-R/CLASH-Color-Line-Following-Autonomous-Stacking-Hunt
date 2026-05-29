// ================================================================
// CLASH BOT — Stage 2
// Full autonomous logic — detect, pick, navigate, place
// Hardware : ESP32 · TCS3200 · 8-array IR · TB6612FNG · SG90 · MG995
// ================================================================

#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// ----------------------------------------------------------------
// Pin Definitions
// ----------------------------------------------------------------

// Motor driver (TB6612FNG)
#define PWMA   12
#define AIN1   14
#define AIN2   27
#define PWMB   13
#define BIN1   26
#define BIN2   25
#define STBY   33

// Colour sensor (TCS3200)
#define S0       17
#define S1       16
#define S2        2
#define S3       19
#define OUT_PIN  21

// Ultrasonic sensor (HC-SR04)
#define TRIG_PIN  22
#define ECHO_PIN  23

// Servo pins
#define GRIPPER_PIN  15
#define LIFT_PIN     18

// ----------------------------------------------------------------
// Line Sensor Configuration
// ----------------------------------------------------------------

QTRSensors qtr;
const uint8_t  SENSOR_COUNT              = 6;
const uint8_t  SENSOR_PINS[SENSOR_COUNT] = {36, 4, 39, 34, 35, 32};
uint16_t       sensorValues[SENSOR_COUNT];
const uint16_t THRESHOLD                 = 4090;

// ----------------------------------------------------------------
// Motion Parameters
// ----------------------------------------------------------------

const int BASE_SPEED   = 120;
const int MAX_SPEED    = 150;
const int TURN_SPEED   = 110;
const int TURN_REVERSE = 100;
const int MIN_SPEED    = 60;

// ----------------------------------------------------------------
// Colour Calibration Ranges
// ----------------------------------------------------------------

const int RED_R1=30,  RED_R2=45,  RED_G1=80,  RED_G2=110, RED_B1=60,  RED_B2=75;
const int BLUE_R1=80, BLUE_R2=110,BLUE_G1=70, BLUE_G2=90, BLUE_B1=35, BLUE_B2=60;

// ----------------------------------------------------------------
// State Machine
// ----------------------------------------------------------------

BluetoothSerial BT;
Servo gripServo, liftServo;

bool systemOn = false;

int phase       = 0;
int redCount    = 0;
int blueCount   = 0;
int totalCount  = 0;
int pickedColor = 0;
int lastPlaced  = 0;

unsigned long lastLineTime   = 0;
unsigned long lastDetectTime = 0;
const unsigned long GAP_TIME = 200;
int lastDir = 0;

// ----------------------------------------------------------------
// Motor Control
// ----------------------------------------------------------------

void drive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  if (abs(left)  < MIN_SPEED && left  != 0) left  = (left  > 0) ? MIN_SPEED : -MIN_SPEED;
  if (abs(right) < MIN_SPEED && right != 0) right = (right > 0) ? MIN_SPEED : -MIN_SPEED;

  digitalWrite(AIN1, left  >= 0); digitalWrite(AIN2, left  < 0);
  digitalWrite(BIN1, right >= 0); digitalWrite(BIN2, right < 0);
  analogWrite(PWMA, abs(left));
  analogWrite(PWMB, abs(right));
}

void stopBot() { drive(0, 0); }

// ----------------------------------------------------------------
// Sensor Reads
// ----------------------------------------------------------------

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long t = pulseIn(ECHO_PIN, HIGH, 4000);
  return (t == 0) ? 999 : (int)(t * 0.034 / 2);
}

void readColour(int &R, int &G, int &B) {
  digitalWrite(S2, LOW);  digitalWrite(S3, LOW);  R = pulseIn(OUT_PIN, LOW, 30000);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); G = pulseIn(OUT_PIN, LOW, 30000);
  digitalWrite(S2, LOW);  digitalWrite(S3, HIGH); B = pulseIn(OUT_PIN, LOW, 30000);
}

// ----------------------------------------------------------------
// Line Utilities
// ----------------------------------------------------------------

void readSensors() { qtr.read(sensorValues); }

bool allWhite() {
  for (int i = 0; i < SENSOR_COUNT; i++) if (sensorValues[i] > THRESHOLD) return false;
  return true;
}

bool allBlack() {
  for (int i = 0; i < SENSOR_COUNT; i++) if (sensorValues[i] < THRESHOLD) return false;
  return true;
}

// ----------------------------------------------------------------
// Movements
// ----------------------------------------------------------------

void uTurn(int dir = 1) {
  stopBot();
  int l = (dir == 1) ?  100 : -100;
  int r = (dir == 1) ? -100 :  100;
  drive(l, r); delay(400);
  while (true) { readSensors(); if (!allWhite()) break; drive(l, r); }
  stopBot();
}

void turnLeft() {
  stopBot();
  drive(BASE_SPEED, BASE_SPEED); delay(300);
  drive(-100, 100);
  while (true) { readSensors(); if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) break; }
  stopBot();
}

void turnRight() {
  stopBot();
  drive(BASE_SPEED, BASE_SPEED); delay(300);
  drive(100, -100);
  while (true) { readSensors(); if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) break; }
  stopBot();
}

// ----------------------------------------------------------------
// Pick & Place
// ----------------------------------------------------------------

void pick() {
  stopBot();
  for (int i = 60;  i >= 26;  i--) { liftServo.write(i); delay(20); }
  for (int i = 80;  i <= 150; i++) { gripServo.write(i); delay(20); }
  for (int i = 26;  i <= 60;  i++) { liftServo.write(i); delay(20); }
  uTurn(1);
  phase = 4;
}

void place() {
  stopBot();
  for (int i = 60;  i >= 26;  i--) { liftServo.write(i); delay(20); }
  for (int i = 150; i >= 80;  i--) { gripServo.write(i); delay(20); }
  for (int i = 26;  i <= 60;  i++) { liftServo.write(i); delay(20); }

  if (pickedColor == 1) redCount--;
  if (pickedColor == 2) blueCount--;
  totalCount--;

  lastPlaced  = pickedColor;
  pickedColor = 0;

  drive(BASE_SPEED, BASE_SPEED); delay(300); stopBot();
  if (lastPlaced == 1) uTurn(1); else uTurn(-1);
  phase = 7;
}

// ----------------------------------------------------------------
// Colour Detection & Action
// ----------------------------------------------------------------

void detectTask() {
  int R, G, B;
  readColour(R, G, B);

  int col = 0;
  if (R>=RED_R1  && R<=RED_R2  && G>=RED_G1  && G<=RED_G2  && B>=RED_B1  && B<=RED_B2)  col = 1;
  if (R>=BLUE_R1 && R<=BLUE_R2 && G>=BLUE_G1 && G<=BLUE_G2 && B>=BLUE_B1 && B<=BLUE_B2) col = 2;

  if (col == 0) return;
  if (millis() - lastDetectTime < 300) return;

  stopBot(); delay(200);

  int d = getDistance();
  if (d > 0 && d < 13) {
    if (phase == 0) {
      if (col == 1) redCount++;
      if (col == 2) blueCount++;
    }
    else if (phase == 1) {
      pickedColor = col;
      pick();
    }
    else if (phase == 6 && col == pickedColor) {
      place();
    }
  }

  lastDetectTime = millis();
}

// ----------------------------------------------------------------
// Line Following
// ----------------------------------------------------------------

void followLine() {
  bool left  = sensorValues[0] > THRESHOLD || sensorValues[1] > THRESHOLD;
  bool mid   = sensorValues[2] > THRESHOLD && sensorValues[3] > THRESHOLD;
  bool right = sensorValues[4] > THRESHOLD || sensorValues[5] > THRESHOLD;

  if (left) {
    drive(-TURN_REVERSE, TURN_SPEED); lastDir = -1; lastLineTime = millis();
  } else if (mid) {
    drive(BASE_SPEED, BASE_SPEED);    lastDir =  0; lastLineTime = millis();
  } else if (right) {
    drive(MAX_SPEED, -MAX_SPEED);     lastDir =  1; lastLineTime = millis();
  } else {
    if (millis() - lastLineTime < GAP_TIME) drive(BASE_SPEED, BASE_SPEED);
    else drive(lastDir == -1 ? -90 : 90, lastDir == 1 ? -90 : 90);
  }
}

// ----------------------------------------------------------------
// Bluetooth
// ----------------------------------------------------------------

void handleBT() {
  if (!BT.available()) return;
  String cmd = BT.readStringUntil('\n'); cmd.trim();
  if (cmd == "ON")  systemOn = true;
  if (cmd == "OFF") { systemOn = false; stopBot(); }
}

// ----------------------------------------------------------------
// Setup & Loop
// ----------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  BT.begin("CLASH_Bot_S2");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);

  gripServo.attach(GRIPPER_PIN);
  liftServo.attach(LIFT_PIN);
  liftServo.write(60);
  gripServo.write(80);
}

void loop() {
  handleBT();
  if (!systemOn) { stopBot(); return; }

  readSensors();
  followLine();

  static unsigned long t = 0;
  if (millis() - t > 40) { detectTask(); t = millis(); }
}
