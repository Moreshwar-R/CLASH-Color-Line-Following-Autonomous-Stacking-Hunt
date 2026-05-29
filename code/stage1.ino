// ================================================================
// CLASH BOT — Stage 1
// Line following + colour detection + autonomous pick sequence
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

const int BASE_SPEED   = 90;
const int TURN_SPEED   = 110;
const int TURN_REVERSE = 100;
const int MIN_SPEED    = 60;

// ----------------------------------------------------------------
// Colour Calibration Ranges
// ----------------------------------------------------------------

struct ColourRange { int rMin, rMax, gMin, gMax, bMin, bMax; };

const ColourRange RED   = {30,  45,  80, 110, 60, 75};
const ColourRange GREEN = {50,  70,  45,  65, 55, 75};
const ColourRange BLUE  = {80, 110,  70,  90, 35, 60};

// ----------------------------------------------------------------
// Servo Positions
// ----------------------------------------------------------------

const int GRIP_OPEN  = 80;
const int GRIP_CLOSE = 150;
const int LIFT_UP    = 60;
const int LIFT_DOWN  = 26;

// ----------------------------------------------------------------
// State
// ----------------------------------------------------------------

BluetoothSerial BT;
Servo gripServo, liftServo;

bool systemOn = false;
bool picked   = false;

unsigned long lastLineTime     = 0;
const unsigned long GAP_TIME   = 400;
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
  digitalWrite(S2, LOW);  digitalWrite(S3, LOW);  R = pulseIn(OUT_PIN, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); G = pulseIn(OUT_PIN, LOW);
  digitalWrite(S2, LOW);  digitalWrite(S3, HIGH); B = pulseIn(OUT_PIN, LOW);
}

String getColour(int R, int G, int B) {
  if (R>=RED.rMin   && R<=RED.rMax   && G>=RED.gMin   && G<=RED.gMax   && B>=RED.bMin   && B<=RED.bMax)   return "RED";
  if (R>=GREEN.rMin && R<=GREEN.rMax && G>=GREEN.gMin && G<=GREEN.gMax && B>=GREEN.bMin && B<=GREEN.bMax) return "GREEN";
  if (R>=BLUE.rMin  && R<=BLUE.rMax  && G>=BLUE.gMin  && G<=BLUE.gMax  && B>=BLUE.bMin  && B<=BLUE.bMax)  return "BLUE";
  return "UNKNOWN";
}

// ----------------------------------------------------------------
// Line Following
// ----------------------------------------------------------------

void readSensors() { qtr.read(sensorValues); }

void followLine() {
  bool left  = sensorValues[0] > THRESHOLD || sensorValues[1] > THRESHOLD;
  bool mid   = sensorValues[2] > THRESHOLD && sensorValues[3] > THRESHOLD;
  bool right = sensorValues[4] > THRESHOLD || sensorValues[5] > THRESHOLD;

  if (mid) {
    drive(BASE_SPEED, BASE_SPEED); lastDir = 0; lastLineTime = millis();
  } else if (left) {
    drive(-TURN_REVERSE, TURN_SPEED); lastDir = -1; lastLineTime = millis();
  } else if (right) {
    drive(TURN_SPEED, -TURN_REVERSE); lastDir = 1; lastLineTime = millis();
  } else {
    if (millis() - lastLineTime < GAP_TIME) drive(BASE_SPEED, BASE_SPEED);
    else if (lastDir == -1) drive(-90,  90);
    else if (lastDir ==  1) drive( 90, -90);
    else                    drive( 90,  90);
  }
}

// ----------------------------------------------------------------
// Pick Sequence
// ----------------------------------------------------------------

void pick() {
  stopBot(); delay(300);
  for (int i = LIFT_UP;   i >= LIFT_DOWN;  i--) { liftServo.write(i); delay(15); }
  for (int i = GRIP_OPEN; i <= GRIP_CLOSE; i++) { gripServo.write(i); delay(15); }
  for (int i = LIFT_DOWN; i <= LIFT_UP;    i++) { liftServo.write(i); delay(15); }
  picked = true;
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
  BT.begin("CLASH_Bot_S1");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);

  gripServo.attach(GRIPPER_PIN);
  liftServo.attach(LIFT_PIN);
  liftServo.write(LIFT_UP);
  gripServo.write(GRIP_OPEN);
}

void loop() {
  handleBT();
  if (!systemOn) { stopBot(); return; }
  readSensors();
  followLine();
}
