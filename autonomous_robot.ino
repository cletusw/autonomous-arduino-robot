#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include "SR04.h"
#include "MedianFilter.h"
#include "MotorController.h"
#include "encoders.h"

// PID
const double Kp = 0.05;
const double Ki = 0.002;
const double Kd = 0;
const unsigned long pidSamplePeriodMillis = 100;

// Motors
const uint8_t maxMotorCommand = 255;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
MotorController leftMotor(*motor1, encoderDCountInternal, maxMotorCommand, pidSamplePeriodMillis, Kp, Ki, Kd);
MotorController rightMotor(*motor2, encoderBCountInternal, maxMotorCommand, pidSamplePeriodMillis, Kp, Ki, Kd);

// For all distance readings
const uint8_t maxDistance = 100;
const uint8_t minDistance = 10;
const unsigned long distanceSamplePeriodMillis = 50;
unsigned long nextMillis = 0;

// Ultrasonic sensor
const uint8_t TRIG_PIN = 10;
const uint8_t ECHO_PIN = 11;
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

// IR
const uint8_t IR0_PIN = A0;
const uint8_t IR1_PIN = A1;

// For smoothing
const uint8_t distanceFilterSize = 7;
MedianFilter ultrasonicFilter(distanceFilterSize, 0);
MedianFilter ir0(distanceFilterSize, 0);
MedianFilter ir1(distanceFilterSize, 0);


void setup() {
  Serial.begin(9600);

  AFMS.begin();  // create with the default frequency 1.6KHz

  setUpEncoderPins();

  pinMode(IR0_PIN, INPUT);
  pinMode(IR1_PIN, INPUT);

  initializeSmoothing();

  leftMotor.setDesiredSpeed(0);
  rightMotor.setDesiredSpeed(0);
  leftMotor.drive();
  rightMotor.drive();

  nextMillis = millis() + distanceSamplePeriodMillis;
}

void loop() {
  if (Serial.available()) {
    byte ch = Serial.read();
    if (ch >= '1' && ch <= '9') {
      double desiredSpeed = map(ch - '0', 1, 9, -50, 50) / 100.0;
      leftMotor.setDesiredSpeed(desiredSpeed);
      rightMotor.setDesiredSpeed(desiredSpeed);
    }
  }

  if (millis() > nextMillis) {
    nextMillis += distanceSamplePeriodMillis;

//    leftMotor.log(Serial);
//    rightMotor.log(Serial);

    takeDistanceReadings();
    uint16_t ir0Distance = ir0.out();
    uint16_t ultrasonicDistance = ultrasonicFilter.out();
    uint16_t ir1Distance = ir1.out();

    Serial.print(ir0Distance);
    Serial.print(' ');
    Serial.print(ultrasonicDistance);
    Serial.print(' ');
    Serial.print(ir1Distance);
    Serial.println();

    if (ultrasonicDistance < 30 || ir0Distance < 30 || ir1Distance < 30) {
      leftMotor.setDesiredSpeed(0);
      rightMotor.setDesiredSpeed(0);
    }
  }
  
  leftMotor.drive();
  rightMotor.drive();
}

void initializeSmoothing() {
  // fill buffer with readings
  for (uint8_t i = 0; i < distanceFilterSize; i++) {
    takeDistanceReadings();
    delay(25);
  }
}

// Need to wait 38ms +/- 10ms between IR readings,
// so you'll probably need a delay(25); after calling this
void takeDistanceReadings() {
  ir0.in(irReadingToDistance(analogRead(IR0_PIN)));
  ir1.in(irReadingToDistance(analogRead(IR1_PIN)));
  ultrasonicFilter.in(constrain(
      sr04.Distance(), minDistance, maxDistance)); // includes a delay(25)
}

// Returns distance in cm
uint8_t irReadingToDistance(uint16_t value) {
  if (value < 10) value = 10;
  return constrain(
      (6787.0 / (value - 3.0)) - 4.0, minDistance, maxDistance);
}


