#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include "SR04.h"
#include "MotorController.h"
#include "encoders.h"
#include "median5.h"

// PID
const double Kp = 0.05;
const double Ki = 0.002;
const double Kd = 0;
const unsigned long pidSamplePeriodMillis = 100;
unsigned long nextMillis = 0;

// Motors
const uint8_t maxMotorCommand = 255;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
MotorController leftMotor(*motor1, maxMotorCommand, pidSamplePeriodMillis, Kp, Ki, Kd);
MotorController rightMotor(*motor2, maxMotorCommand, pidSamplePeriodMillis, Kp, Ki, Kd);

// Ultrasonic sensor
const int TRIG_PIN = 10;
const int ECHO_PIN = 11;
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

// For smoothing with the median-of-5 filter
const int numUltrasonicReadings = 5;
int ultrasonicReadings[numUltrasonicReadings];
uint8_t ultrasonicReadIndex = 0; // the index of the oldest/next-to-be-overwritten reading


void setup() {
  Serial.begin(9600);

  AFMS.begin();  // create with the default frequency 1.6KHz

  setUpEncoderPins();

  leftMotor.setDesiredSpeed(0);
  rightMotor.setDesiredSpeed(0);
  leftMotor.drive();
  rightMotor.drive();

  initializeSmoothing();

  nextMillis = millis() + pidSamplePeriodMillis;
}

void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    byte ch = Serial.read();
    if (ch >= '1' && ch <= '9') {
      double desiredSpeed = map(ch - '0', 1, 9, -50, 50) / 100.0;
      leftMotor.setDesiredSpeed(desiredSpeed);
      rightMotor.setDesiredSpeed(desiredSpeed);
    }
  }

  if (readUltrasonicSensor() < 20) {
    leftMotor.setDesiredSpeed(0);
    rightMotor.setDesiredSpeed(0);
  }

  if (currentMillis > nextMillis) {
    nextMillis += pidSamplePeriodMillis;

    leftMotor.updateMeasuredSpeed(encoderDCountInternal);
    rightMotor.updateMeasuredSpeed(encoderBCountInternal);

    leftMotor.log(Serial);
//    rightMotor.log(Serial);
  }
  
  leftMotor.drive();
  rightMotor.drive();
}

void initializeSmoothing() {
  // fill buffer with readings
  for (int i = 0; i < numUltrasonicReadings; i++) {
    ultrasonicReadings[i] = sr04.Distance();
  }
}

int readUltrasonicSensor() {
  int distance = sr04.Distance(); //  2cm - 400cm (ish)
  ultrasonicReadings[ultrasonicReadIndex] = distance;
  ultrasonicReadIndex = (ultrasonicReadIndex + 1) % numUltrasonicReadings;
  return distance;
}


