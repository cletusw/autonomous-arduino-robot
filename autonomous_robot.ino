#include <limits.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include <NewPing.h>
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
const uint8_t maxDistance = 100; // <=255 so it fits in the uint8_t
const uint8_t minDistance = 10;
const unsigned long distanceSamplePeriodMillis = 75; // need to wait 38ms +/- 10ms between IR readings,
unsigned long nextMillis = 0;

// Ultrasonic sensor
const uint8_t TRIGGER_PIN = 10;
const uint8_t ECHO_PIN = 11;
NewPing sonar(TRIGGER_PIN, ECHO_PIN);
volatile unsigned long latestSonarResult = 0;

// IR
const uint8_t IR0_PIN = A0;
const uint8_t IR1_PIN = A1;

// For smoothing
const uint8_t distanceFilterSize = 7;
MedianFilter sonarFilter(distanceFilterSize, 0);
MedianFilter ir0(distanceFilterSize, 0);
MedianFilter ir1(distanceFilterSize, 0);


void setup() {
  Serial.begin(115200);

  AFMS.begin();  // create with the default frequency 1.6KHz

  setUpEncoderPins();

  pinMode(IR0_PIN, INPUT);
  pinMode(IR1_PIN, INPUT);

  nextMillis = millis();
  initializeSmoothing();

  leftMotor.setDesiredSpeed(0);
  rightMotor.setDesiredSpeed(0);
  leftMotor.drive();
  rightMotor.drive();
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

    takeSonarReading();
    takeIrReadings();

    uint8_t ir0Distance = ir0.out();
    uint8_t sonarDistance = sonarFilter.out();
    uint8_t ir1Distance = ir1.out();

    Serial.print(ir0Distance);
    Serial.print(' ');
    Serial.print(sonarDistance);
    Serial.print(' ');
    Serial.print(ir1Distance);
    Serial.println();

    if (sonarDistance < 30 || ir0Distance < 30 || ir1Distance < 30) {
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
    while (millis() < nextMillis);
    nextMillis += distanceSamplePeriodMillis;
    handleSonarResult(sonar.ping());
    takeIrReadings();
  }
}

void takeSonarReading() {
  sonar.timer_stop();
  unsigned long sonarResult = latestSonarResult; // Not worried about a data race because we stopped the timer interrupt
  latestSonarResult = 0;
  sonar.ping_timer(echoCheck); // Start the next one ASAP
  handleSonarResult(sonarResult ? sonarResult : NO_ECHO);
}

void echoCheck() {
  if (sonar.check_timer()) {
    latestSonarResult = sonar.ping_result;
  }
}

void handleSonarResult(unsigned long pingTimeMicroseconds) {
  uint8_t noisyDistanceInCm =
    (pingTimeMicroseconds == NO_ECHO ||
    pingTimeMicroseconds > UINT_MAX) ?
      maxDistance :
      constrain(
        NewPing::convert_cm(pingTimeMicroseconds),
        minDistance,
        maxDistance);
  sonarFilter.in(noisyDistanceInCm);
}

void takeIrReadings() {
  ir0.in(irReadingToDistance(analogRead(IR0_PIN)));
  ir1.in(irReadingToDistance(analogRead(IR1_PIN)));
}

// Returns distance in cm
uint8_t irReadingToDistance(uint16_t value) {
  if (value < 10) value = 10;
  return constrain(
      (6787.0 / (value - 3.0)) - 4.0, minDistance, maxDistance);
}

