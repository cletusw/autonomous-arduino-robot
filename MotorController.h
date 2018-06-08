#include <Adafruit_MotorShield.h>

class MotorController {
  private:

  Adafruit_DCMotor &motor;
  const uint8_t maxMotorCommand;
  int16_t signedMotorCommand = 0; // -255 to 255
  PID pid;
  long encoderCountPrevious = 0;
  double desiredSpeed = 0; // in encoder clicks per samplePeriodMillis
  double measuredSpeed = 0; // in encoder clicks per samplePeriodMillis
  double pidOutput = 0;

  public:

  MotorController(
      Adafruit_DCMotor &motor,
      uint8_t maxMotorCommand,
      unsigned long pidSamplePeriodMillis,
      double Kp,
      double Ki,
      double Kd
  ) :
      motor(motor),
      maxMotorCommand(maxMotorCommand),
      pid(&measuredSpeed, &pidOutput, &desiredSpeed, Kp, Ki, Kd, DIRECT) {
    pid.SetSampleTime(pidSamplePeriodMillis);
    pid.SetOutputLimits(-255, 255); // Capped to maxMotorCommand separately
    pid.SetMode(AUTOMATIC);
  }

  void setDesiredSpeed(double desiredSpeed) {
    this->desiredSpeed = desiredSpeed;
    if (desiredSpeed == 0) {
      pid.SetMode(MANUAL);
      signedMotorCommand = 0;
    }
    else {
      pid.SetMode(AUTOMATIC);
    }
  }

  void updateMeasuredSpeed(volatile long &encoderCount) {
    noInterrupts();
    long tempEncoderCount = encoderCount;
    interrupts();
    measuredSpeed = tempEncoderCount - encoderCountPrevious;
    encoderCountPrevious = tempEncoderCount;
  }

  void drive() {
    if (pid.Compute()) { // If pidSamplePeriodMillis met, reads measuredSpeed, desiredSpeed, sets pidOutput, returns true
      signedMotorCommand = constrain(signedMotorCommand + pidOutput, -maxMotorCommand, maxMotorCommand);
    }
  
    if (desiredSpeed > 0 && signedMotorCommand > 0) {
      motor.setSpeed(signedMotorCommand);
      motor.run(FORWARD);
    }
    else if (desiredSpeed < 0 && signedMotorCommand < 0) {
      motor.setSpeed(-signedMotorCommand);
      motor.run(BACKWARD);
    }
    else {
      motor.setSpeed(0);
      motor.run(RELEASE);
    }
  }

  void log(Stream &stream) {
    stream.print(desiredSpeed);
    stream.print(' ');
    stream.print(measuredSpeed);
    stream.print(' ');
    stream.print(signedMotorCommand);
    stream.println();
  }
};
