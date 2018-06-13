#include <Adafruit_MotorShield.h>

class MotorController {
  private:

  Adafruit_DCMotor &motor;
  volatile long &encoderCount;
  const uint8_t maxMotorCommand;
  int16_t signedMotorCommand = 0; // -255 to 255
  const unsigned long pidSamplePeriodMillis;
  const double metersPerSecondToPulsesPerSample;
  PID pid;
  double desiredSpeed = 0; // in encoder clicks per samplePeriodMillis
  double measuredSpeed = 0; // in encoder clicks per samplePeriodMillis
  double pidOutput = 0;

  public:

  MotorController(
      Adafruit_DCMotor &motor,
      volatile long &encoderCount,
      uint8_t maxMotorCommand,
      unsigned long pidSamplePeriodMillis,
      double Kp,
      double Ki,
      double Kd
  ) :
      motor(motor),
      encoderCount(encoderCount),
      maxMotorCommand(maxMotorCommand),
      pidSamplePeriodMillis(pidSamplePeriodMillis),
      metersPerSecondToPulsesPerSample(
        0.001 /*  s/ms */ *
        3840.0 /* pulses/rev */ /
        0.204 /* m/rev */ *
        pidSamplePeriodMillis /* ms/sample */
        /* = pulses/sample */),
      pid(&measuredSpeed, &pidOutput, &desiredSpeed, Kp, Ki, Kd, DIRECT) {
    pid.SetSampleTime(pidSamplePeriodMillis);
    pid.SetOutputLimits(-255, 255); // Capped to maxMotorCommand separately
    pid.SetMode(AUTOMATIC);
  }

  // 0.5 m/s is about the most I've been getting from these motors
  void setDesiredSpeed(double desiredSpeedInMetersPerSecond) {
    this->desiredSpeed = desiredSpeedInMetersPerSecond * metersPerSecondToPulsesPerSample;
    if (desiredSpeed == 0) {
      pid.SetMode(MANUAL);
      signedMotorCommand = 0;
    }
    else {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      pid.SetMode(AUTOMATIC);
    }
  }

  void drive() {
    noInterrupts();
    measuredSpeed = encoderCount;
    interrupts();

    if (pid.Compute()) { // If pidSamplePeriodMillis met, reads measuredSpeed, desiredSpeed, sets pidOutput, returns true
      noInterrupts();
      encoderCount = 0;
      interrupts();
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
    stream.print(desiredSpeed / metersPerSecondToPulsesPerSample * 500.0); // x500 for better range for Serial Plotter
    stream.print(' ');
    stream.print(measuredSpeed / metersPerSecondToPulsesPerSample * 500.0); // x500 for better range for Serial Plotter
    stream.print(' ');
    stream.print(signedMotorCommand);
    stream.println();
  }
};
