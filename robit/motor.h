#ifndef MOTOR_H
#define MOTOR_H

#include <PID_v1.h>
#include <Encoder.h>

#define KP 2.0
#define KI 5.0
#define KD 1.0

class Motor
{
private:
  Encoder myEnc;
  double Setpoint, Input, Output;
  PID myPID;
  Servo myESC;
  int totalTicks;
public:
  Motor(int encoderPinA, int encoderPinB, int escPin) : myEnc(encoderPinA, encoderPinB), myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT)
  {
    myESC.attach(escPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    myPID.SetMode(AUTOMATIC);
    totalTicks = 0; // Initialize totalTicks to 0
  }

  void setSpeed(int speed)
  {
    Setpoint = speed;
  }

  void compute()
  {
    Input = myEnc.read();
    myPID.Compute();
    int pulseWidth = map(static_cast<int>(Output), 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    myESC.writeMicroseconds(pulseWidth);
  }

  int getTotalTicks()
  {
    return myEnc.read();
  }
};


#endif // MOTOR_H

