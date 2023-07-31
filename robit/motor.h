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
  int in1Pin;
  int in2Pin;
  int enablePin;
  int channel;
  String motorName;

  Encoder myEnc;
  double Setpoint, Input, Output;
  PID myPID;

  int totalTicks;

public:
  Motor(int in1Pin, int in2Pin, int enablePin, int channel, String motorName, int encoderPinA, int encoderPinB) 
  : in1Pin(in1Pin), in2Pin(in2Pin), enablePin(enablePin), channel(channel), motorName(motorName), 
    myEnc(encoderPinA, encoderPinB), myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT)
  {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    ledcSetup(channel, 25000, 8);
    ledcAttachPin(enablePin, channel);

    myPID.SetMode(AUTOMATIC);
    totalTicks = 0; // Initialize totalTicks to 0
  }

  void setSpeed(int direction, float speedPercentage) {
    if (direction == 0) {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      Serial.println(motorName + " running backward at " + String(speedPercentage) + "% speed");
    } else {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      Serial.println(motorName + " running forward at " + String(speedPercentage) + "% speed");
    }
    int dutyCycle = speedPercentage * 255 / 100; // convert percentage to duty cycle
    ledcWrite(channel, dutyCycle);

    if(speedPercentage == 0) {
      Serial.println(motorName + " stopped");
    }
  }

  void compute()
  {
    Input = myEnc.read();
    myPID.Compute();
    int dutyCycle = Output * 255 / 100; // convert PID output to duty cycle
    ledcWrite(channel, dutyCycle);
  }

  int getTotalTicks()
  {
    return myEnc.read();
  }
};

#endif // MOTOR_H
