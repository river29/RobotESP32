#ifndef MOTOR_H
#define MOTOR_H

#include <PID_v1.h>
#include <Encoder.h>

#define KP 0.2
#define KI 33
#define KD 0

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

  long previousTicks = 0;
  unsigned long previousTime = 0;
  float maxRpm = 40;
  float currentRpm = 0;
  int totalTicksPerRevolution = 640;
  int maxSpeed = 5;

public:
  Motor(int in1Pin, int in2Pin, int enablePin, int channel, String motorName, int encoderPinA, int encoderPinB) 
  : in1Pin(in1Pin), in2Pin(in2Pin), enablePin(enablePin), channel(channel), motorName(motorName), 
    myEnc(encoderPinA, encoderPinB), myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT)
  {
    Setpoint = 0;
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    ledcSetup(channel, 25000, 8);
    ledcAttachPin(enablePin, channel);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
    previousTicks = 0;
    previousTime = 0;
  }

  void setSetpoint(double newSetpoint) {
    Setpoint = newSetpoint;
  }

  void setSpeed(int direction, float speedPercentage) {
    if (direction == 0) {
      Setpoint = speedPercentage *-1;
    } else {
      Setpoint = speedPercentage;
    }
  }

  void setSpeed2(float speedPercentage) {
    if (speedPercentage >= 0) {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      // Serial.println(motorName + " running backward at " + String(speedPercentage) + "% speed");
    } else {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      // Serial.println(motorName + " running forward at " + String(speedPercentage) + "% speed");
    }
    // int dutyCycle = speedPercentage * 255 / 100; // convert percentage to duty cycle
    ledcWrite(channel, abs(speedPercentage));
  }



void compute()
{

  // Calculate percentage of maximum RPM
  Input = currentRpm/maxSpeed;
  // Input = myEnc.read();
  // Update PID controller
  myPID.Compute();
  int dutyCycle = constrain(Output, -255, 255); // Constrain PID output


  setSpeed2(dutyCycle);

  // Update previous values for next iteration


  // Log current RPM, setpoint, and duty cycle in CSV format
  // Serial.println(String(Input) + "," + String(Output) + "," + String(Setpoint));// + "," + String(dutyCycle));
}

  int getTotalTicks()
  {
    return myEnc.read();
  }

  float getRPM()
  {
    return currentRpm;
  }

  void updateRPM(){
    long currentTicks = myEnc.read();
    unsigned long currentTime = millis();
    long deltaTicks = currentTicks - previousTicks;
    unsigned long deltaTime = currentTime - previousTime;

    // Calculate RPM
    if (deltaTime > 0) {
      currentRpm = (float)deltaTicks / deltaTime * 60000.0 / totalTicksPerRevolution;
    }

    previousTicks = currentTicks;
    previousTime = currentTime;
  }
  void setPidValues(double kp, double ki, double kd) {
    myPID.SetTunings(kp, ki, kd);
  }

  int getMaxSpeed(){
    return maxSpeed;
  }
  void setMaxSpeed(int newSpeed) {
    maxSpeed = newSpeed;
  }




};

#endif // MOTOR_H
