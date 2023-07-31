class Motor {
  private:
    int in1Pin;
    int in2Pin;
    int enablePin;
    int channel;
    String motorName;

  public:
    Motor(int in1Pin, int in2Pin, int enablePin, int channel, String motorName) {
      this->in1Pin = in1Pin;
      this->in2Pin = in2Pin;
      this->enablePin = enablePin;
      this->channel = channel;
      this->motorName = motorName;
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
      ledcSetup(channel, 5000, 8);
      ledcAttachPin(enablePin, channel);
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
};

// create motor objects
Motor motorA(4, 16, 19, 0, "Motor A"); // IN1, IN2, ENA, channel, motor name
Motor motorB(17, 18, 21, 1, "Motor B"); // IN3, IN4, ENB, channel, motor name

void setup() {
  Serial.begin(115200);
  Serial.println("Motor controller setup complete");
}

void loop() {
  motorA.setSpeed(1, 100); // direction, speed (percentage)
  delay(1000);

  motorA.setSpeed(1, 0); // stop motor A
  delay(1000);

  motorA.setSpeed(0, 100); // direction, speed (percentage)
  delay(1000);

  motorA.setSpeed(0, 0); // stop motor A
  delay(1000);

  motorB.setSpeed(1, 100); // direction, speed (percentage)
  delay(1000);

  motorB.setSpeed(1, 0); // stop motor B
  delay(1000);

  motorB.setSpeed(0, 100); // direction, speed (percentage)
  delay(1000);

  motorB.setSpeed(0, 0); // stop motor B
  delay(1000);
}
