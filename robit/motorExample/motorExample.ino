class Motor {
  private:
    int in1Pin;
    int in2Pin;
    int enablePin;
    int channel;

  public:
    Motor(int in1Pin, int in2Pin, int enablePin, int channel) {
      this->in1Pin = in1Pin;
      this->in2Pin = in2Pin;
      this->enablePin = enablePin;
      this->channel = channel;
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
      ledcSetup(channel, 5000, 8);
      ledcAttachPin(enablePin, channel);
    }

    void setSpeed(int direction, float speedPercentage) {
      if (direction == 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
      } else {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
      }
      int dutyCycle = speedPercentage * 255 / 100; // convert percentage to duty cycle
      ledcWrite(channel, dutyCycle);
    }
};

// create motor objects
Motor motorA(4, 16, 19, 0); // IN1, IN2, ENA, channel
Motor motorB(17, 18, 21, 1); // IN3, IN4, ENB, channel

void setup() {
  Serial.begin(115200);
  Serial.println("Motor controller setup complete");
}

void loop() {
  Serial.println("Motor A running forward at 5% speed");
  motorA.setSpeed(1, 100); // direction, speed (percentage)
  delay(1000);

  motorA.setSpeed(1, 0); // stop motor A
  Serial.println("Motor A stopped");
  delay(1000);

  Serial.println("Motor A running backward at 5% speed");
  motorA.setSpeed(0, 100); // direction, speed (percentage)
  delay(1000);

  motorA.setSpeed(0, 0); // stop motor A
  Serial.println("Motor A stopped");
  delay(1000);

  Serial.println("Motor B running forward at 5% speed");
  motorB.setSpeed(1, 100); // direction, speed (percentage)
  delay(1000);

  motorB.setSpeed(1, 0); // stop motor B
  Serial.println("Motor B stopped");
  delay(1000);

  Serial.println("Motor B running backward at 5% speed");
  motorB.setSpeed(0, 100); // direction, speed (percentage)
  delay(1000);

  motorB.setSpeed(0, 0); // stop motor B
  Serial.println("Motor B stopped");
  delay(1000);
}
