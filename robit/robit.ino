#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include "Motor.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>


#define WIFI_SSID "ssid"
#define WIFI_PASS "pass"

#define SERVO_PIN 13
#define SERVO_CHANNEL 3 // Using channel 3 now
#define SERVO_FREQ 50
#define SERVO_MIN_DUTY 819 // Duty cycle for 0 degrees
#define SERVO_MAX_DUTY 4092 // Duty cycle for 180 degrees


Motor* motorA;
Motor* motorB;
bool waving = false;
int pos = 0;    

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);
  Serial.println("Serial port opened");

  // motorA = new Motor(4, 16, 19, 0, "Motor A", 23, 22); // IN1, IN2, ENA, channel, motor name, encoderPinA, encoderPinB
  // motorB = new Motor(17, 18, 21, 1, "Motor B", 34, 39); // IN3, IN4, ENB, channel, motor name, encoderPinA, encoderPinB

  motorA = new Motor(16, 17, 4, 0, "Motor A", 23, 22); // IN1, IN2, ENA, channel, motor name, encoderPinA, encoderPinB
  motorB = new Motor(18, 19, 21, 1, "Motor B", 34, 39); // IN3, IN4, ENB, channel, motor name, encoderPinA, encoderPinB
  motorA->setMaxSpeed(7);
  motorA->setMaxSpeed(7);
  
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, 16); // 16-bit width
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getHtml());
  });
  server.addHandler(&ws);
  ws.onEvent(onWsEvent);

  server.begin();
}

void setServoAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  int dutyCycle = map(angle, 0, 180, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  ledcWrite(SERVO_CHANNEL, dutyCycle);

  Serial.println("Angle " + String(angle));

}

void waveServo() {
  static unsigned long lastWaveTime = 0;
  static int position = 0;
  static int direction = 1; // 1 for increasing, -1 for decreasing

  if (millis() - lastWaveTime >= 6) {
    position += direction * 5; // Change angle by 5 degrees
    if (position >= 180 || position <= 0) direction = -direction; // Reverse direction at limits
    setServoAngle(position);
    lastWaveTime = millis();
  }
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  static unsigned long lastUpdateTime2 = 0;
  unsigned long now = millis();

  // motorB->compute();
  motorA->compute();

  motorB->compute();

  if (waving) {
    waveServo();
  } else {
    setServoAngle(100);
  }

  if (now - lastUpdateTime2 >= 10) {
    motorB->updateRPM();
    motorA->updateRPM();

    lastUpdateTime2 = now;
  }
  if (now - lastUpdateTime >= 1000) {

    lastUpdateTime = now;

    DynamicJsonDocument doc(1024);
    doc["motorA"] = motorA->getTotalTicks();
    doc["motorA_rpm"] = motorA->getRPM();
    doc["motorB"] = motorB->getTotalTicks();
    doc["motorB_rpm"] = motorB->getRPM();
    doc["motorA_speed"] = motorA->getMaxSpeed();
    doc["KI"] = motorA->getKI();
    doc["KP"] = motorA->getKP();
    doc["kD"] = motorA->getKD();
    
    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
  }

  ws.cleanupClients();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
  } else if(type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if(type == WS_EVT_DATA) {
    // Parse JSON object
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, (char*)data);

    if (doc.containsKey("changeMaxSpeed")) {
      int direction = doc["changeMaxSpeed"];
      int newSpeed = motorA->getMaxSpeed() + direction;
      newSpeed = constrain(newSpeed, 1, 10); // Ensure the speed is within the range [1, 10]
      
      motorA->setMaxSpeed(newSpeed);
      motorB->setMaxSpeed(newSpeed);
    }

    if (doc.containsKey("axis1") && doc.containsKey("axis2")) {
      // Extract speed values
      double axis1 = doc["axis1"];
      double axis2 = doc["axis2"];
      axis1 = axis1/(10-7*(abs(axis2)/1.0));
      double motorBSpeed = min(100.0, max(-100.0, (axis1 - axis2) * 100.0));
      double motorASpeed = min(100.0, max(-100.0, (-axis2 - axis1) * 100.0));

      // Set motor speeds
      // Serial.println("Motor A Speed: " + String(motorASpeed) + "% speed" + " Motor B Speed: " + String(motorBSpeed) + "% speed");
      
      motorA->setSpeed(motorASpeed < 0, abs(motorASpeed));
      // Serial.println(String(motorBSpeed));
      motorB->setSpeed(motorBSpeed > 0, abs(motorBSpeed));
    }



    if (doc.containsKey("kp") && doc.containsKey("ki") && doc.containsKey("kd")) {
      // Extract PID values
      double kp = doc["kp"];
      double ki = doc["ki"];
      double kd = doc["kd"];

      // Set PID values for both motors
      motorA->setPidValues(kp, ki, kd);
      motorB->setPidValues(kp, ki, kd);
    }

    if (doc.containsKey("waveServo")) {
      waving = doc["waveServo"];
    }

  }


}


String getHtml() {
return R"rawliteral(
<!DOCTYPE HTML>
<html>
  <body>
    <div id="controllerInfo"></div>
    <br>
    <div id="motorInfo">
      <div id="leftMotor">Left Motor: <span id="leftDirection">&#8593;</span> <span id="leftSpeed">0</span>% <span id="leftEncoder">0</span></div>
      <div id="rightMotor">Right Motor: <span id="rightDirection">&#8593;</span> <span id="rightSpeed">0</span>% <span id="rightEncoder">0</span></div>
      <div id="pid">KP: <span id="KP">0</span> KI: <span id="KI">0</span> KD: <span id="KP">0</span></div>
    </div>
    <p>Adjust PID Settings:</p>
    <label>KP: <input type="text" id="kp" value="2.0"></label>
    <label>KI: <input type="text" id="ki" value="5.0"></label>
    <label>KD: <input type="text" id="kd" value="1.0"></label>
    <button onclick="updatePID()">Update PID</button>
    <script>

      var connection = new WebSocket('ws://' + location.hostname + ':80/ws');

      window.addEventListener("gamepadconnected", function(e) {
        var gp = navigator.getGamepads()[e.gamepad.index];
        var controllerInfo = document.getElementById("controllerInfo");
        controllerInfo.innerHTML = "Gamepad connected at index " + gp.index + ": " + gp.id + ".";
        setInterval(updateStatus, 100);
      });

function updateStatus(){
  var gp = navigator.getGamepads()[0];
  var axis1Int = 0;
  var axis2Int = 5;
  var b1Int = 6;
  var b2Int = 7;
  var b3Int = 1;
  if(gp){
    var controllerInfo = document.getElementById("controllerInfo");
    
    if (isMobileDevice()) {
      axis1Int = 0;
      axis2Int = 3;
      b1Int = 4;
      b2Int = 5;
      b3Int = 1;
    } 

    // Round axis2 and axis5 to the nearest hundreth
    var axis1 = parseFloat(gp.axes[axis1Int].toFixed(2));
    var axis2 = parseFloat(gp.axes[axis2Int].toFixed(2));
    // Handle B6 and B7 buttons
    var b1 = gp.buttons[b1Int].pressed;
    var b2 = gp.buttons[b2Int].pressed;
    var b3 = gp.buttons[b3Int].pressed;


    if(b1) {
      connection.send(JSON.stringify({ changeMaxSpeed: -1 }));
    }

    if(b2) {
      connection.send(JSON.stringify({ changeMaxSpeed: 1 }));
    }

    
    connection.send(JSON.stringify({ waveServo: b3 }));
    
    controllerInfo.innerHTML = "AXIS 1: " + axis1 + ", AXIS 2: " + axis2 + "<br />";
    // controllerInfo.innerHTML += "motorASpeed: " + motorASpeed + ", motorBSpeed: " + motorBSpeed + ", dir: " + dir + ", Max Speed: " + motorA.getMaxSpeed();

    connection.send(JSON.stringify({ axis1: axis1, axis2: axis2}));
  }
}

      var leftMotor = document.getElementById("leftSpeed");
      var rightMotor = document.getElementById("rightSpeed");
      var leftDirection = document.getElementById("leftDirection");
      var rightDirection = document.getElementById("rightDirection");
      var leftEncoder = document.getElementById("leftEncoder");
      var rightEncoder = document.getElementById("rightEncoder");
      var pid = document.getElementById("pid");

      connection.onmessage = function(e) {
        var data = JSON.parse(e.data);
        if (data.hasOwnProperty('motorA') && data.hasOwnProperty('motorB')) {
          leftEncoder.innerText = data.motorA + " ticks, " + data.motorA_rpm + " rpm" + " speed: " + data.motorA_speed;
          rightEncoder.innerText = data.motorB + " ticks, " + data.motorB_rpm + " rpm";
          pid.innerText = "PID: " + " KP: " + data.KP + " KI: " + data.KI + " KD: " + data.KD;
        }
      };

      function updatePID() {
        var kp = document.getElementById("kp").value;
        var ki = document.getElementById("ki").value;
        var kd = document.getElementById("kd").value;

        // Send PID values over WebSocket connection
        connection.send(JSON.stringify({ kp: kp, ki: ki, kd: kd }));
      }
      function isMobileDevice() {
        return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
      }


    </script>
  </body>
</html>
)rawliteral";


}
