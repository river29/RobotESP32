#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include "Motor.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>

#define WIFI_SSID "ssid"
#define WIFI_PASS "pass"

Motor* motorA;
Motor* motorB;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);
  Serial.println("Serial port opened");

  motorA = new Motor(4, 16, 19, 0, "Motor A", 23, 22); // IN1, IN2, ENA, channel, motor name, encoderPinA, encoderPinB
  motorB = new Motor(17, 18, 21, 1, "Motor B", 34, 39); // IN3, IN4, ENB, channel, motor name, encoderPinA, encoderPinB
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

void loop() {
  static unsigned long lastUpdateTime = 0;
  static unsigned long lastUpdateTime2 = 0;
  unsigned long now = millis();

  // motorB->compute();
  motorA->compute();

  motorB->compute();

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
      newSpeed = constrain(newSpeed, 1, 14); // Ensure the speed is within the range [1, 10]
      
      motorA->setMaxSpeed(newSpeed);
      motorB->setMaxSpeed(newSpeed);
    }

    if (doc.containsKey("axis2") && doc.containsKey("axis5")) {
      // Extract speed values
      double axis2 = doc["axis2"];
      double axis5 = doc["axis5"];
      axis2 = axis2/(10-7*(abs(axis5)/1.0));
      double motorBSpeed = min(100.0, max(-100.0, (axis2 - axis5) * 100.0));
      double motorASpeed = min(100.0, max(-100.0, (-axis5 - axis2) * 100.0));

      // Set motor speeds
      Serial.println("Motor A Speed: " + String(motorASpeed) + "% speed" + " Motor B Speed: " + String(motorBSpeed) + "% speed");
      
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
  }
}


String getHtml() {
return R"rawliteral(
<!DOCTYPE HTML>
<html>
  <body>
    <div id="controllerInfo"></div>
    <div id="controlPad" style="width: 80vw; height: 80vw; position: relative; border: 1px solid black;">
      <div id="verticalLine" style="width: 1px; height: 100%; position: absolute; top: 0; left: 50%; background: black;"></div>
      <div id="horizontalLine" style="width: 100%; height: 1px; position: absolute; top: 50%; left: 0; background: black;"></div>
      <div id="cursor" style="width: 10px; height: 10px; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: red;"></div>
    </div>
    <br>
    <div id="motorInfo">
      <div id="leftMotor">Left Motor: <span id="leftDirection">&#8593;</span> <span id="leftSpeed">0</span>% <span id="leftEncoder">0</span></div>
      <div id="rightMotor">Right Motor: <span id="rightDirection">&#8593;</span> <span id="rightSpeed">0</span>% <span id="rightEncoder">0</span></div>
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
  if(gp){
    var controllerInfo = document.getElementById("controllerInfo");

    var axis2 = gp.axes[0];
    var axis5 = gp.axes[5];

    // Handle B6 and B7 buttons
    var b6 = gp.buttons[6].pressed;
    var b7 = gp.buttons[7].pressed;

    if(b6) {
      connection.send(JSON.stringify({ changeMaxSpeed: -1 }));
    }

    if(b7) {
      connection.send(JSON.stringify({ changeMaxSpeed: 1 }));
    }

    controllerInfo.innerHTML = "AXIS 2: " + axis2 + ", AXIS 5: " + axis5 + "<br />";
    // controllerInfo.innerHTML += "motorASpeed: " + motorASpeed + ", motorBSpeed: " + motorBSpeed + ", dir: " + dir + ", Max Speed: " + motorA.getMaxSpeed();

    connection.send(JSON.stringify({ axis2: axis2, axis5: axis5}));
  }
}
      var controlPad = document.getElementById("controlPad");
      var cursor = document.getElementById("cursor");
      var leftMotor = document.getElementById("leftSpeed");
      var rightMotor = document.getElementById("rightSpeed");
      var leftDirection = document.getElementById("leftDirection");
      var rightDirection = document.getElementById("rightDirection");
      var leftEncoder = document.getElementById("leftEncoder");
      var rightEncoder = document.getElementById("rightEncoder");

      connection.onmessage = function(e) {
        var data = JSON.parse(e.data);
        if (data.hasOwnProperty('motorA') && data.hasOwnProperty('motorB')) {
          leftEncoder.innerText = data.motorA + " ticks, " + data.motorA_rpm + " rpm" + " speed: " + data.motorA_speed;
          rightEncoder.innerText = data.motorB + " ticks, " + data.motorB_rpm + " rpm";
        }
      };
  
      function updatePID() {
        var kp = document.getElementById("kp").value;
        var ki = document.getElementById("ki").value;
        var kd = document.getElementById("kd").value;

        // Send PID values over WebSocket connection
        connection.send(JSON.stringify({ kp: kp, ki: ki, kd: kd }));
      }
    </script>
  </body>
</html>
)rawliteral";


}
