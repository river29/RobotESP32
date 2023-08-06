#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include "Motor.h"
#include <ArduinoJson.h>

#define WIFI_SSID "GetOffMyLawn"
#define WIFI_PASS "gobeavs1234"

Motor* motorA;
Motor* motorB;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



void setup() {
  Serial.begin(115200);
  Serial.println("Serial port opened");
  
  motorA = new Motor(4, 16, 19, 0, "Motor A", 23, 22);
  motorB = new Motor(17, 18, 21, 1, "Motor B", 34, 39);
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

  if (now - lastUpdateTime2 >= 10) {
    motorB->compute();
    // motorA->compute();

    lastUpdateTime2 = now;
  }
  if (now - lastUpdateTime >= 1000) {
    lastUpdateTime = now;

    DynamicJsonDocument doc(1024);
    doc["motorA"] = motorA->getTotalTicks();
    doc["motorA_rpm"] = motorA->getRPM();
    doc["motorB"] = motorB->getTotalTicks();
    doc["motorB_rpm"] = motorB->getRPM();
    
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

    if (doc.containsKey("motorA") && doc.containsKey("motorB") && doc.containsKey("dir")) {
      // Extract speed values
      int motorASpeed = doc["motorA"];
      int motorBSpeed = doc["motorB"];

      int dir = doc["dir"];

      // Set motor speeds
      // Serial.println("Motor A Speed: " + String(motorASpeed) + "% speed" + " - direction: " + String(dir));

      // motorA->setSpeed(dir, motorASpeed);
      motorB->setSpeed(!dir, motorBSpeed);
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
    <div id="controlPad" style="width: 30vw; height: 30vw; position: relative; border: 1px solid black;">
      <div id="verticalLine" style="width: 1px; height: 100%; position: absolute; top: 0; left: 50%; background: black;"></div>
      <div id="horizontalLine" style="width: 100%; height: 1px; position: absolute; top: 50%; left: 0; background: black;"></div>
      <div id="cursor" style="width: 20px; height: 20px; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: red;"></div>
    </div>
    <br>
    <div id="motorInfo">
      <div id="leftMotor">Left Motor: <span id="leftDirection">&#8593;</span> <span id="leftSpeed">0</span>% <span id="leftEncoder">0</span> ticks</div>
      <div id="rightMotor">Right Motor: <span id="rightDirection">&#8593;</span> <span id="rightSpeed">0</span>% <span id="rightEncoder">0</span> ticks</div>
    </div>
    <p>Adjust PID Settings:</p>
    <label>KP: <input type="text" id="kp" value="2.0"></label>
    <label>KI: <input type="text" id="ki" value="5.0"></label>
    <label>KD: <input type="text" id="kd" value="1.0"></label>
    <button onclick="updatePID()">Update PID</button>
    <script>
      var connection = new WebSocket('ws://' + location.hostname + ':80/ws');

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
          leftEncoder.innerText = data.motorA + " ticks, " + data.motorA_rpm + " rpm";
          rightEncoder.innerText = data.motorB + " ticks, " + data.motorB_rpm + " rpm";
        }
      };

      function handleMovement(clientX, clientY) {
        var rect = controlPad.getBoundingClientRect();
        var x = clientX - rect.left;
        var y = clientY - rect.top;
        cursor.style.left = x + "px";
        cursor.style.top = y + "px";

        var xPercent = x / rect.width;
        var yPercent = y / rect.height;

        var speedPercent = 2 * Math.abs(0.5 - yPercent);
        var dir = yPercent < 0.5 ? "&#8593;" : "&#8595;"; // Up arrow and Down arrow in HTML

        var rightTurnMultiplier = xPercent < 0.5 ? 1 : 1 - 2 * (xPercent - 0.5);
        var leftTurnMultiplier = xPercent > 0.5 ? 1 : 1 - 2 * (0.5 - xPercent);

        var motorASpeed = Math.round(speedPercent * 100 * rightTurnMultiplier);
        var motorBSpeed = Math.round(speedPercent * 100 * leftTurnMultiplier);

        leftMotor.innerHTML = motorBSpeed;
        rightMotor.innerHTML = motorASpeed;
        leftDirection.innerHTML = rightDirection.innerHTML = dir;

        // Send motor commands over WebSocket connection
        connection.send(JSON.stringify({ motorA: motorASpeed, motorB: motorBSpeed, dir: yPercent < 0.5 }));
      }

      function handleMouseOut() {
        cursor.style.left = "50%";
        cursor.style.top = "50%";

        leftMotor.innerHTML = rightMotor.innerHTML = "0";
        leftDirection.innerHTML = rightDirection.innerHTML = "&#8593;";

        // Send stop command over WebSocket connection
        connection.send(JSON.stringify({ motorA: 0, motorB: 0, dir: true }));
      }

      controlPad.addEventListener("mousemove", function(event) {
        handleMovement(event.clientX, event.clientY);
      });

      controlPad.addEventListener("mouseout", function(event) {
        handleMouseOut();
      });


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
