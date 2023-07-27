#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include "Motor.h"

#define WIFI_SSID "ssid"
#define WIFI_PASS "pass"

Motor* motor1; // Declare pointers to Motor objects
Motor* motor2;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);
  motor1 = new Motor(34, 39, 1); // Encoder channel A, Encoder channel B, ESC
  motor2 = new Motor(23, 22, 2); // Encoder channel A, Encoder channel B, ESC
  
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
  static unsigned long lastUpdateTime = 0; // Time of last update
  unsigned long now = millis();

  motor1->compute();
  motor2->compute();

  if (now - lastUpdateTime >= 1000) { // Update every 1 second
    lastUpdateTime = now;
    String msg = "{";
    msg += "\"motor1\":" + String(motor1->getTotalTicks()) + ",";
    msg += "\"motor2\":" + String(motor2->getTotalTicks());
    msg += "}";
    ws.textAll(msg); // Send encoder counts to all connected clients
  }

  ws.cleanupClients();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
  } else if(type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if(type == WS_EVT_DATA) {
    // Handle data received from the web interface
  }
}

String getHtml() {
  return R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <body>
      <div id="controlPad" style="width: 300px; height: 300px; position: relative; border: 1px solid black;">
        <div id="verticalLine" style="width: 1px; height: 100%; position: absolute; top: 0; left: 50%; background: black;"></div>
        <div id="horizontalLine" style="width: 100%; height: 1px; position: absolute; top: 50%; left: 0; background: black;"></div>
        <div id="cursor" style="width: 10px; height: 10px; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: red;"></div>
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
          if (data.hasOwnProperty('motor1') && data.hasOwnProperty('motor2')) {
            leftEncoder.innerText = data.motor1 + " ticks";
            rightEncoder.innerText = data.motor2 + " ticks";
          }
        };

        controlPad.addEventListener("mousemove", function(event) {
          var rect = controlPad.getBoundingClientRect();
          var x = event.clientX - rect.left;
          var y = event.clientY - rect.top;
          cursor.style.left = x + "px";
          cursor.style.top = y + "px";

          var xPercent = x / rect.width;
          var yPercent = y / rect.height;

          var speedPercent = 2 * Math.abs(0.5 - yPercent);
          var dir = yPercent < 0.5 ? "&#8593;" : "&#8595;"; // Up arrow and Down arrow in HTML

          var rightTurnMultiplier = xPercent < 0.5 ? 1 : 1 - 2 * (xPercent - 0.5);
          var leftTurnMultiplier = xPercent > 0.5 ? 1 : 1 - 2 * (0.5 - xPercent);

          leftMotor.innerHTML = Math.round(speedPercent * 100 * leftTurnMultiplier);
          rightMotor.innerHTML = Math.round(speedPercent * 100 * rightTurnMultiplier);
          leftDirection.innerHTML = rightDirection.innerHTML = dir;

          connection.send(JSON.stringify({x: xPercent, y: yPercent, leftTurnMultiplier: leftTurnMultiplier, rightTurnMultiplier: rightTurnMultiplier}));
        });

        cursor.addEventListener("mouseout", function(event) {
          cursor.style.left = "50%";
          cursor.style.top = "50%";

          leftMotor.innerHTML = rightMotor.innerHTML = "0";
          leftDirection.innerHTML = rightDirection.innerHTML = "&#8593;"; // Up arrow in HTML

          connection.send(JSON.stringify({x: 0.5, y: 0.5, leftTurnMultiplier: 1, rightTurnMultiplier: 1}));
        });

        function updatePID() {
          var kp = document.getElementById("kp").value;
          var ki = document.getElementById("ki").value;
          var kd = document.getElementById("kd").value;

          connection.send(JSON.stringify({kp: kp, ki: ki, kd: kd}));
        }
      </script>
    </body>
  </html>



    )rawliteral";
}
