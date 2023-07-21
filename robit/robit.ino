#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include "Motor.h"

#define WIFI_SSID "Your_SSID"
#define WIFI_PASS "Your_PASSWORD"

Motor motor1(34, 39, 1); // Encoder channel A, Encoder channel B, ESC
Motor motor2(23, 22, 2); // Encoder channel A, Encoder channel B, ESC

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);

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
  motor1.compute();
  motor2.compute();
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
        <div id="cursor" style="width: 10px; height: 10px; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: red;"></div>
      </div>
      <br>
      <p id="speed">0</p>
      <p id="direction">Neutral</p>
      <p>Adjust PID Settings:</p>
      <label>KP: <input type="text" id="kp" value="2.0"></label>
      <label>KI: <input type="text" id="ki" value="5.0"></label>
      <label>KD: <input type="text" id="kd" value="1.0"></label>
      <button onclick="updatePID()">Update PID</button>
      <script>
        var connection = new WebSocket('ws://' + location.hostname + ':80/ws');

        var controlPad = document.getElementById("controlPad");
        var cursor = document.getElementById("cursor");

        controlPad.addEventListener("mousemove", function(event) {
          var rect = controlPad.getBoundingClientRect();
          var x = event.clientX - rect.left; //x position within the element.
          var y = event.clientY - rect.top;  //y position within the element.
          cursor.style.left = x + "px";
          cursor.style.top = y + "px";

          var xPercent = x / rect.width;
          var yPercent = y / rect.height;

          // Send the position to the server
          connection.send(JSON.stringify({x: xPercent, y: yPercent}));
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
