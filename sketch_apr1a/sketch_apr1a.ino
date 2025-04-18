#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

// ---- Wi-Fi Credentials ----
const char* ssid = "TimeisaRiver";
const char* password = "qwer1234";

// ---- Motor Control Pins ----
#define ENA  12
#define ENB  14
#define IN1  26
#define IN2  27
#define IN3  32
#define IN4  33

int leftSpeed = 200;
int rightSpeed = 200;

// ---- Compass Setup ----
QMC5883LCompass compass;
float headingAngle = 0;

// ---- PID Control Variables ----
float targetHeading = 0;
float kp = 2.5, ki = 0.01, kd = 0.1;
float previousError = 0, integral = 0;
unsigned long pidStartTime = 0;
bool pidActive = false;

// ---- Web Server and WebSocket ----
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Motor Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background-color: #f4f4f4;
      display: flex;
      flex-direction: column;
      align-items: center;
      height: 100vh;
      justify-content: center;
    }
    h1, h2 {
      color: #333;
    }
    .btn-container {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 15px;
      width: 200px;
    }
    .btn {
      width: 80px;
      height: 80px;
      font-size: 18px;
      font-weight: bold;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      transition: 0.2s;
    }
    .btn:active {
      transform: scale(0.9);
    }
    .forward { grid-column: 2; background: #4CAF50; color: white; }
    .left { grid-column: 1; background: #2196F3; color: white; }
    .stop { grid-column: 2; background: #F44336; color: white; }
    .right { grid-column: 3; background: #FFC107; color: white; }
    .backward { grid-column: 2; background: #FF5722; color: white; }
  </style>
</head>
<body>
  <h1>ESP32 Motor Control</h1>
  <h2 id="heading">Heading: --°</h2>

  <div class="btn-container">
    <button class="btn forward" onmousedown="sendCmd('FORWARD')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('FORWARD')" ontouchend="sendCmd('STOP')">↑</button>
    <button class="btn left" onmousedown="sendCmd('LEFT')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('LEFT')" ontouchend="sendCmd('STOP')">←</button>
    <button class="btn stop" onclick="sendCmd('STOP')">STOP</button>
    <button class="btn right" onmousedown="sendCmd('RIGHT')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('RIGHT')" ontouchend="sendCmd('STOP')">→</button>
    <button class="btn backward" onmousedown="sendCmd('BACKWARD')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('BACKWARD')" ontouchend="sendCmd('STOP')">↓</button>
  </div>

  <br><button onclick="sendCmd('TURN30')" style="padding: 10px 20px; font-size: 18px;">Turn +30°</button>

  <script>
    const socket = new WebSocket(`ws://${location.host}/ws`);
    socket.onopen = () => console.log('WebSocket connected');
    socket.onmessage = (event) => {
      console.log('Received:', event.data);
      if (event.data.startsWith("HEADING:")) {
        const angle = event.data.split(":")[1];
        document.getElementById("heading").innerText = `Heading: ${angle}°`;
      }
    };
    socket.onclose = () => console.log('WebSocket disconnected');
    function sendCmd(cmd) {
      socket.send(cmd);
    }
    document.addEventListener("keydown", (event) => {
      if (event.repeat) return;
      switch (event.key) {
        case "ArrowUp": sendCmd("FORWARD"); break;
        case "ArrowDown": sendCmd("BACKWARD"); break;
        case "ArrowLeft": sendCmd("LEFT"); break;
        case "ArrowRight": sendCmd("RIGHT"); break;
      }
    });
    document.addEventListener("keyup", () => { sendCmd("STOP"); });
  </script>
</body>
</html>
)rawliteral";

// -------- Motor Control --------
void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void motorForwardCustom(int l, int r) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, l); analogWrite(ENB, r);
}

// -------- WebSocket Handler --------
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*) arg;
  if (info->opcode == WS_TEXT) {
    String cmd;
    for (size_t i = 0; i < len; i++) {
      cmd += (char) data[i];
    }
    Serial.printf("Received: %s\n", cmd.c_str());

    if (cmd == "FORWARD") {
      motorForwardCustom(leftSpeed, rightSpeed); ws.textAll("Motors FORWARD");
    } else if (cmd == "BACKWARD") {
      // backward control logic if needed
    } else if (cmd == "LEFT") {
      motorForwardCustom(leftSpeed * 0.5, rightSpeed);
    } else if (cmd == "RIGHT") {
      motorForwardCustom(leftSpeed, rightSpeed * 0.6);
    } else if (cmd == "STOP") {
      motorStop(); ws.textAll("Motors STOP");
    } else if (cmd == "TURN30") {
      targetHeading = headingAngle + 30;
      if (targetHeading >= 360) targetHeading -= 360;
      integral = 0; previousError = 0;
      pidStartTime = millis(); pidActive = true;
      ws.textAll("Turning 30 degrees...");
    }
  }
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) Serial.println("WebSocket client connected");
  else if (type == WS_EVT_DISCONNECT) Serial.println("WebSocket client disconnected");
  else if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorStop();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");

  Wire.begin(21, 22);
  compass.init(); compass.setSmoothing(10, true);

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
}

void loop() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  headingAngle = atan2(y, x) * 180.0 / PI;
  if (headingAngle < 0) headingAngle += 360;

  ws.textAll("HEADING:" + String(headingAngle, 1));

  if (pidActive) {
    unsigned long now = millis();
    if (now - pidStartTime <= 3000) {
      float error = targetHeading - headingAngle;
      if (error > 180) error -= 360;
      if (error < -180) error += 360;
      // Serial.println("--------------------------------");
      // Serial.print("error: "); Serial.println(error);
      integral += error;
      float derivative = error - previousError;
      float output = kp * error + ki * integral + kd * derivative;
      // Serial.print("derivative: "); Serial.println(derivative);
      // Serial.print("integral: "); Serial.println(integral);
      // Serial.print("output: "); Serial.println(output);
      previousError = error;

      float baseSpeed = 180;
      int left = baseSpeed - output;
      int right = baseSpeed + output;
      left = constrain(left, 0, 255);
      right = constrain(right, 0, 255);
      // Serial.println("--------------------------------");
      // Serial.print("left: "); Serial.println(left);
      // Serial.print("right: "); Serial.println(right);
      motorForwardCustom(left, right);
    } else {
      pidActive = false;
      motorStop();
      ws.textAll("30 degree turn complete");
    }
  }

  delay(100);
}
