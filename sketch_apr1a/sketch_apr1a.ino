#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---- Wi-Fi Credentials ----
const char* ssid = "TimeisaRiver";
const char* password = "qwer1234";

// ---- Define Motor Control Pins (GPIO) ----
#define ENA  12  // Left Motor Speed (PWM)
#define ENB  14  // Right Motor Speed (PWM)
#define IN1  26  // Left Motor Forward
#define IN2  27  // Left Motor Backward
#define IN3  32  // Right Motor Forward
#define IN4  33  // Right Motor Backward

// Default speed (0-255)
int leftSpeed = 200;
int rightSpeed = 200;

// Create Async Web Server on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// HTML/JS Page (Same as before, no changes needed)
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
    h1 {
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

  <div class="btn-container">
    <button class="btn forward" id="btn-forward"
      onmousedown="sendCmd('FORWARD')" 
      onmouseup="sendCmd('STOP')" 
      ontouchstart="sendCmd('FORWARD')" 
      ontouchend="sendCmd('STOP')">
      ↑
    </button>

    <button class="btn left" id="btn-left"
      onmousedown="sendCmd('LEFT')" 
      onmouseup="sendCmd('STOP')" 
      ontouchstart="sendCmd('LEFT')" 
      ontouchend="sendCmd('STOP')">
      ←
    </button>

    <button class="btn stop" id="btn-stop" onclick="sendCmd('STOP')">STOP</button>

    <button class="btn right" id="btn-right"
      onmousedown="sendCmd('RIGHT')" 
      onmouseup="sendCmd('STOP')" 
      ontouchstart="sendCmd('RIGHT')" 
      ontouchend="sendCmd('STOP')">
      →
    </button>

    <button class="btn backward" id="btn-backward"
      onmousedown="sendCmd('BACKWARD')" 
      onmouseup="sendCmd('STOP')" 
      ontouchstart="sendCmd('BACKWARD')" 
      ontouchend="sendCmd('STOP')">
      ↓
    </button>
  </div>

  <script>
    const socket = new WebSocket(`ws://${location.host}/ws`);
    socket.onopen = () => console.log('WebSocket connected');
    socket.onmessage = (event) => console.log('Received:', event.data);
    socket.onclose = () => console.log('WebSocket disconnected');

    function sendCmd(cmd) {
      console.log('Sending command:', cmd);
      socket.send(cmd);
    }

    // ----- Handle Keyboard Controls -----
    document.addEventListener("keydown", (event) => {
      if (event.repeat) return; // Ignore repeated key presses

      switch (event.key) {
        case "ArrowUp":
          sendCmd("FORWARD");
          break;
        case "ArrowDown":
          sendCmd("BACKWARD");
          break;
        case "ArrowLeft":
          sendCmd("LEFT");
          break;
        case "ArrowRight":
          sendCmd("RIGHT");
          break;
      }
    });

    document.addEventListener("keyup", () => {
      sendCmd("STOP"); // Stop when the key is released
    });
  </script>
</body>
</html>

)rawliteral";

// ---------- Motor Control Functions ----------
// Set motor speed and direction
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void motorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void motorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void motorLeft() {
  // Reduce right motor speed for smooth left turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed * 0.6);  // Reduce left speed
  analogWrite(ENB, rightSpeed);       // Full right speed
}

void motorRight() {
  // Reduce left motor speed for smooth right turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);       // Full left speed
  analogWrite(ENB, rightSpeed * 0.6); // Reduce right speed
}

// -------- Handle WebSocket Messages --------
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*) arg;
  if (info->opcode == WS_TEXT) {
    String cmd;
    for (size_t i = 0; i < len; i++) {
      cmd += (char) data[i];
    }
    Serial.printf("Received: %s\n", cmd.c_str());

    // Execute motor actions
    if (cmd == "FORWARD") {
      motorForward();
      ws.textAll("Motors FORWARD");
    } else if (cmd == "BACKWARD") {
      motorBackward();
      ws.textAll("Motors BACKWARD");
    } else if (cmd == "LEFT") {
      motorLeft();
      ws.textAll("Motors LEFT (Smooth)");
    } else if (cmd == "RIGHT") {
      motorRight();
      ws.textAll("Motors RIGHT (Smooth)");
    } else if (cmd == "STOP") {
      motorStop();
      ws.textAll("Motors STOP");
    }
  }
}

// WebSocket event callback
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("WebSocket client connected");
      break;
    case WS_EVT_DISCONNECT:
      Serial.println("WebSocket client disconnected");
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motorStop(); // Ensure motors are stopped

  // Connect to Wi-Fi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve the main HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Start server
  server.begin();
  Serial.println("HTTP & WebSocket server started on port 80");
}

void loop() {
  // Async server, no loop logic needed
}
