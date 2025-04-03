/*************************************************************
 * ESP32 + L298N + Async WebSocket Control 
 * Press-and-hold logic: motors run while button is pressed,
 * and stop when button is released.
 *
 * Wi-Fi credentials: 
 *   SSID: TimeisaRiver
 *   PASS: qwer1234
 *************************************************************/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---- Wi-Fi Credentials ----
const char* ssid = "TimeisaRiver";
const char* password = "qwer1234";

// ---- Define Motor Control Pins (GPIO) ----
// (Adjust pins if needed for your wiring.)
#define IN1  26
#define IN2  27
#define IN3  32
#define IN4  33

// Create Async Web Server on port 80
AsyncWebServer server(80);
// WebSocket endpoint at "/ws"
AsyncWebSocket ws("/ws");

// HTML/JS Page: uses onmousedown/up + ontouchstart/end to send commands
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Press-Hold Motor Control</title>
  <style>
    button {
      width: 120px; height: 60px; font-size: 16px; margin: 5px;
    }
  </style>
</head>
<body>
  <h1>DC Motor Control (Press-Hold Logic)</h1>

  <button 
    onmousedown="sendCmd('FORWARD')" 
    onmouseup="sendCmd('STOP')" 
    ontouchstart="sendCmd('FORWARD')" 
    ontouchend="sendCmd('STOP')">
    FORWARD
  </button>
  <br/>

  <button 
    onmousedown="sendCmd('BACKWARD')" 
    onmouseup="sendCmd('STOP')" 
    ontouchstart="sendCmd('BACKWARD')" 
    ontouchend="sendCmd('STOP')">
    BACKWARD
  </button>
  <br/>

  <button 
    onmousedown="sendCmd('LEFT')" 
    onmouseup="sendCmd('STOP')" 
    ontouchstart="sendCmd('LEFT')" 
    ontouchend="sendCmd('STOP')">
    LEFT
  </button>
  <button 
    onmousedown="sendCmd('RIGHT')" 
    onmouseup="sendCmd('STOP')" 
    ontouchstart="sendCmd('RIGHT')" 
    ontouchend="sendCmd('STOP')">
    RIGHT
  </button>
  <br/>

  <!-- Optional separate STOP button if needed -->
  <button onclick="sendCmd('STOP')">STOP</button>

  <script>
    // Create a WebSocket connection to the ESP32
    const socket = new WebSocket(`ws://${location.host}/ws`);

    socket.onopen = () => {
      console.log('WebSocket connected');
    };

    socket.onmessage = (event) => {
      console.log('Received from ESP32:', event.data);
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected');
    };

    function sendCmd(cmd) {
      console.log('Sending command:', cmd);
      socket.send(cmd);
    }
  </script>
</body>
</html>
)rawliteral";

// ---------- Motor Control Functions ----------
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void motorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void motorLeft() {
// Left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorRight() {
  // Left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// -------- Handle Incoming WebSocket Messages --------
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*) arg;
  if (info->opcode == WS_TEXT) {
    // Convert bytes to String
    String cmd;
    for (size_t i = 0; i < len; i++) {
      cmd += (char) data[i];
    }
    Serial.printf("Received command: %s\n", cmd.c_str());

    // Execute motor actions
    if (cmd == "FORWARD") {
      motorForward();
      ws.textAll("Motors FORWARD");
    } else if (cmd == "BACKWARD") {
      motorBackward();
      ws.textAll("Motors BACKWARD");
    } else if (cmd == "LEFT") {
      motorLeft();
      ws.textAll("Motors LEFT");
    } else if (cmd == "RIGHT") {
      motorRight();
      ws.textAll("Motors RIGHT");
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
      // Not used in this example
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor pins as outputs
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
  // Nothing special needed here for Async server
}
