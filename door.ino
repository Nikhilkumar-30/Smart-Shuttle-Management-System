#include <SPI.h>
#include <mcp2515.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// WiFi and WebSocket Configuration
const char* WIFI_SSID = "Error 404";
const char* WIFI_PASSWORD = "77777777";
const char* WS_URL = "shuttle-websocket-server.onrender.com";
const uint16_t WS_PORT = 443;

WebSocketsClient webSocket;

struct can_frame canMsg;
MCP2515 mcp2515(5);  // CS pin GPIO5

#define CAN_SPEED_ID 0x100
#define CAN_ACK_ID 0x037

Servo doorServo;
const int servoPin = 13;
const int buttonPin = 12;
const int buzzerPin = 27;

// Speed Violation Configuration
const int SPEED_THRESHOLD = 50;  // km/h
bool speedViolationAlertSent = false;
bool doorViolationAlertSent = false;

bool doorOpen = false;
int currentSpeed = 0;
unsigned long lastDebounce = 0;
int lastButtonState = HIGH;
bool buzzerState = false;
unsigned long lastBuzzerToggle = 0;

void setup() {
  Serial.begin(115200);
  
  // WiFi Setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // WebSocket Setup
  webSocket.beginSSL(WS_URL, WS_PORT, "/");
  webSocket.onEvent(webSocketEvent);

  // CAN and Peripheral Setup
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Servo setup
  ESP32PWM::allocateTimer(0);
  doorServo.setPeriodHertz(50);
  doorServo.attach(servoPin, 500, 2400);

  // Buzzer setup as digital output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Start with buzzer off

  pinMode(buttonPin, INPUT_PULLUP);
  doorServo.write(0);
}

void sendWebSocketAlert(const String& type, const String& message) {
  if (webSocket.isConnected()) {
    String alertData = "{\"type\": \"" + type + "\", \"alert\": \"" + message + "\"}";
    webSocket.sendTXT(alertData);
    Serial.println("Sent WebSocket alert: " + alertData);
  } else {
    Serial.println("WebSocket not connected. Cannot send alert.");
  }
}

void checkSpeedViolation() {
  // Speed violation check
  if (currentSpeed > SPEED_THRESHOLD && doorOpen) {
    if (!speedViolationAlertSent) {
      sendWebSocketAlert("violationAlert", 
        "Door open while speed exceeds " + String(SPEED_THRESHOLD) + 
        " km/h. Current speed: " + String(currentSpeed) + " km/h");
      speedViolationAlertSent = true;
    }
  } else if (currentSpeed <= SPEED_THRESHOLD && speedViolationAlertSent) {
    // Speed has returned to safe levels
    sendWebSocketAlert("infoAlert", 
      "Speed violation resolved. Current speed: " + String(currentSpeed) + " km/h");
    speedViolationAlertSent = false;
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Disconnected!");
      break;
    case WStype_CONNECTED:
      Serial.println("[WSc] Connected to WebSocket server");
      break;
    case WStype_TEXT:
      // Optional: Handle incoming WebSocket messages if needed
      break;
  }
}

void loop() {
  webSocket.loop();  // Maintain WebSocket connection

  // Button handling
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) lastDebounce = millis();
  
  if ((millis() - lastDebounce) > 50 && reading == LOW) {
    doorOpen = !doorOpen;
    doorServo.write(doorOpen ? 55 : 0);
    
    // Send door state alert
    
    
    Serial.println(doorOpen ? "Door Opened" : "Door Closed");
    delay(300);
  }
  lastButtonState = reading;

  // CAN Handling
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == CAN_SPEED_ID && canMsg.can_dlc == 2) {
      // Proper speed decoding
      currentSpeed = (canMsg.data[0] << 8) | canMsg.data[1];
      Serial.print("Real Speed: ");
      Serial.print(currentSpeed);
      Serial.println(" km/h");
      
      // Check for speed violations
      checkSpeedViolation();
      
      // Send ACK
      struct can_frame ack;
      ack.can_id = CAN_ACK_ID;
      ack.can_dlc = 1;
      ack.data[0] = 0x01;
      mcp2515.sendMessage(&ack);
    }
  }

  // Buzzer Control Logic
  if (currentSpeed >= 10) {
    if (doorOpen) {
      if (currentSpeed < 20) {
        // Speed 10-20 km/h: Buzzer on for 1 sec, off for 3 sec
        if (millis() - lastBuzzerToggle >= 4000 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 1000 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      } else if (currentSpeed < 30) {
        // Speed 20-30 km/h: Buzzer on for 1 sec, off for 1 sec
        if (millis() - lastBuzzerToggle >= 2000 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 1000 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      } else if (currentSpeed < 40) {
        // Speed 30-40 km/h: Buzzer on for 0.5 sec, off for 0.5 sec
        if (millis() - lastBuzzerToggle >= 1000 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 500 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      } else if (currentSpeed < 50) {
        // Speed 40-50 km/h: Buzzer on for 0.25 sec, off for 0.25 sec
        if (millis() - lastBuzzerToggle >= 500 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 250 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      } else if (currentSpeed < 60) {
        // Speed 50-60 km/h: Buzzer on for 0.15 sec, off for 0.15 sec
        if (millis() - lastBuzzerToggle >= 300 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 150 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      } else {
        // Speed ≥60 km/h: Buzzer on for 0.05 sec, off for 0.05 sec
        if (millis() - lastBuzzerToggle >= 100 && !buzzerState) {
          digitalWrite(buzzerPin, HIGH);
          buzzerState = true;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer ON");
        } else if (millis() - lastBuzzerToggle >= 50 && buzzerState) {
          digitalWrite(buzzerPin, LOW);
          buzzerState = false;
          lastBuzzerToggle = millis();
          Serial.println("Buzzer OFF");
        }
      }
    } else {
      if (buzzerState) {
        digitalWrite(buzzerPin, LOW);
        buzzerState = false;
        Serial.println("Buzzer OFF");
      }
    }
  }
}