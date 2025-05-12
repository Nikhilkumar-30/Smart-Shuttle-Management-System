#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebSocketsClient.h>

// --- GPS Settings ---
#define RXPin 16  // GPS RX pin
#define TXPin 17  // GPS TX pin
#define GPSBaud 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// --- CAN Settings ---
#define CAN_CS 5
#define CAN_INT 27
#define CAN_SPEED_ID 0x100
#define CAN_ACK_ID 0x037

MCP2515 mcp2515(CAN_CS);
struct can_frame canMsg;

// --- Motor & Sensor Pins ---
#define POT_PIN 34
#define MOTOR_PWM 12

// --- WiFi ---
const char* WIFI_SSID = "Error 404";
const char* WIFI_PASSWORD = "77777777";

// --- WebSocket Settings ---
const char* WS_URL = "shuttle-websocket-server.onrender.com"; // Your WebSocket URL
const uint16_t WS_PORT = 443; // Default HTTP port

WebSocketsClient webSocket;

// Geofence center coordinates  12.971490536855452, 79.16107680115357
const float geoFenceLat = 12.971490536855452; // Example latitude
const float geoFenceLng = 79.16107680115357; // Example longitude
const float geoFenceRadius = 675.0; // Radius in meters // Radius in meters
bool geofenceAlertSent = false;

// --- OLED Settings ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Checkpoints ---
struct Checkpoint {
    int id;
    float latitude;
    float longitude;
    float radius;
    String type;
    int count;
    String name;
};
Checkpoint* checkpoints = nullptr;
int numCheckpoints = 0;

// Display Mode
enum DisplayMode {
    SPEED_MODE,
    CHECKPOINT_MODE
};
DisplayMode currentDisplayMode = SPEED_MODE;
unsigned long lastDisplayModeChange = 0;
const unsigned long DISPLAY_MODE_INTERVAL = 5000; // 5 seconds

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
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

    // CAN Setup
    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println("CAN BUS Initialized");

    // Motor PWM Setup
    ledcAttachChannel(MOTOR_PWM, 5000, 8, 1);  // Channel 1 for PWM

    // OLED Setup
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println("SSD1306 allocation failed");
        for(;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();
}

void loop() {
    webSocket.loop();
    
    // Read potentiometer for speed control
    int potValue = analogRead(POT_PIN);
    int rpm = map(potValue, 0, 4095, 0, 200);
    int speedKmph = rpm * 0.3;  // 200 RPM = 12 km/h

    // Send Speed over CAN (Always runs)
    canMsg.can_id = CAN_SPEED_ID;
    canMsg.can_dlc = 2;
    canMsg.data[0] = (speedKmph >> 8) & 0xFF;
    canMsg.data[1] = speedKmph & 0xFF;
    mcp2515.sendMessage(&canMsg);
    Serial.print("Speed sent over CAN: "); Serial.println(speedKmph);

    // Control Motor Speed
    ledcWrite(1, map(rpm, 0, 200, 0, 255));

    // Process GPS Data
    while (gpsSerial.available() > 0) {
        char gpsChar = gpsSerial.read();
        Serial.write(gpsChar); // Print raw GPS data for debugging
        gps.encode(gpsChar);
    }

    // Toggle Display Mode
    if (millis() - lastDisplayModeChange > DISPLAY_MODE_INTERVAL) {
        toggleDisplayMode();
        lastDisplayModeChange = millis();
    }

    if (gps.location.isValid()) {
        float lat = gps.location.lat();
        float lng = gps.location.lng();

        Serial.print("Latitude: "); Serial.println(lat, 6);
        Serial.print("Longitude: "); Serial.println(lng, 6);

        // Send GPS Data to WebSocket
        sendGPSDataToWebSocket(lat, lng);

        // Geofence Check
        double distance = calculateDistance(lat, lng, geoFenceLat, geoFenceLng);
        Serial.print("Distance from Geofence Center: "); Serial.println(distance);
        
        if (distance > geoFenceRadius) {
            Serial.println("Outside Geofence.");
            if (!geofenceAlertSent) {
                geofenceAlertSent = true;
                sendAlert("Shuttle crossed geo fence at lat: " + String(lat, 6) + ", long: " + String(lng, 6));
                Serial.println("Geofence alert triggered.");
            }
        } else {
            Serial.println("Inside Geofence.");
            geofenceAlertSent = false;
        }

        // Check Checkpoint Proximity
        checkCheckpointProximity(lat, lng);
    } else {
        Serial.println("Waiting for valid GPS signal...");
    }

    // Update OLED Display
    updateOLEDDisplay(speedKmph);

    delay(200);
}

// --- Function to Toggle Display Mode ---
void toggleDisplayMode() {
    currentDisplayMode = (currentDisplayMode == SPEED_MODE) 
        ? CHECKPOINT_MODE 
        : SPEED_MODE;
}

// --- Function to Update OLED Display ---
void updateOLEDDisplay(int speedKmph) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    if (currentDisplayMode == SPEED_MODE) {
        // Speed Display
        display.setCursor(20, 20);
        display.setTextSize(2);
        display.print("Speed: ");
        display.println(speedKmph);
        display.print(" km/h");
    } else {
        // Checkpoint Display
        display.setCursor(0, 0);
        display.setTextSize(1);
        
        if (checkpoints == nullptr || numCheckpoints == 0) {
            display.println("No Checkpoints");
        } else {
            for (int i = 0; i < min(numCheckpoints, 4); i++) {
                display.print(checkpoints[i].name.substring(0, 15)); // Truncate long names
                display.print(": ");
                display.println(checkpoints[i].count);
            }
        }
    }

    display.display();
}
void sendAlert(String alert) {
    if (webSocket.isConnected()) {
        String alertData = "{\"type\": \"violationAlert\", \"alert\": \"" + alert + "\"}";
        webSocket.sendTXT(alertData);
        Serial.println("Sent violation alert to WebSocket: " + alertData);
    } else {
        Serial.println("WebSocket not connected. Cannot send violation alert.");
    }
}
// --- Function to Send GPS Data to WebSocket ---
void sendGPSDataToWebSocket(float latitude, float longitude) {
    if (webSocket.isConnected()) {
        String gpsData = "{\"type\": \"shuttleUpdate\", \"shuttleId\": \"1\", \"latitude\": " + String(latitude, 6) + ", \"longitude\": " + String(longitude, 6) + "}";
        webSocket.sendTXT(gpsData);
        Serial.println("GPS Data sent to WebSocket: " + gpsData);
    } else {
        Serial.println("WebSocket not connected. Cannot send GPS data.");
    }
}

// --- WebSocket Event Handler ---
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WSc] Disconnected!");
            break;
        case WStype_CONNECTED:
            Serial.println("[WSc] Connected to WebSocket server");
            break;
        case WStype_TEXT: {
            // Parse incoming JSON
            StaticJsonDocument<2048> doc;  // Increased buffer size
            DeserializationError error = deserializeJson(doc, payload, length);
            
            if (error) {
                Serial.print("JSON parsing failed: ");
                Serial.println(error.c_str());
                return;
            }

            // Check if it's checkpoint data
            const char* type = doc["type"];
            if (strcmp(type, "checkpointData") == 0) {
                // Free previous checkpoints if they exist
                if (checkpoints != nullptr) {
                    delete[] checkpoints;
                }

                // Parse checkpoint array
                JsonArray checkpointArray = doc["data"].as<JsonArray>();
                numCheckpoints = checkpointArray.size();
                checkpoints = new Checkpoint[numCheckpoints];

                // Populate checkpoint data
                for (int i = 0; i < numCheckpoints; i++) {
                    JsonObject checkpoint = checkpointArray[i];
                    checkpoints[i].id = checkpoint["id"].as<int>();
                    checkpoints[i].latitude = checkpoint["latitude"].as<float>();
                    checkpoints[i].longitude = checkpoint["longitude"].as<float>();
                    checkpoints[i].radius = checkpoint["radius"].as<float>();
                    checkpoints[i].type = checkpoint["type"].as<String>();
                    checkpoints[i].count = checkpoint["count"].as<int>();
                    checkpoints[i].name = checkpoint["name"].as<String>();

                    // Debug print
                    Serial.printf("Checkpoint %d: %s (Count: %d, Type: %s)\n", 
                        checkpoints[i].id, 
                        checkpoints[i].name.c_str(), 
                        checkpoints[i].count,
                        checkpoints[i].type.c_str()
                    );
                }
            }
            break;
        }
        case WStype_BIN:
            Serial.println("[WSc] Received binary data");
            break;
        case WStype_PING:
            Serial.println("[WSc] Received ping");
            break;
        case WStype_PONG:
            Serial.println("[WSc] Received pong");
            break;
    }
}
void clearCheckpointData(int checkpointId) {
    if (webSocket.isConnected()) {
        String clearData = "{\"type\": \"clearCheckpointData\", \"checkpointId\": " + String(checkpointId) + "}";
        webSocket.sendTXT(clearData);
        Serial.println("Sent clearCheckpointData request to WebSocket: " + clearData);
    } else {
        Serial.println("WebSocket not connected. Cannot send clearCheckpointData request.");
    }
}
// --- Function to Check Checkpoint Proximity ---
void checkCheckpointProximity(float currentLat, float currentLng) {
    if (checkpoints == nullptr) {
        return; // No checkpoints loaded
    }

    for (int i = 0; i < numCheckpoints; i++) {
        double distance = calculateDistance(currentLat, currentLng, 
                                            checkpoints[i].latitude, 
                                            checkpoints[i].longitude);
        
        // Check if within checkpoint radius
        if (distance <= checkpoints[i].radius) {
            Serial.printf("Near checkpoint: %s (Distance: %.2f meters)\n", 
                          checkpoints[i].name.c_str(), distance);
            clearCheckpointData(checkpoints[i].id);
            // Potential additional actions could be added here
        }
    }
}

// --- Function to Calculate Distance (Haversine Formula) ---
double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
    const double R = 6371e3; // Radius of Earth in meters
    double dLat = deg2rad(lat2 - lat1);
    double dLng = deg2rad(lng2 - lng1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLng / 2) * sin(dLng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

double deg2rad(double deg) {
    return deg * PI / 180;
}