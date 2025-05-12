#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <MFRC522.h>

// WiFi Credentials
const char* ssid = "Error 404";
const char* password = "77777777";

// API Configuration
const String API_BASE_URL = "https://shuttle-api.vercel.app/api/transaction/deduct-balance";
const int TRANSACTION_AMOUNT = 20; // Fixed transaction amount

// RFID Module Pins (using standard ESP32 RFID setup)
#define SS_PIN 5  // SS pin for ESP32
#define RST_PIN 0 // RST pin for ESP32
MFRC522 rfid(SS_PIN, RST_PIN);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize SPI and RFID module
    SPI.begin();
    rfid.PCD_Init();
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ Connected to WiFi");
}

void ensureWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠ WiFi disconnected. Reconnecting...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\n✅ Reconnected to WiFi");
    }
}

void deductBalance(String cardID) {
    // Ensure WiFi is connected
    ensureWiFi();

    // Construct the full API URL for deducting balance
    String url = API_BASE_URL + "/" + cardID + "/" + String(TRANSACTION_AMOUNT);
    
    // Create secure WiFi client
    WiFiClientSecure client;
    client.setInsecure(); // Note: Use proper SSL verification in production
    
    // Initialize HTTP client
    HTTPClient https;
    https.begin(client, url);
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    // Send GET request
    int httpCode = https.GET();
    
    // Handle different response scenarios
    if (httpCode == 200) {
        String response = https.getString();
        Serial.println("✅ Balance Deducted Successfully!");
        Serial.println("Response: " + response);
    } 
    else if (httpCode == 400) {
        String errorResponse = https.getString();
        Serial.println("❌ Transaction Failed: " + errorResponse);
    }
    else {
        Serial.println("❌ HTTP Error: " + String(httpCode));
    }
    
    // Close connection
    https.end();
}

void loop() {
    // Check for new RFID card
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return;
    }
    
    // Convert RFID UID to HEX format
    String rfid_uid = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
        rfid_uid += String(rfid.uid.uidByte[i], HEX);
    }
    rfid_uid.toUpperCase();  // Convert to uppercase for consistency

    Serial.println("RFID UID (HEX): " + rfid_uid);
    
    // Halt RFID reading to prevent multiple reads
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    
    // Attempt to deduct balance
    deductBalance(rfid_uid);
    
    // Delay to prevent rapid consecutive reads
    delay(2000);
}
