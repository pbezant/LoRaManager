/**
 * Fast Downlink Example for LoRaManager
 * 
 * This example demonstrates how to configure the LoRaManager
 * for fast downlink reception with minimum delay of 1 second.
 * 
 * It configures the device for Class C operation with optimized
 * datarate setting to minimize downlink latency.
 * 
 * Hardware Compatibility:
 * - ESP32 boards with SX1262 modules
 * - Adjust the pin definitions for your specific board
 */

#include <Arduino.h>
#include <LoRaManager.h>

// Define LoRa pins for your board (example for HELTEC WiFi LoRa 32 V3)
#define LORA_CS   18
#define LORA_DIO1 23
#define LORA_RST  14
#define LORA_BUSY 33

// LoRaWAN credentials (replace with your own values)
uint64_t joinEUI = 0x0000000000000000;  // Replace with your Join EUI
uint64_t devEUI = 0x0000000000000000;   // Replace with your Device EUI
uint8_t appKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Replace with your App Key
uint8_t nwkKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Replace with your Network Key (same as App Key for LoRaWAN 1.0.x)

// Create LoRaManager instance
LoRaManager lora;

// Last time uplink was sent
unsigned long lastUplinkTime = 0;
const unsigned long UPLINK_INTERVAL = 60000; // Send uplink every 60 seconds

// Downlink reception time tracking
unsigned long lastDownlinkTime = 0;
unsigned long downlinkResponseTime = 0;
unsigned long downlinkCount = 0;

// Callback for downlink data
void handleDownlink(uint8_t* payload, size_t size, uint8_t port) {
  // Calculate response time (time since last uplink)
  downlinkResponseTime = millis() - lastUplinkTime;
  lastDownlinkTime = millis();
  downlinkCount++;
  
  Serial.print("Received downlink on port ");
  Serial.print(port);
  Serial.print(" after ");
  Serial.print(downlinkResponseTime);
  Serial.println(" ms");
  
  Serial.print("Payload (");
  Serial.print(size);
  Serial.print(" bytes): ");
  for (size_t i = 0; i < size; i++) {
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Simple payload processing example
  if (size > 0) {
    switch (payload[0]) {
      case 0x01:
        Serial.println("Command: On");
        // Add your control code here
        break;
      case 0x02:
        Serial.println("Command: Off");
        // Add your control code here
        break;
      case 0x03:
        Serial.println("Command: Status");
        // Reply with a status message
        sendStatusMessage();
        break;
    }
  }
}

// Send a status message
void sendStatusMessage() {
  // Example status message with device metrics
  uint8_t data[8];
  // First byte: battery level (example: 75%)
  data[0] = 75;
  // Second byte: temperature (example: 23°C)
  data[1] = 23;
  // Message counter
  data[2] = (downlinkCount >> 8) & 0xFF;
  data[3] = downlinkCount & 0xFF;
  // Last response time (ms)
  data[4] = (downlinkResponseTime >> 24) & 0xFF;
  data[5] = (downlinkResponseTime >> 16) & 0xFF;
  data[6] = (downlinkResponseTime >> 8) & 0xFF;
  data[7] = downlinkResponseTime & 0xFF;
  
  // Send on port 2 (unconfirmed)
  if (lora.sendData(data, sizeof(data), 2, false)) {
    Serial.println("Status message sent successfully!");
    lastUplinkTime = millis();
  } else {
    Serial.println("Failed to send status message");
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("Starting LoRaWAN Fast Downlink Example...");
  
  // Initialize the LoRa module
  if (!lora.begin(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY)) {
    Serial.println("Failed to initialize LoRa!");
    while (1);
  }
  
  // Set LoRaWAN credentials
  lora.setCredentials(joinEUI, devEUI, appKey, nwkKey);
  
  // Set downlink callback
  lora.setDownlinkCallback(handleDownlink);
  
  // Note: We don't need to explicitly set the downlink datarate for Class C mode
  // The library will automatically set it to the fastest rate (DR5) when switching to Class C
  // This gives approximately 1-second minimum downlink delay
  
  // Join the network
  Serial.println("Joining LoRaWAN network...");
  if (lora.joinNetwork()) {
    Serial.println("Successfully joined the network!");
    
    // For most responsive downlinks, switch to Class C
    if (lora.setDeviceClass(DEVICE_CLASS_C)) {
      Serial.println("Switched to Class C mode, continuous reception active");
      Serial.println("Device is now ready for fast downlink commands (DR5 set automatically)");
    } else {
      Serial.println("Failed to switch to Class C mode!");
    }
  } else {
    Serial.println("Failed to join the network!");
    // Will retry in the loop
  }
}

void loop() {
  // Handle LoRaWAN events (required)
  lora.handleEvents();
  
  // If we're not joined, try to join
  if (!lora.isNetworkJoined()) {
    static unsigned long lastJoinAttempt = 0;
    if (millis() - lastJoinAttempt > 30000) { // Try every 30 seconds
      Serial.println("Attempting to join network...");
      lora.joinNetwork();
      lastJoinAttempt = millis();
    }
    return;
  }
  
  // Send periodic uplink data
  if (millis() - lastUplinkTime > UPLINK_INTERVAL) {
    // Simple telemetry data
    uint8_t data[4];
    // Example: uptime in seconds
    uint32_t uptime = millis() / 1000;
    data[0] = (uptime >> 24) & 0xFF;
    data[1] = (uptime >> 16) & 0xFF;
    data[2] = (uptime >> 8) & 0xFF;
    data[3] = uptime & 0xFF;
    
    Serial.print("Sending uplink data... ");
    if (lora.sendData(data, sizeof(data), 1, false)) {
      Serial.println("Success!");
      lastUplinkTime = millis();
    } else {
      Serial.println("Failed!");
    }
  }
  
  // Print downlink statistics every minute
  static unsigned long lastStatsPrint = 0;
  if (millis() - lastStatsPrint > 60000) {
    Serial.println("=== Downlink Performance Statistics ===");
    Serial.print("Total downlinks received: ");
    Serial.println(downlinkCount);
    
    if (downlinkCount > 0) {
      Serial.print("Last downlink response time: ");
      Serial.print(downlinkResponseTime);
      Serial.println(" ms");
      
      Serial.print("Time since last downlink: ");
      Serial.print((millis() - lastDownlinkTime) / 1000);
      Serial.println(" seconds");
    } else {
      Serial.println("No downlinks received yet");
    }
    
    Serial.println("======================================");
    lastStatsPrint = millis();
  }
  
  // Brief delay to prevent CPU hogging
  delay(10);
} 