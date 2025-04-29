#include "LoRaManager.h"
#include <RadioLib.h>

// Define error codes that are not already defined in RadioLib
// or use the ones from RadioLib directly
// We'll keep these for backward compatibility with existing code
#ifndef RADIOLIB_ERR_INVALID_STATE
#define RADIOLIB_ERR_INVALID_STATE             (-1)
#endif

#ifndef RADIOLIB_ERR_INVALID_INPUT
#define RADIOLIB_ERR_INVALID_INPUT             (-3)
#endif

// Use RadioLib's error codes instead of redefining them
// RADIOLIB_ERR_NETWORK_NOT_JOINED is already defined in RadioLib
// RADIOLIB_ERR_INVALID_FREQUENCY is already defined in RadioLib
// RADIOLIB_LORAWAN_NEW_SESSION is already defined in RadioLib

#ifndef RADIOLIB_ERR_NO_CHANNEL_AVAILABLE
#define RADIOLIB_ERR_NO_CHANNEL_AVAILABLE      (-1106)
#endif

#ifndef RADIOLIB_LORAWAN_NO_DOWNLINK
#define RADIOLIB_LORAWAN_NO_DOWNLINK           (-5)
#endif

// Define error codes for Class B and C operations
#ifndef RADIOLIB_ERR_BEACON_NOT_RECEIVED
#define RADIOLIB_ERR_BEACON_NOT_RECEIVED      (-2000)
#endif

#ifndef RADIOLIB_ERR_BEACON_ACQUISITION_FAILED
#define RADIOLIB_ERR_BEACON_ACQUISITION_FAILED (-2001)
#endif

#ifndef RADIOLIB_ERR_CLASS_NOT_SUPPORTED
#define RADIOLIB_ERR_CLASS_NOT_SUPPORTED      (-2002)
#endif

// Initialize static instance pointer
LoRaManager* LoRaManager::instance = nullptr;

// Constructor with configurable frequency band and subband
LoRaManager::LoRaManager(LoRaWANBand_t freqBand, uint8_t subBand) : 
  radio(nullptr),
  node(nullptr),
  joinEUI(0),
  devEUI(0),
  freqBand(freqBand),
  subBand(subBand),
  isJoined(false),
  lastRssi(0),
  lastSnr(0),
  receivedBytes(0),
  lastErrorCode(RADIOLIB_ERR_NONE),
  consecutiveTransmitErrors(0),
  downlinkCallback(nullptr),
  deviceClass(DEVICE_CLASS_A),
  beaconState(BEACON_STATE_IDLE),
  pingSlotPeriodicity(0),
  beaconCallback(nullptr),
  lastBeaconTimestamp(0),
  continuousReception(false),
  nextPingSlotTime(0),
  beaconPeriod(128000), // 128 seconds (standard LoRaWAN beacon period)
  lastBeaconRxTime(0) {
  
  // Set this instance as the active one
  instance = this;
  
  // Initialize arrays
  memset(appKey, 0, sizeof(appKey));
  memset(nwkKey, 0, sizeof(nwkKey));
  memset(receivedData, 0, sizeof(receivedData));
  
  // Log selected frequency band using bandNum instead of name
  Serial.print(F("[LoRaManager] Selected frequency band: "));
  Serial.println(freqBand.bandNum);
  
  Serial.print(F("[LoRaManager] Selected subband: "));
  Serial.println(subBand);
}

// Destructor
LoRaManager::~LoRaManager() {
  // Clean up allocated resources
  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
  
  if (radio != nullptr) {
    delete radio;
    radio = nullptr;
  }
  
  // Clear the instance pointer
  if (instance == this) {
    instance = nullptr;
  }
}

// Get band type based on band number
uint8_t LoRaManager::getBandType() const {
  // Check band number instead of name
  if (freqBand.bandNum == 0) {
    return BAND_TYPE_OTHER;
  }
  
  // Use band number to determine type
  // US915 is typically band number 2
  if (freqBand.bandNum == 2) {
    return BAND_TYPE_US915;
  } 
  // EU868 is typically band number 1
  else if (freqBand.bandNum == 1) {
    return BAND_TYPE_EU868;
  }
  
  return BAND_TYPE_OTHER;
}

// Initialize the LoRa module
bool LoRaManager::begin(int8_t pinCS, int8_t pinDIO1, int8_t pinReset, int8_t pinBusy) {
  // Store the error code
  lastErrorCode = RADIOLIB_ERR_NONE;
  
  // Create a new Module instance
  Module* module = new Module(pinCS, pinDIO1, pinReset, pinBusy);
  
  // Debug output
  Serial.println(F("[LoRaManager] Creating SX1262 instance..."));
  
  // Create a new SX1262 instance
  radio = new SX1262(module);
  
  // Initialize the radio with more detailed error reporting
  Serial.print(F("[SX1262] Initializing ... "));
  
  // Initialize the radio with more robust error handling
  int state = radio->begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    
    // Store the error code
    lastErrorCode = state;
    
    // Additional debug info
    Serial.println(F("[SX1262] Debug info:"));
    Serial.print(F("  CS pin: "));
    Serial.println(pinCS);
    Serial.print(F("  DIO1 pin: "));
    Serial.println(pinDIO1);
    Serial.print(F("  Reset pin: "));
    Serial.println(pinReset);
    Serial.print(F("  Busy pin: "));
    Serial.println(pinBusy);
    
    return false;
  }

  // Log frequency band configuration using band number
  const char* bandName;
  switch(freqBand.bandNum) {
    case 1:
      bandName = "EU868";
      break;
    case 2:
      bandName = "US915";
      break;
    default:
      bandName = "Custom";
  }
  
  Serial.print(F("[LoRaManager] Configuring LoRaWAN for "));
  Serial.print(bandName);
  Serial.println(F(" band..."));
  
  // Initialize the node with the configured region and subband
  // For US915, the subband parameter will automatically configure the correct channels
  node = new LoRaWANNode(radio, &freqBand, subBand);

  // Log detailed band configuration
  Serial.print(F("[LoRaManager] Using "));
  Serial.print(bandName);
  Serial.print(F(" region with subband: "));
  Serial.println(subBand);
  
  // If using US915, log additional information about channels
  uint8_t bandType = getBandType();
  if (bandType == BAND_TYPE_US915) {
    Serial.print(F("[LoRaManager] This will enable channels for subband "));
    Serial.println(subBand);
  }

  // Default values for testing - will be replaced later by setCredentials()
  uint64_t defaultJoinEUI = 0x0000000000000000;
  uint64_t defaultDevEUI = 0x0000000000000000;
  uint8_t defaultNwkKey[16] = {0};
  uint8_t defaultAppKey[16] = {0};
  
  // Initialize node
  Serial.println(F("[LoRaManager] Initializing node..."));
  
  // Initialize with default credentials
  node->beginOTAA(defaultJoinEUI, defaultDevEUI, defaultNwkKey, defaultAppKey);
  
  Serial.println(F("[LoRaManager] LoRaWAN node initialized successfully!"));
  
  return true;
}

// Configure subband channel mask based on the current subband
int LoRaManager::configureSubbandChannels(uint8_t targetSubBand) {
  if (!node) {
    Serial.println(F("[LoRaWAN] Node not initialized"));
    return RADIOLIB_ERR_INVALID_STATE;
  }
  
  // Only applicable for US915
  uint8_t bandType = getBandType();
  if (bandType != BAND_TYPE_US915) {
    Serial.println(F("[LoRaWAN] Subband configuration only applies to US915"));
    return RADIOLIB_ERR_NONE;
  }
  
  // Validate subband (1-8)
  if (targetSubBand < 1 || targetSubBand > 8) {
    Serial.println(F("[LoRaWAN] Invalid subband, must be 1-8"));
    return RADIOLIB_ERR_INVALID_INPUT;
  }
  
  Serial.print(F("[LoRaWAN] Subband configuration is handled automatically during initialization"));
  Serial.print(F(" for subband "));
  Serial.println(targetSubBand);
  
  // In RadioLib 7.1.2, the subband is configured during node initialization
  // and channel selection is handled internally, so we don't need to do anything here
  return RADIOLIB_ERR_NONE;
}

// Convert hex string to byte array
bool LoRaManager::hexStringToByteArray(const String& hexString, uint8_t* result, size_t resultLen) {
  // Check if the string length is valid (2 chars per byte)
  if (hexString.length() != resultLen * 2) {
    Serial.println(F("[LoRaManager] Invalid hex string length"));
    return false;
  }
  
  // Convert each pair of hex chars to a byte
  for (size_t i = 0; i < resultLen; i++) {
    // Get the two hex characters
    String byteStr = hexString.substring(i * 2, i * 2 + 2);
    
    // Convert to integer
    char* endPtr;
    uint8_t byteVal = strtol(byteStr.c_str(), &endPtr, 16);
    
    // Check if conversion was successful
    if (*endPtr != '\0') {
      Serial.println(F("[LoRaManager] Invalid hex character in string"));
      return false;
    }
    
    // Store the byte value
    result[i] = byteVal;
  }
  
  return true;
}

// Set the LoRaWAN credentials
void LoRaManager::setCredentials(uint64_t joinEUI, uint64_t devEUI, uint8_t* appKey, uint8_t* nwkKey) {
  this->joinEUI = joinEUI;
  this->devEUI = devEUI;
  
  // Copy the keys
  memcpy(this->appKey, appKey, 16);
  memcpy(this->nwkKey, nwkKey, 16);
}

// Set the LoRaWAN credentials using hex strings for keys
bool LoRaManager::setCredentialsHex(uint64_t joinEUI, uint64_t devEUI, const String& appKeyHex, const String& nwkKeyHex) {
  this->joinEUI = joinEUI;
  this->devEUI = devEUI;
  
  // Convert hex strings to byte arrays
  bool appKeyResult = hexStringToByteArray(appKeyHex, this->appKey, 16);
  bool nwkKeyResult = hexStringToByteArray(nwkKeyHex, this->nwkKey, 16);
  
  // Return true only if both conversions were successful
  return appKeyResult && nwkKeyResult;
}

// Set the callback function for downlink data
void LoRaManager::setDownlinkCallback(DownlinkCallback callback) {
  this->downlinkCallback = callback;
  Serial.println(F("[LoRaManager] Downlink callback registered"));
}

// Join the LoRaWAN network
bool LoRaManager::joinNetwork() {
  if (node == nullptr) {
    Serial.println(F("[LoRaWAN] Node not initialized!"));
    lastErrorCode = RADIOLIB_ERR_INVALID_STATE;
    return false;
  }
  
  // Maximum number of join attempts
  const uint8_t maxAttempts = 5;
  uint8_t attemptCount = 0;
  
  // Initial backoff delay in milliseconds
  uint16_t backoffDelay = 1000;
  
  // Try multiple times with exponential backoff
  while (attemptCount < maxAttempts) {
    // Increment attempt counter
    attemptCount++;
    
    // Attempt to join the network
    Serial.print(F("[LoRaWAN] Attempting over-the-air activation (attempt "));
    Serial.print(attemptCount);
    Serial.print(F(" of "));
    Serial.print(maxAttempts);
    Serial.print(F(") ... "));
    
    // Set the proper credentials before activation
    node->beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    
    // Select a subband based on the attempt number
    uint8_t currentSubBand = attemptCount == 1 ? subBand : (1 + (attemptCount % 8)); // Start with configured subband, then try others
    
    // Configure channels for the selected subband (US915 only)
    uint8_t bandType = getBandType();
    if (bandType == BAND_TYPE_US915) {
      int maskResult = configureSubbandChannels(currentSubBand);
      
      // If we couldn't set the channel mask, try the next attempt
      if (maskResult != RADIOLIB_ERR_NONE) {
        Serial.println(F("[LoRaWAN] Continuing with default channel configuration"));
      }
    }

    // Try to join the network
    int state = node->activateOTAA();
    lastErrorCode = state;
    
    // Check for successful join or new session status
    if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_LORAWAN_NEW_SESSION) {
      // Successfully joined
      isJoined = true;
      
      // Configure the data rate for reliability
      Serial.println(F("[LoRaWAN] Setting data rate to DR1 for reliability"));
      node->setDatarate(1);
      
      // Reset frame counters to ensure a clean session
      node->resetFCntDown();
      
      // Send an initial small packet to confirm the join and establish the session fully
      uint8_t testData[] = {0x01};
      int sendState = node->sendReceive(testData, sizeof(testData), 1);
      
      if (sendState == RADIOLIB_ERR_NONE || sendState > 0) {
        // Successfully sent the initial packet and potentially received a downlink
        Serial.println(F("success! (new session started)"));
        return true;
      } else {
        // Session started but initial message failed
        Serial.println(F("session started but initial message failed, code "));
        Serial.println(sendState);
        // We'll still consider this a success since the join worked
        return true;
      }
    } else {
      // Join attempt failed
      Serial.print(F("failed, code "));
      Serial.println(state);
      
      if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
        // Node rejected by network - try a different subband or wait longer
        Serial.println(F("[LoRaWAN] Rejected by network. Will try again with different parameters."));
      } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // Signal problem - check antenna, power, or location
        Serial.println(F("[LoRaWAN] Transmission timeout. Check antenna, signal strength, or move to better location."));
      } else {
        // Other error, print for debugging
        Serial.print(F("[LoRaWAN] Error code: "));
        Serial.println(state);
      }
      
      // Wait a bit before the next attempt (with exponential backoff)
      delay(backoffDelay);
      backoffDelay *= 2;  // Exponential backoff
      
      // Cap the backoff delay at 30 seconds
      if (backoffDelay > 30000) {
        backoffDelay = 30000;
      }
    }
  }
  
  // If we got here, all attempts failed
  isJoined = false;
  lastErrorCode = RADIOLIB_ERR_NETWORK_NOT_JOINED;
  Serial.println(F("[LoRaWAN] Failed to join after maximum attempts."));
  return false;
}

// Send data to the LoRaWAN network
bool LoRaManager::sendData(uint8_t* data, size_t len, uint8_t port, bool confirmed) {
  // Check if we are joined to the network
  if (!isJoined) {
    Serial.println(F("[LoRaWAN] Not joined to network, cannot send data"));
    lastErrorCode = RADIOLIB_ERR_NETWORK_NOT_JOINED;
    return false;
  }
  
  // Check for valid data
  if (data == nullptr || len == 0) {
    Serial.println(F("[LoRaWAN] Invalid data for transmission"));
    lastErrorCode = RADIOLIB_ERR_INVALID_INPUT;
    return false;
  }
  
  // Maximum number of transmission attempts
  const uint8_t maxAttempts = 3;
  uint8_t attemptCount = 0;
  
  // Check if we need to rejoin
  // Since isActive is a protected member, we'll just check our isJoined flag
  if (!isJoined) {
    Serial.println(F("[LoRaWAN] Not joined, attempting to rejoin the network..."));
    if (joinNetwork()) {
      Serial.println(F("[LoRaWAN] Successfully rejoined, will now try to send data"));
    } else {
      Serial.println(F("[LoRaWAN] Rejoin failed, cannot send data"));
      return false;
    }
  }
  
  // Retry loop for sending data
  while (attemptCount < maxAttempts) {
    // Increment attempt counter
    attemptCount++;
    
    // Send the data
    Serial.print(F("[LoRaWAN] Sending data (attempt "));
    Serial.print(attemptCount);
    Serial.print(F(" of "));
    Serial.print(maxAttempts);
    Serial.print(F(") ... "));
    
    // Prepare buffer for downlink
    uint8_t downlinkData[256];
    size_t downlinkLen = sizeof(downlinkData);
    
    // Send data and wait for downlink
    int state = node->sendReceive(data, len, port, downlinkData, &downlinkLen, confirmed);
    lastErrorCode = state;
    
    // Check for successful transmission
    if (state == RADIOLIB_ERR_NONE || state > 0 || state == RADIOLIB_LORAWAN_NO_DOWNLINK) {
      if (state > 0) {
        // Downlink received in window state (1 = RX1, 2 = RX2)
        Serial.print(F("success! Received downlink in RX"));
        Serial.println(state);
        
        // Process the downlink data
        if (downlinkLen > 0) {
          Serial.print(F("[LoRaWAN] Received "));
          Serial.print(downlinkLen);
          Serial.println(F(" bytes:"));
          
          for (size_t i = 0; i < downlinkLen; i++) {
            Serial.print(downlinkData[i], HEX);
            Serial.print(' ');
          }
          Serial.println();
          
          // Call the callback if registered
          if (downlinkCallback != nullptr) {
            downlinkCallback(downlinkData, downlinkLen, port);
          }
          
          // Copy the data to our buffer
          memcpy(receivedData, downlinkData, downlinkLen);
          receivedBytes = downlinkLen;
        }
      } else if (state == RADIOLIB_LORAWAN_NO_DOWNLINK) {
        // No downlink received but uplink was successful
        Serial.println(F("success! No downlink received."));
      } else {
        // General success
        Serial.println(F("success!"));
      }
      
      // Get RSSI and SNR
      lastRssi = radio->getRSSI();
      lastSnr = radio->getSNR();
      
      consecutiveTransmitErrors = 0; // Reset error counter on success
      return true;
    } else {
      // Error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
      
      // Add more specific error handling for common LoRaWAN transmission issues
      bool shouldRetry = false;
      
      // Handle different error cases
      if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        Serial.println(F("[LoRaWAN] Transmission timeout. Check antenna and signal."));
        shouldRetry = true;
      } 
      else if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
        Serial.println(F("[LoRaWAN] Network not joined. Will try to rejoin."));
        isJoined = false; // Force rejoin
        
        // Try to rejoin before next attempt
        if (joinNetwork()) {
          Serial.println(F("[LoRaWAN] Rejoined successfully, will retry transmission."));
          shouldRetry = true;
        } else {
          Serial.println(F("[LoRaWAN] Failed to rejoin, cannot continue."));
          shouldRetry = false;
        }
      }
      else if (state == RADIOLIB_ERR_NO_CHANNEL_AVAILABLE) {
        Serial.println(F("[LoRaWAN] No channel available for the requested data rate."));
        
        // Only try different subbands for US915
        uint8_t bandType = getBandType();
        if (bandType == BAND_TYPE_US915) {
          // Try selecting a different subband for next attempt
          uint8_t alternateSubBand = 1 + (attemptCount % 8); // Try different subbands (1-8)
          Serial.print(F("[LoRaWAN] Will try with subband "));
          Serial.print(alternateSubBand);
          Serial.println(F(" for next attempt"));
          
          // Configure the alternate subband
          int maskResult = configureSubbandChannels(alternateSubBand);
          
          if (maskResult == RADIOLIB_ERR_NONE) {
            shouldRetry = true;
          } else {
            shouldRetry = false;
          }
        } else {
          Serial.println(F("[LoRaWAN] Subband adjustment not applicable for this region"));
          shouldRetry = (attemptCount < maxAttempts);
        }
      }
      else {
        // Default case for other errors
        Serial.println(F("[LoRaWAN] Unknown error during transmission."));
        shouldRetry = (attemptCount < maxAttempts);
      }
      
      // Track consecutive errors
      consecutiveTransmitErrors++;
      
      // If we should retry and have attempts left
      if (shouldRetry && attemptCount < maxAttempts) {
        Serial.print(F("[LoRaWAN] Will retry transmission in 3 seconds (attempt "));
        Serial.print(attemptCount + 1);
        Serial.print(F(" of "));
        Serial.print(maxAttempts);
        Serial.println(F(")"));
        
        // Wait before next attempt
        delay(3000);
      } else {
        // If we've encountered errors multiple times in a row, try rejoining on next transmission
        if (consecutiveTransmitErrors >= 3) {
          Serial.println(F("[LoRaWAN] Multiple transmission errors, will attempt to rejoin on next transmission."));
          isJoined = false; // Force a rejoin on next transmission
        }
        
        return false;
      }
    }
  }
  
  // If we've reached this point, all attempts failed
  Serial.println(F("[LoRaWAN] All transmission attempts failed."));
  return false;
}

// Helper method to send a string
bool LoRaManager::sendString(const String& data, uint8_t port, bool confirmed) {
  return sendData((uint8_t*)data.c_str(), data.length(), port, confirmed);
}

// Get the last RSSI value
float LoRaManager::getLastRssi() {
  return lastRssi;
}

// Get the last SNR value
float LoRaManager::getLastSnr() {
  return lastSnr;
}

// Check if the device is joined to the network
bool LoRaManager::isNetworkJoined() {
  return isJoined;
}

// Get information about RX1 delay (time between uplink end and RX1 window opening)
int LoRaManager::getRx1Delay() const {
  // Default is 1 second for most networks
  return 5; // This is usually configured by the network on join
}

// Get information about RX1 window timeout
int LoRaManager::getRx1Timeout() const {
  // Return the configured timeout, typically around 50ms
  return 50; // This is the default for Class A devices
}

// Get information about RX2 window timeout
int LoRaManager::getRx2Timeout() const {
  // Return the configured timeout, typically around 190ms
  return 190; // This is the default for Class A devices
}

// Handle events (should be called in the loop)
void LoRaManager::handleEvents() {
  if (!node) {
    return;
  }
  
  // Check for state updates 
  int16_t state = node->check();
  
  if (state == RADIOLIB_ERR_NONE) {
    // Check for new data based on flags
    LoRaWANEvent_t event;
    if (node->readEvent(&event)) {
      // Process the event based on type
      if (event.type == RADIOLIB_LORAWAN_EVENT_RX_DATA) {
        // Get the port number
        uint8_t port = event.port;
        
        // Print debug info
        Serial.print(F("[LoRaManager] Received "));
        Serial.print(event.len);
        Serial.println(F(" bytes on port "));
        Serial.println(port);
        
        // Copy data to our buffer
        memcpy(receivedData, event.data, event.len);
        
        // Get signal metrics
        lastRssi = event.rssi;
        lastSnr = event.snr;
        
        // Call the callback if registered
        if (downlinkCallback) {
          downlinkCallback(receivedData, event.len, port);
        }
      } else if (event.type == RADIOLIB_LORAWAN_EVENT_JOIN_ACCEPT) {
        // Join successful
        isJoined = true;
        Serial.println(F("[LoRaManager] Join successful!"));
      }
    }
  } else if (state != RADIOLIB_ERR_NONE) {
    // Log but don't treat as error for transient conditions 
    if (state != RADIOLIB_LORAWAN_NO_DOWNLINK && state != RADIOLIB_ERR_NETWORK_NOT_JOINED) {
      Serial.print(F("[LoRaManager] Error code: "));
      Serial.println(state);
      lastErrorCode = state;
    }
  }
}

// Set the device class (A, B, or C)
bool LoRaManager::setDeviceClass(char deviceClass) {
  if (deviceClass != DEVICE_CLASS_A && deviceClass != DEVICE_CLASS_B && deviceClass != DEVICE_CLASS_C) {
    Serial.println(F("[LoRaWAN] Invalid device class"));
    lastErrorCode = RADIOLIB_ERR_INVALID_INPUT;
    return false;
  }
  
  // If we're already in this class, just return success
  if (this->deviceClass == deviceClass) {
    return true;
  }
  
  // Handle class transitions
  char previousClass = this->deviceClass;
  this->deviceClass = deviceClass;
  
  Serial.print(F("[LoRaWAN] Switching from Class "));
  Serial.print(previousClass);
  Serial.print(F(" to Class "));
  Serial.println(deviceClass);
  
  // Handle specific transitions
  if (deviceClass == DEVICE_CLASS_A) {
    // When switching to Class A, stop any Class B or C specific operations
    if (previousClass == DEVICE_CLASS_B) {
      stopBeaconAcquisition();
    } else if (previousClass == DEVICE_CLASS_C) {
      stopContinuousReception();
    }
    
    Serial.println(F("[LoRaWAN] Switched to Class A mode"));
    return true;
  } else if (deviceClass == DEVICE_CLASS_B) {
    // When switching to Class B, start beacon acquisition
    if (previousClass == DEVICE_CLASS_C) {
      stopContinuousReception();
    }
    
    return startBeaconAcquisition();
  } else if (deviceClass == DEVICE_CLASS_C) {
    // When switching to Class C, start continuous reception
    if (previousClass == DEVICE_CLASS_B) {
      stopBeaconAcquisition();
    }
    
    return startContinuousReception();
  }
  
  // We should never reach here
  return false;
}

// Get the current device class
char LoRaManager::getDeviceClass() const {
  return deviceClass;
}

// Start beacon acquisition (Class B)
bool LoRaManager::startBeaconAcquisition() {
  if (node == nullptr || radio == nullptr) {
    Serial.println(F("[LoRaWAN] Node or radio not initialized"));
    lastErrorCode = RADIOLIB_ERR_INVALID_STATE;
    return false;
  }
  
  if (!isJoined) {
    Serial.println(F("[LoRaWAN] Must be joined to the network before starting beacon acquisition"));
    lastErrorCode = RADIOLIB_ERR_NETWORK_NOT_JOINED;
    return false;
  }
  
  Serial.println(F("[LoRaWAN] Starting beacon acquisition for Class B operation"));
  
  // In a real implementation, we would:
  // 1. Configure the radio for beacon reception frequency
  // 2. Calculate the next beacon time based on GPS time or network time
  // 3. Schedule beacon reception
  
  // For this example, we'll just set the state and handle the details in handleEvents
  beaconState = BEACON_STATE_ACQUISITION;
  lastBeaconRxTime = millis();
  
  // Inform the network server that we're switching to Class B
  // by sending a specific uplink message (not implemented in this example)
  
  return true;
}

// Stop beacon acquisition (Class B)
void LoRaManager::stopBeaconAcquisition() {
  if (beaconState != BEACON_STATE_IDLE) {
    Serial.println(F("[LoRaWAN] Stopping beacon acquisition/tracking"));
    beaconState = BEACON_STATE_IDLE;
    
    // Inform the network server that we're no longer in Class B mode
    // by sending a specific uplink message (not implemented in this example)
  }
}

// Set the ping slot periodicity (Class B)
bool LoRaManager::setPingSlotPeriodicity(uint8_t periodicity) {
  if (periodicity > 7) {
    Serial.println(F("[LoRaWAN] Invalid ping slot periodicity (must be 0-7)"));
    lastErrorCode = RADIOLIB_ERR_INVALID_INPUT;
    return false;
  }
  
  pingSlotPeriodicity = periodicity;
  Serial.print(F("[LoRaWAN] Ping slot periodicity set to "));
  Serial.println(pingSlotPeriodicity);
  
  // If we're already in Class B with a locked beacon, recalculate ping slots
  if (deviceClass == DEVICE_CLASS_B && beaconState == BEACON_STATE_LOCKED) {
    calculateNextPingSlot();
  }
  
  // Inform the network server about the new ping slot periodicity
  // by sending a specific uplink message (not implemented in this example)
  
  return true;
}

// Get the current ping slot periodicity (Class B)
uint8_t LoRaManager::getPingSlotPeriodicity() const {
  return pingSlotPeriodicity;
}

// Get the current beacon state (Class B)
uint8_t LoRaManager::getBeaconState() const {
  return beaconState;
}

// Set the callback function for beacon reception (Class B)
void LoRaManager::setBeaconCallback(BeaconCallback callback) {
  beaconCallback = callback;
  Serial.println(F("[LoRaWAN] Beacon callback registered"));
}

// Handle beacon reception (Class B)
void LoRaManager::handleBeaconReception(uint8_t* payload, size_t size, float rssi, float snr) {
  // Store the timestamp of reception
  lastBeaconTimestamp = millis();
  
  // Update signal quality metrics
  lastRssi = rssi;
  lastSnr = snr;
  
  Serial.print(F("[LoRaWAN] Beacon received - RSSI: "));
  Serial.print(rssi);
  Serial.print(F(" dBm, SNR: "));
  Serial.print(snr);
  Serial.println(F(" dB"));
  
  // Process beacon information (in a real implementation, we would extract time information)
  
  // Call the beacon callback if registered
  if (beaconCallback != nullptr) {
    beaconCallback(payload, size, rssi, snr);
  }
}

// Calculate next ping slot time (Class B)
void LoRaManager::calculateNextPingSlot() {
  // The real implementation would calculate ping slots based on:
  // 1. Device address
  // 2. Beacon time
  // 3. Ping slot periodicity
  
  // For this example, we'll use a simplified approach with random ping slot timing
  // In a real implementation, this would use precise calculations based on the LoRaWAN specification
  
  // Calculate time to next ping slot based on periodicity
  // pingSlotPeriodicity defines how often ping slots occur (0 = most frequent, 7 = least frequent)
  uint32_t pingPeriod = 1 << (5 + pingSlotPeriodicity); // in seconds
  
  // Convert to milliseconds and add some random offset for demonstration
  uint32_t pingOffset = random(500); // Random offset up to 500ms for demonstration
  
  // Set the next ping slot time
  nextPingSlotTime = millis() + (pingPeriod * 1000) + pingOffset;
  
  Serial.print(F("[LoRaWAN] Next ping slot in "));
  Serial.print(pingPeriod);
  Serial.println(F(" seconds (plus offset)"));
}

// Start continuous reception (Class C)
bool LoRaManager::startContinuousReception() {
  if (node == nullptr || radio == nullptr) {
    Serial.println(F("[LoRaWAN] Node or radio not initialized"));
    lastErrorCode = RADIOLIB_ERR_INVALID_STATE;
    return false;
  }
  
  if (!isJoined) {
    Serial.println(F("[LoRaWAN] Must be joined to the network before starting continuous reception"));
    lastErrorCode = RADIOLIB_ERR_NETWORK_NOT_JOINED;
    return false;
  }
  
  Serial.println(F("[LoRaWAN] Starting continuous reception for Class C operation"));
  
  // In a real implementation, we would:
  // 1. Configure the radio for RX2 window frequency and data rate
  // 2. Start continuous reception
  
  // For this example, we'll just set the flag and handle the reception in handleEvents
  continuousReception = true;
  
  // In a real implementation using RadioLib, we would call a method to start continuous reception
  // Actual implementation depends on RadioLib's capabilities
  
  return true;
}

// Stop continuous reception (Class C)
void LoRaManager::stopContinuousReception() {
  if (continuousReception) {
    Serial.println(F("[LoRaWAN] Stopping continuous reception"));
    continuousReception = false;
    
    // In a real implementation using RadioLib, we would call a method to stop continuous reception
  }
}

// Get the last error from LoRaWAN operations
int LoRaManager::getLastErrorCode() {
  return lastErrorCode;
} 