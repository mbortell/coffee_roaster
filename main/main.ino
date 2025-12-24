/*
 * Coffee Roaster Firmware for Artisan
 * MAX31855 Thermocouple Amplifier
 * Arduino Uno
 */

#include "Adafruit_MAX31855.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define MAXCS_BT    10    // Chip Select for Bean Temperature thermocouple
#define HEATER_PIN  8     // Digital pin to control heater relay (optional)

// ============================================================================
// CONFIGURATION
// ============================================================================
#define BAUD_RATE       115200  // Serial communication speed
#define READ_INTERVAL   500     // Milliseconds between readings
#define ARTISAN_FORMAT  true    // Format output for Artisan compatibility

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
Adafruit_MAX31855 thermoBT(MAXCS_BT);  // Bean Temperature sensor

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
unsigned long lastReadTime = 0;
bool heaterState = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void readAndSendTemperature();
void handleThermocoupleError(uint8_t error);
void processSerialCommand();
void parsePIDCommand(String command);

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================
void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  
  // Initialize heater control pin (optional)
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  
  // Wait for MAX31855 to stabilize
  delay(500);
  
  // Send startup message
  Serial.println("# Coffee Roaster Ready");
  Serial.println("# Firmware v1.0");
  
  // Verify thermocouple connection
  if (isnan(thermoBT.readCelsius())) {
    Serial.println("# WARNING: Check thermocouple connection!");
  }
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================
void loop() {
  // Check if it's time to read temperature
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read and send temperature
    readAndSendTemperature();
  }
  
  // Check for incoming commands from Artisan
  if (Serial.available() > 0) {
    processSerialCommand();
  }
}

// ============================================================================
// TEMPERATURE READING FUNCTION
// ============================================================================
void readAndSendTemperature() {
  // Read temperature in Celsius
  double tempC = thermoBT.readCelsius();
  
  // Check for sensor errors
  uint8_t error = thermoBT.readError();
  
  if (error != 0) {
    // Handle thermocouple errors
    handleThermocoupleError(error);
    return;
  }
  
  // Check for invalid reading (NaN)
  if (isnan(tempC)) {
    Serial.println("# ERROR: Invalid temperature reading");
    return;
  }
  
  // Optional: Read internal temperature (cold junction compensation)
  double tempInternal = thermoBT.readInternal();
  
  // Send data to Artisan
  if (ARTISAN_FORMAT) {
    // Artisan expects: BT,ET format (Bean Temp, Environment Temp)
    // If you only have one sensor, send BT and use internal temp for ET
    Serial.print(tempC, 2);
    Serial.print(",");
    Serial.println(tempInternal, 2);
  } else {
    // Simple format: just the temperature
    Serial.println(tempC, 2);
  }
}

// ============================================================================
// ERROR HANDLING FUNCTION
// ============================================================================
void handleThermocoupleError(uint8_t error) {
  Serial.print("# ERROR: ");
  
  if (error & MAX31855_FAULT_OPEN) {
    Serial.println("Thermocouple open circuit - check connections");
  }
  else if (error & MAX31855_FAULT_SHORT_GND) {
    Serial.println("Thermocouple short to ground");
  }
  else if (error & MAX31855_FAULT_SHORT_VCC) {
    Serial.println("Thermocouple short to VCC");
  }
  else {
    Serial.println("Unknown thermocouple fault");
  }
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================
void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();  // Remove whitespace
  
  // Convert to uppercase for easier parsing
  command.toUpperCase();
  
  // Process commands
  if (command == "READ") {
    // Force immediate temperature reading
    readAndSendTemperature();
  }
  else if (command.startsWith("CHAN")) {
    // Artisan sometimes sends CHAN;xxxx to select channels
    // Acknowledge the command
    Serial.println("# OK");
  }
  else if (command.startsWith("PID")) {
    // PID control commands (if implementing heater control)
    // Format: PID;SV;123.4
    parsePIDCommand(command);
  }
  else if (command == "HEATER;ON") {
    // Turn heater on
    digitalWrite(HEATER_PIN, HIGH);
    heaterState = true;
    Serial.println("# Heater ON");
  }
  else if (command == "HEATER;OFF") {
    // Turn heater off
    digitalWrite(HEATER_PIN, LOW);
    heaterState = false;
    Serial.println("# Heater OFF");
  }
  else if (command == "UNITS;F") {
    // Note: This firmware always sends Celsius
    // Artisan handles the conversion
    Serial.println("# Celsius mode");
  }
  else if (command == "UNITS;C") {
    Serial.println("# Celsius mode");
  }
  else {
    // Unknown command - acknowledge anyway
    Serial.println("# OK");
  }
}

// ============================================================================
// PID COMMAND PARSER (for advanced heater control)
// ============================================================================
void parsePIDCommand(String command) {
  // Example: PID;SV;200.5 (Set Value to 200.5°C)
  // This is a placeholder for implementing PID-controlled heating
  
  int firstSemicolon = command.indexOf(';');
  int secondSemicolon = command.indexOf(';', firstSemicolon + 1);
  
  if (secondSemicolon > 0) {
    String pidType = command.substring(firstSemicolon + 1, secondSemicolon);
    String value = command.substring(secondSemicolon + 1);
    
    if (pidType == "SV") {
      // Set Value received - this is the target temperature
      float targetTemp = value.toFloat();
      Serial.print("# Target temperature set: ");
      Serial.println(targetTemp);
      
      // Here you would implement PID logic to control heater
      // based on current temp vs target temp
    }
  }
  
  Serial.println("# OK");
}