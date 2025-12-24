/*
 * Custom Coffee Roaster Firmware for Artisan
 * 
 * This sketch controls a coffee roaster based on a West Bend Poppery I,
 * interfacing with Artisan roast logging software. It manages two 
 * thermocouples, a variable-speed AC fan, and a heating element.
 * 
 * HARDWARE:
 * - Arduino Uno
 * - 2x K-Type Thermocouples with MAX31855 Breakouts
 * - 1x RobotDyn AC Dimmer for fan motor (phase-angle control)
 * - 1x Crydom D2425-10 SSR for fan
 * - 1x Crydom D2425 SSR for heater (burst-fire control)
 * 
 * LIBRARIES:
 * - Adafruit_MAX31855.h: For reading thermocouple temperatures.
 * - RBDdimmer.h: For AC fan motor control.
 *   NOTE: You must install the "RBDdimmer" library by RobotDyn from the
 *   Arduino Library Manager for this code to compile.
 */

#include <Adafruit_MAX31855.h>
#include <RBDdimmer.h> // Requires installation of RBDdimmer library

// ============================================================================ 
// PIN DEFINITIONS (Mandatory)
// ============================================================================ 
// Shared SPI for Thermocouples (Software SPI)
#define MAXDO   3  // Data Out
#define MAXCLK  5  // Clock

// Chip Select (CS) for individual thermocouples
#define CS_ET   4  // Environmental Temp (ET)
#define CS_BT   6  // Bean Temp (BT)

// AC Dimmer for Fan Control
#define ZC_PIN  2  // Zero-Cross Detection (MUST be on an interrupt pin)
#define FAN_PIN 7  // PSM/DIM Pin for dimmer control

// ============================================================================ 
// CONFIGURATION
// ============================================================================ 
#define BAUD_RATE       115200 // Serial communication speed for Artisan
#define READ_INTERVAL   500    // Milliseconds between sensor readings

// ============================================================================ 
// GLOBAL OBJECTS
// ============================================================================ 
// Initialize thermocouples using software SPI
Adafruit_MAX31855 thermo_et(MAXCLK, CS_ET, MAXDO); // Environmental Temp
Adafruit_MAX31855 thermo_bt(MAXCLK, CS_BT, MAXDO); // Bean Temp

// Initialize AC dimmer for the fan
// NOTE: The ZC_PIN (2) is handled by the library's interrupt setup.
dimmerLamp dimmer(FAN_PIN); 

// ============================================================================ 
// GLOBAL VARIABLES
// ============================================================================ 
unsigned long lastReadTime = 0;
String commandString = "";
bool commandComplete = false;

// Power levels (0-100%)
int heater_power = 0;
int fan_power = 0;

// ============================================================================ 
// SETUP - Runs once at startup
// ============================================================================ 
void setup() {
  Serial.begin(BAUD_RATE);
  commandString.reserve(20); // Pre-allocate memory for serial commands

  // Initialize Heater Pin
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  // Initialize Fan Dimmer
  dimmer.begin(NORMAL_MODE, ON); // Start in normal mode, with output on
  dimmer.setPower(0); // Start with fan off

  // Initialize Thermocouples
  delay(500); // Wait for MAX31855s to stabilize
  
  Serial.println("# Coffee Roaster Ready. Waiting for Artisan commands.");
}

// ============================================================================ 
// MAIN LOOP - Runs continuously
// ============================================================================ 
void loop() {
  // 1. Check for and parse incoming serial commands from Artisan
  processSerialCommands();

  // 2. Send temperature data to Artisan every READ_INTERVAL
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();
    readAndSendTemperatures();
  }

  // 3. Update hardware based on power variables
  updateFan();
  updateHeater();
}

// ============================================================================ 
// SERIAL COMMAND PROCESSING
// ============================================================================ 
void processSerialCommands() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      commandString += inChar;
    }
  }

  if (commandComplete) {
    commandString.trim();
    // Artisan commands for heater/fan are "OT1;value" and "OT2;value"
    if (commandString.startsWith("OT1")) {
      int separator = commandString.indexOf(';');
      if (separator > 0) {
        String valueStr = commandString.substring(separator + 1);
        heater_power = valueStr.toInt();
        heater_power = constrain(heater_power, 0, 100); // Ensure 0-100 range
      }
    } else if (commandString.startsWith("OT2")) {
      int separator = commandString.indexOf(';');
      if (separator > 0) {
        String valueStr = commandString.substring(separator + 1);
        fan_power = valueStr.toInt();
        fan_power = constrain(fan_power, 0, 100); // Ensure 0-100 range
      }
    }
    // Acknowledge other common Artisan commands
    else if (commandString.startsWith("CHAN") || commandString.startsWith("UNITS")) {
       Serial.println("# OK");
    }
    
    commandString = "";
    commandComplete = false;
  }
}

// ============================================================================ 
// TEMPERATURE READING & SENDING
// ============================================================================ 
void readAndSendTemperatures() {
  // Read temperatures
  double bt = thermo_bt.readCelsius();
  double et = thermo_et.readCelsius();

  // --- Basic Error Handling ---
  // Check for NaN (Not-a-Number) which indicates a read failure
  if (isnan(bt)) {
    // You could use a previous value or send an error code.
    // For simplicity, we'll send 0.00, but log an error.
    Serial.println("# WARNING: Bean Temp (BT) sensor read failed (NaN).");
    bt = 0.00;
  }
  if (isnan(et)) {
    Serial.println("# WARNING: Environment Temp (ET) sensor read failed (NaN).");
    et = 0.00;
  }

  // Check for other MAX31855 fault conditions
  uint8_t bt_error = thermo_bt.readError();
  uint8_t et_error = thermo_et.readError();
  if (bt_error) {
    Serial.print("# WARNING: BT Fault: ");
    if (bt_error & MAX31855_FAULT_OPEN) Serial.println("Open Circuit");
    else if (bt_error & MAX31855_FAULT_SHORT_GND) Serial.println("Short to GND");
    else if (bt_error & MAX31855_FAULT_SHORT_VCC) Serial.println("Short to VCC");
  }
   if (et_error) {
    Serial.print("# WARNING: ET Fault: ");
    if (et_error & MAX31855_FAULT_OPEN) Serial.println("Open Circuit");
    else if (et_error & MAX31855_FAULT_SHORT_GND) Serial.println("Short to GND");
    else if (et_error & MAX31855_FAULT_SHORT_VCC) Serial.println("Short to VCC");
  }

  // Send data in Artisan-compatible CSV format: "BT,ET"
  Serial.print(bt, 2);
  Serial.print(",");
  Serial.println(et, 2);
}

// ============================================================================ 
// HARDWARE CONTROL FUNCTIONS
// ============================================================================ 
void updateFan() {
  // The RBDdimmer library takes a power value from 0-100
  dimmer.setPower(fan_power);
}

void updateHeater() {
  // --- SAFETY CHECK ---
  // Do not allow the heater to run if the fan is off.
  if (fan_power == 0) {
    heater_power = 0;
  }

  // Convert power percentage (0-100) to PWM duty cycle (0-255)
  // This provides a simple "Burst Fire" or Integral Cycle control for the SSR
  int heater_pwm = map(heater_power, 0, 100, 0, 255);
  analogWrite(HEATER_PIN, heater_pwm);
}
