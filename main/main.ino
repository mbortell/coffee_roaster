/*
 * Custom Coffee Roaster Firmware
 * - Fan: Fixed jitter/timing issues (only updates on change).
 * - Fan: Clamped to 20-85% to prevent stall/dropout.
 * - Heater: Standard PWM (For Random Fire SSR).
 */

#include <Adafruit_MAX31855.h>
#include <RBDdimmer.h> 

// --- PIN DEFINITIONS ---
#define MAXDO   3
#define MAXCLK  5
#define CS_ET   4
#define CS_BT   6

#define ZC_PIN  2
#define FAN_PIN 9
#define HEATER_PIN 10

// --- CONFIGURATION ---
#define BAUD_RATE       115200 

// --- OBJECTS ---
Adafruit_MAX31855 thermo_et(MAXCLK, CS_ET, MAXDO);
Adafruit_MAX31855 thermo_bt(MAXCLK, CS_BT, MAXDO);
dimmerLamp dimmer(FAN_PIN); 

// --- VARIABLES ---
String commandString = "";
bool commandComplete = false;

// Targets
int target_heater_power = 0; // 0-100%
int target_fan_power = 0;    // 0-100% from Artisan

// State Tracking (To fix Fan jitter)
int current_fan_power = -1;  // Force update on startup

void setup() {
  Serial.begin(BAUD_RATE);
  commandString.reserve(20);

  // Initialize Heater
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  // FIX: Start with the dimmer state OFF so it doesn't blip on startup
  dimmer.begin(NORMAL_MODE, OFF); 
  dimmer.setPower(0); 

  delay(500); 
  Serial.println("# Roaster Ready.");
}

void loop() {
  processSerialCommands();
  
  // FIX: Only update Fan if target changed (Eliminates Jitter)
  updateFanSafe();

  // REVERTED: Standard PWM for Random Fire SSR
  updateHeaterPWM();
}

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
    
    // HEATER COMMAND (OT1)
    if (commandString.startsWith("OT1")) {
      int separator = commandString.indexOf(';');
      if (separator > 0) {
        String valueStr = commandString.substring(separator + 1);
        target_heater_power = constrain(valueStr.toInt(), 0, 100);
      }
    } 
    // FAN COMMAND (OT2)
    else if (commandString.startsWith("OT2")) {
      int separator = commandString.indexOf(';');
      if (separator > 0) {
        String valueStr = commandString.substring(separator + 1);
        target_fan_power = constrain(valueStr.toInt(), 0, 100);
      }
    }
    // READ COMMAND
    else if (commandString.startsWith("READ")) {
       readAndSendTemperatures(); 
    }
    
    commandString = "";
    commandComplete = false;
  }
}

void updateFanSafe() {
  // 1. Clamp Logic (20% min, 85% max)
  int safe_power = target_fan_power;
  
  if (safe_power > 0 && safe_power < 20) {
    safe_power = 20; 
  } else if (safe_power > 85) {
    safe_power = 85;
  }

  // 2. Anti-Jitter & Auto-On/Off Logic
  if (safe_power != current_fan_power) {
    
    // If we are turning ON from a dead stop
    if (current_fan_power == 0 && safe_power > 0) {
      dimmer.setPower(safe_power);
      dimmer.setState(ON); // Enable the interrupt
    }
    // If we are turning OFF completely
    else if (safe_power == 0) {
      dimmer.setState(OFF); // Disable the interrupt (Silence)
      dimmer.setPower(0);
    }
    // If we are just changing speed while running
    else {
      dimmer.setPower(safe_power);
    }

    current_fan_power = safe_power;
  }
}

void updateHeaterPWM() {
  // Safety: Cut heater if fan is off
  if (current_fan_power == 0) {
    analogWrite(HEATER_PIN, 0);
    return;
  }

  // Standard PWM (0-255) mapping
  // Works for Random Fire SSRs acting as fast switches
  int pwm_val = map(target_heater_power, 0, 100, 0, 255);
  analogWrite(HEATER_PIN, pwm_val);
}

void readAndSendTemperatures() {
  double bt = thermo_bt.readCelsius();
  double et = thermo_et.readCelsius();
  
  if (isnan(bt)) bt = 0.0;
  if (isnan(et)) et = 0.0;

  Serial.print(bt, 1);
  Serial.print(",");
  Serial.println(et, 1);
}
