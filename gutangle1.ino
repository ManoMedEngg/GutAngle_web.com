/*
 * =====================================================================================
 * PROJECT: A Smart Belt for Spine and Stomach Wellness (GutAngle)
 * AUTHOR: Manoj V (210822121040)
 * INSTITUTION: Kings Engineering College - Dept of Biomedical Engineering
 * PROCESSOR: ESP32 (WROOM-32)
 * DESCRIPTION: Advanced real-time biosignal acquisition, DSP filtering, 
 * posture classification, and Bluetooth IoMT transmission.
 * =====================================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h" 

// =====================================================================================
// 1. HARDWARE PIN DEFINITIONS
// =====================================================================================
const int EGG_PIN = 32;       // ADC1_CH4 - Electrogastrogram Analog Input
const int FLEX_PIN = 33;      // ADC1_CH5 - Lumbar Flex Sensor Analog Input
const int BUZZER_PIN = 19;    // Digital Output for Active Buzzer Alert System

// =====================================================================================
// 2. SYSTEM CONSTANTS & THRESHOLDS
// =====================================================================================
const int SAMPLE_RATE_MS = 10;          // 100 Hz sampling frequency
const int SERIAL_BAUD_RATE = 115200;    // Serial communication speed

// Posture Thresholds (Degrees)
const float LUMBAR_X_MAX = 25.0;
const float LUMBAR_Y_MAX = 40.0;
const float LUMBAR_Z_MAX = 25.0;
const float FLEX_ANGLE_MAX = 35.0;

// Biosignal Thresholds
const float RESP_RATE_MIN = 8.0;        // Minimum acceptable breathing rate (BPM)
const float RESP_RATE_MAX = 25.0;       // Maximum acceptable breathing rate (BPM)
const float STOMACH_ACTIVITY_MAX = 50.0;// EGG upper threshold 
const float MOTION_ARTIFACT_LIMIT = 5.0;// IMU variance limit to pause EGG processing

// Alert Durations
const unsigned long GASTRIC_ALERT_DURATION = 20000;  // 20 seconds for gastric issues
const unsigned long SPINAL_ALERT_DURATION = 10000;   // 10 seconds for spinal issues

// =====================================================================================
// 3. ENUMS & DATA STRUCTURES
// =====================================================================================
enum SystemState {
  STATE_INIT,
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_ERROR
};

enum AlertType {
  ALERT_NONE = 0,
  ALERT_POSTURE_LUMBAR = 1,
  ALERT_POSTURE_FLEX = 2,
  ALERT_RESPIRATORY = 3,
  ALERT_GASTRIC = 4,
  ALERT_HARDWARE_FAIL = 5
};

struct SensorData {
  float eggRaw;
  float flexRaw;
  float imuX;
  float imuY;
  float imuZ;
};

// Structure to track alert timing
struct AlertPattern {
  bool active;
  bool isPlaying;
  int currentStep;
  unsigned long alertStartTime;
  unsigned long lastBeepTime;
  AlertType currentAlert;
  unsigned long alertEndTime;
};

// =====================================================================================
// 4. GLOBAL OBJECTS & VARIABLES
// =====================================================================================
Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT; 

SystemState currentState = STATE_INIT;
SensorData currentData = {0, 0, 0, 0, 0};

// -- IMU Tracking --
float baselineX = 0.0, baselineY = 0.0, baselineZ = 0.0;
float lumbarX = 0.0, lumbarY = 0.0, lumbarZ = 0.0;
bool calibratedIMU = false;
float motionVariance = 0.0; 

// -- Flex Sensor Tracking --
const int FLEX_BUFFER_SIZE = 15;
float flexBuffer[FLEX_BUFFER_SIZE];
int flexBufferIndex = 0;
float flexZero = 0.0; 
float smoothedFlex = 0.0; 
float flexAngle = 0.0;

// -- EGG & Respiration Tracking --
float eggSmooth = 0.0;
float eggBaseline = 0.0;
float stomachActivity = 0.0;
float respWave = 0.0;
const float softwareGain = 100.0; 
bool isBreathingIn = false;
unsigned long lastBreathTime = 0;
float respRateBPM = 0.0; 

// -- Timing & State Control --
unsigned long lastSampleTime = 0;
unsigned long stateStartTime = 0;
bool systemMuted = false;

// -- Alert Management --
AlertType activeAlert = ALERT_NONE;
AlertPattern alertPattern = {false, false, 0, 0, 0, ALERT_NONE, 0};
bool alertTriggered = false;

// =====================================================================================
// 5. DSP & FILTERING FUNCTIONS
// =====================================================================================

// Adds new flex reading to circular buffer and returns Moving Average
float getMovingAverageFlex(float newValue) {
  flexBuffer[flexBufferIndex] = newValue;
  flexBufferIndex = (flexBufferIndex + 1) % FLEX_BUFFER_SIZE;
  float sum = 0;
  for (int i = 0; i < FLEX_BUFFER_SIZE; i++) {
    sum += flexBuffer[i];
  }
  return sum / FLEX_BUFFER_SIZE;
}

// Fills buffer with initial value to prevent startup drift
void initializeDSPBuffers(float startValue) {
  for (int i = 0; i < FLEX_BUFFER_SIZE; i++) {
    flexBuffer[i] = startValue;
  }
}

// Basic High-Pass filter equivalent for EGG baseline wandering removal
void processEGGSignal(float rawVal) {
  if (motionVariance > MOTION_ARTIFACT_LIMIT) {
    // If user is moving too much, freeze EGG baseline to prevent artifact corruption
    return; 
  }
  
  // Exponential Moving Average (EMA) Low Pass
  eggSmooth = (eggSmooth * 0.90) + (rawVal * 0.10);
  // Ultra-slow EMA for Baseline tracking
  eggBaseline = (eggBaseline * 0.99) + (eggSmooth * 0.01);
  
  // Extract AC component (Activity)
  stomachActivity = abs(eggSmooth - eggBaseline);
  
  // Amplified wave for respiration extraction
  respWave = (eggSmooth - eggBaseline) * softwareGain;
}

// Calculates respiration rate using Zero-Crossing Detection (ZCD) logic
void calculateRespirationRate() {
  unsigned long now = millis();
  
  if (respWave > 1.5 && !isBreathingIn) {
    isBreathingIn = true;
    unsigned long breathDuration = now - lastBreathTime;
    
    // Valid human breath duration constraints (10 to 30 BPM mapping)
    if (breathDuration >= 2000 && breathDuration <= 6000) {
      respRateBPM = 60000.0 / breathDuration;
    }
    lastBreathTime = now;
  } else if (respWave < -1.5) {
    isBreathingIn = false;
  }
  
  // Timeout - if no breath detected for 7 seconds, reset to 0
  if (now - lastBreathTime > 7000) {
    respRateBPM = 0.0;
  }
}

// =====================================================================================
// 6. HARDWARE CONTROL & DIAGNOSTICS
// =====================================================================================

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

// Simple beep function for calibration and testing
void simpleBeep(int durationOn, int durationOff) {
  if (systemMuted) return;
  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationOn);
  digitalWrite(BUZZER_PIN, LOW);
  if (durationOff > 0) delay(durationOff);
}

// Non-blocking beep for alert patterns
void beep(int durationOn) {
  if (systemMuted) return;
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationOn);
  digitalWrite(BUZZER_PIN, LOW);
}

void startAlertPattern(AlertType type) {
  if (alertPattern.active && alertPattern.currentAlert == type) {
    return;
  }
  
  unsigned long now = millis();
  
  // Reset pattern for new alert
  alertPattern.active = true;
  alertPattern.isPlaying = true;
  alertPattern.currentStep = 0;
  alertPattern.alertStartTime = now;
  alertPattern.lastBeepTime = 0;
  alertPattern.currentAlert = type;
  
  // Set alert duration based on type
  if (type == ALERT_GASTRIC) {
    alertPattern.alertEndTime = now + GASTRIC_ALERT_DURATION;
  } else if (type == ALERT_POSTURE_LUMBAR || type == ALERT_POSTURE_FLEX) {
    alertPattern.alertEndTime = now + SPINAL_ALERT_DURATION;
  } else {
    alertPattern.alertEndTime = now + 5000; // 5 seconds for other alerts
  }
}

void stopAlertPattern() {
  alertPattern.active = false;
  alertPattern.isPlaying = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void playGastricAlertPattern() {
  // Pattern: beep beep beep beeeep (repeating for 20 seconds)
  // Step 0: Short beep 1
  // Step 1: Short beep 2
  // Step 2: Short beep 3
  // Step 3: Long beep
  
  const int patternSteps = 4;
  int beepDurations[] = {150, 150, 150, 600};  // Short, Short, Short, Long
  int pauseDurations[] = {100, 100, 100, 200}; // Pauses between beeps
  
  unsigned long now = millis();
  
  if (alertPattern.currentStep < patternSteps) {
    // Check if it's time to start the next beep
    if (alertPattern.lastBeepTime == 0 || 
        (now - alertPattern.lastBeepTime) >= pauseDurations[alertPattern.currentStep - 1]) {
      
      // Play the beep
      beep(beepDurations[alertPattern.currentStep]);
      alertPattern.lastBeepTime = now;
      alertPattern.currentStep++;
      
      // If pattern complete, reset to start for next cycle
      if (alertPattern.currentStep >= patternSteps) {
        alertPattern.currentStep = 0;
        // Add a small delay between pattern repetitions
        delay(300);
      }
    }
  }
}

void playSpinalAlertPattern() {
  // Pattern: Beep beeeep beep beeeeeep (repeating for 10 seconds)
  // Step 0: Short beep
  // Step 1: Medium beep
  // Step 2: Short beep
  // Step 3: Long beep
  
  const int patternSteps = 4;
  int beepDurations[] = {150, 400, 150, 800};  // Short, Medium, Short, Long
  int pauseDurations[] = {100, 150, 100, 200}; // Pauses between beeps
  
  unsigned long now = millis();
  
  if (alertPattern.currentStep < patternSteps) {
    // Check if it's time to start the next beep
    if (alertPattern.lastBeepTime == 0 || 
        (now - alertPattern.lastBeepTime) >= pauseDurations[alertPattern.currentStep - 1]) {
      
      // Play the beep
      beep(beepDurations[alertPattern.currentStep]);
      alertPattern.lastBeepTime = now;
      alertPattern.currentStep++;
      
      // If pattern complete, reset to start for next cycle
      if (alertPattern.currentStep >= patternSteps) {
        alertPattern.currentStep = 0;
        // Add a small delay between pattern repetitions
        delay(300);
      }
    }
  }
}

void playRespiratoryAlertPattern() {
  // Gentle reminder pattern
  const int patternSteps = 2;
  int beepDurations[] = {200, 200};
  int pauseDurations[] = {500, 1000};
  
  unsigned long now = millis();
  
  if (alertPattern.currentStep < patternSteps) {
    if (alertPattern.lastBeepTime == 0 || 
        (now - alertPattern.lastBeepTime) >= pauseDurations[alertPattern.currentStep - 1]) {
      
      beep(beepDurations[alertPattern.currentStep]);
      alertPattern.lastBeepTime = now;
      alertPattern.currentStep++;
      
      if (alertPattern.currentStep >= patternSteps) {
        alertPattern.currentStep = 0;
        delay(1000);
      }
    }
  }
}

void manageAlertBuzzer(AlertType type) {
  if (systemMuted) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  
  unsigned long now = millis();
  
  // Start alert pattern if not already active
  if (!alertPattern.active) {
    startAlertPattern(type);
  }
  
  // Check if alert duration has expired
  if (alertPattern.active && now >= alertPattern.alertEndTime) {
    stopAlertPattern();
    return;
  }
  
  // Play the appropriate pattern based on alert type
  if (alertPattern.active && alertPattern.isPlaying) {
    switch(type) {
      case ALERT_GASTRIC:
        playGastricAlertPattern();
        break;
        
      case ALERT_POSTURE_LUMBAR:
      case ALERT_POSTURE_FLEX:
        playSpinalAlertPattern();
        break;
        
      case ALERT_RESPIRATORY:
        playRespiratoryAlertPattern();
        break;
        
      case ALERT_HARDWARE_FAIL:
        // SOS pattern for hardware failure
        if (alertPattern.currentStep == 0) beep(100);
        else if (alertPattern.currentStep == 1) beep(100);
        else if (alertPattern.currentStep == 2) beep(100);
        else if (alertPattern.currentStep == 3) beep(300);
        else if (alertPattern.currentStep == 4) beep(300);
        else if (alertPattern.currentStep == 5) beep(300);
        else if (alertPattern.currentStep == 6) beep(100);
        else if (alertPattern.currentStep == 7) beep(100);
        else if (alertPattern.currentStep == 8) beep(100);
        
        alertPattern.currentStep++;
        if (alertPattern.currentStep >= 9) {
          alertPattern.currentStep = 0;
          delay(1000);
        }
        break;
        
      default:
        stopAlertPattern();
        break;
    }
  }
}

// =====================================================================================
// 7. BLUETOOTH COMMAND PARSER
// =====================================================================================

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim(); // Remove whitespace
    command.toUpperCase();
    
    if (command == "MUTE") {
      systemMuted = true;
      stopAlertPattern();
      digitalWrite(BUZZER_PIN, LOW);
      SerialBT.println("MSG: System Muted.");
      Serial.println(">>> BT COMMAND: Audio Muted");
    } 
    else if (command == "UNMUTE") {
      systemMuted = false;
      SerialBT.println("MSG: Audio Restored.");
      Serial.println(">>> BT COMMAND: Audio Restored");
    }
    else if (command == "CAL") {
      SerialBT.println("MSG: Recalibrating...");
      Serial.println(">>> BT COMMAND: Force Recalibration");
      currentState = STATE_CALIBRATING;
      stateStartTime = millis();
    }
    else if (command == "STATUS") {
      SerialBT.print("MSG: SYS_OK | Muted:");
      SerialBT.print(systemMuted ? "YES" : "NO");
      SerialBT.print(" | IMU:");
      SerialBT.print(calibratedIMU ? "OK" : "VIRTUAL");
      SerialBT.print(" | Alert:");
      if (activeAlert != ALERT_NONE) {
        switch(activeAlert) {
          case ALERT_POSTURE_LUMBAR: SerialBT.print("SPINAL"); break;
          case ALERT_POSTURE_FLEX: SerialBT.print("SPINAL"); break;
          case ALERT_RESPIRATORY: SerialBT.print("RESP"); break;
          case ALERT_GASTRIC: SerialBT.print("GASTRIC"); break;
          default: SerialBT.print("NONE");
        }
      } else {
        SerialBT.print("NONE");
      }
      SerialBT.println();
    }
  }
}

// =====================================================================================
// 8. CALIBRATION ROUTINE
// =====================================================================================

void runCalibration() {
  Serial.println(F("\n--- SYSTEM CALIBRATION ---"));
  Serial.println(F("Instruction: Sit perfectly upright and still for 5 seconds."));
  simpleBeep(500, 0);
  
  long flexSum = 0;
  long eggSum = 0;
  int validSamples = 0;
  
  // 3-second calibration gathering
  for(int i = 0; i < 30; i++) {
    // 12-bit ADC scaled to 10-bit for mathematical consistency
    float currentFlex = analogRead(FLEX_PIN) / 4.0;
    float currentEgg = analogRead(EGG_PIN) / 4.0;
    
    // Basic fault detection
    if (currentFlex > 10 && currentEgg > 10) { 
      flexSum += currentFlex;
      eggSum += currentEgg;
      validSamples++;
    }
    delay(100);
  }
  
  if (validSamples < 10) {
    Serial.println(F("ERR: Sensor fault during calibration."));
    currentState = STATE_ERROR;
    return;
  }

  flexZero = flexSum / (float)validSamples;
  eggSmooth = eggSum / (float)validSamples;
  eggBaseline = eggSmooth;
  
  initializeDSPBuffers(flexZero);
  
  // MPU6050 Calibration
  if (mpu.begin()) {
    float xSum = 0, ySum = 0, zSum = 0;
    for (int i = 0; i < 50; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      xSum += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      ySum += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
      zSum += atan2(a.acceleration.x, a.acceleration.y) * 180.0 / PI;
      delay(10);
    }
    baselineX = xSum / 50.0;
    baselineY = ySum / 50.0;
    baselineZ = zSum / 50.0;
    calibratedIMU = true;
    Serial.println(F("IMU Calibration: SUCCESS"));
  } else {
    calibratedIMU = false;
    Serial.println(F("IMU Calibration: FAILED (Using Flex Fallback)"));
  }
  
  simpleBeep(200, 100);
  simpleBeep(200, 0);
  Serial.println(F("Calibration Complete. Entering Monitoring Mode."));
  
  currentState = STATE_MONITORING;
}

// =====================================================================================
// 9. CORE ALERT LOGIC
// =====================================================================================

void evaluateSystemAlerts() {
  AlertType previousAlert = activeAlert;
  activeAlert = ALERT_NONE; // Assume safe state initially

  // Check for Gastric issues (Electrogastrography) - HIGHEST PRIORITY
  if (stomachActivity > STOMACH_ACTIVITY_MAX) {
    activeAlert = ALERT_GASTRIC;
  }
  // Check for Spinal issues (Posture)
  else if (abs(lumbarX) > LUMBAR_X_MAX || abs(lumbarY) > LUMBAR_Y_MAX || abs(lumbarZ) > LUMBAR_Z_MAX) {
    activeAlert = ALERT_POSTURE_LUMBAR;
  }
  else if (abs(flexAngle) > FLEX_ANGLE_MAX) {
    activeAlert = ALERT_POSTURE_FLEX;
  }
  // Respiratory checks
  else if ((respRateBPM < RESP_RATE_MIN && respRateBPM > 0) || respRateBPM > RESP_RATE_MAX) {
    activeAlert = ALERT_RESPIRATORY;
  }

  // Handle State Transitions and Logging
  if (activeAlert != ALERT_NONE) {
    manageAlertBuzzer(activeAlert);
    
    // Only print alert text once when it triggers, not continuously
    if (activeAlert != previousAlert) {
      Serial.print(F("\n>>> CRITICAL ALERT DETECTED: "));
      
      if (activeAlert == ALERT_GASTRIC) {
        Serial.println(F("GASTRIC ISSUE - Pattern: beep beep beep beeeep (20 seconds)"));
        SerialBT.println("MSG: ALERT_GASTRIC - Gastric activity abnormal");
      } 
      else if (activeAlert == ALERT_POSTURE_LUMBAR || activeAlert == ALERT_POSTURE_FLEX) {
        Serial.println(F("SPINAL ISSUE - Pattern: Beep beeeep beep beeeeeep (10 seconds)"));
        SerialBT.println("MSG: ALERT_SPINAL - Posture correction needed");
      }
      else if (activeAlert == ALERT_RESPIRATORY) {
        Serial.println(F("RESPIRATORY ISSUE - Pattern: Gentle reminder beeps"));
        SerialBT.println("MSG: ALERT_RESPIRATORY - Breathing rate abnormal");
      }
    }
  } else if (previousAlert != ALERT_NONE) {
    // Alert just cleared
    stopAlertPattern();
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println(F(">>> Alert Cleared. Back to Normal."));
    SerialBT.println(F("MSG: NORMAL"));
  }
}

// =====================================================================================
// 10. MAIN ARDUINO SETUP
// =====================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Initialize IoMT Wireless Interface
  SerialBT.begin("GutAngle_Belt"); 
  
  setupBuzzer();
  simpleBeep(200, 100);
  simpleBeep(200, 0);
  
  Serial.println(F("=========================================="));
  Serial.println(F("  GUTANGLE BIOSIGNAL ACQUISITION SYSTEM   "));
  Serial.println(F("=========================================="));
  Serial.println(F("Booting sequence initiated..."));
  Serial.println(F("Alert Patterns:"));
  Serial.println(F("  - Gastric: beep beep beep beeeep (20 seconds)"));
  Serial.println(F("  - Spinal: Beep beeeep beep beeeeeep (10 seconds)"));
  
  // I2C IMU Initialization
  if (!mpu.begin()) {
    Serial.println(F("WRN: MPU6050 unavailable. Check I2C wiring."));
    simpleBeep(100, 50);
    simpleBeep(100, 50);
    simpleBeep(500, 0);
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Hardware low-pass filter
  }
  
  currentState = STATE_CALIBRATING;
}

// =====================================================================================
// 11. MAIN PROCESSING LOOP
// =====================================================================================

void loop() {
  unsigned long now = millis();
  
  // --- Check for remote commands ---
  handleBluetoothCommands();
  
  // --- State Machine Execution ---
  switch (currentState) {
    
    case STATE_CALIBRATING:
      runCalibration();
      break;
      
    case STATE_ERROR:
      manageAlertBuzzer(ALERT_HARDWARE_FAIL);
      delay(1000); // Wait in error state
      break;
      
    case STATE_MONITORING:
      if (now - lastSampleTime >= SAMPLE_RATE_MS) {
        lastSampleTime = now;
        
        // 1. Acquire Raw Sensor Data
        currentData.eggRaw = analogRead(EGG_PIN) / 4.0;
        currentData.flexRaw = analogRead(FLEX_PIN) / 4.0;
        
        // 2. Process Gastric & Respiratory Data
        processEGGSignal(currentData.eggRaw);
        calculateRespirationRate();
        
        // 3. Process Flex Sensor Data via MA Filter
        smoothedFlex = getMovingAverageFlex(currentData.flexRaw);
        float flexDiff = flexZero - smoothedFlex;
        if (abs(flexDiff) < 1.5) flexDiff = 0.0; // Deadzone filter
        flexAngle = flexDiff * 0.45; // Arbitrary degree mapping
        
        // 4. Process IMU Kinematics
        if (calibratedIMU) {
          sensors_event_t a, g, temp;
          mpu.getEvent(&a, &g, &temp);
          
          float rawX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
          float rawY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
          float rawZ = atan2(a.acceleration.x, a.acceleration.y) * 180.0 / PI;
          
          // Determine motion variance (sum of absolute gyro rates)
          motionVariance = abs(g.gyro.x) + abs(g.gyro.y) + abs(g.gyro.z);
          
          float normX = rawX - baselineX;
          float normY = rawY - baselineY;
          float normZ = rawZ - baselineZ;
          
          // IIR Smoothing for Kinematics
          lumbarX = (lumbarX * 0.8) + (normX * 0.2);
          lumbarY = (lumbarY * 0.8) + (normY * 0.2);
          lumbarZ = (lumbarZ * 0.8) + (normZ * 0.2);
        } else {
          // Virtual IMU Fallback using Flex correlation
          lumbarX = flexAngle * 0.3;
          lumbarY = flexAngle;
          lumbarZ = 0.0;
        }
        
        // 5. Run Classification & Alert Rules
        evaluateSystemAlerts();
        
        // 6. Data Transmission (Wired Text Format)
        Serial.print(F("S:")); Serial.print(stomachActivity, 0);
        Serial.print(F(" F:")); Serial.print(flexAngle, 0);
        Serial.print(F(" X:")); Serial.print(lumbarX, 0);
        Serial.print(F(" Y:")); Serial.print(lumbarY, 0);
        Serial.print(F(" Z:")); Serial.print(lumbarZ, 0);
        Serial.print(F(" R:")); Serial.print(respRateBPM, 0);
        if (activeAlert != ALERT_NONE) {
          Serial.print(F(" !"));
          if (activeAlert == ALERT_GASTRIC) {
            Serial.print("GASTRIC");
          } else if (activeAlert == ALERT_POSTURE_LUMBAR || activeAlert == ALERT_POSTURE_FLEX) {
            Serial.print("SPINAL");
          } else if (activeAlert == ALERT_RESPIRATORY) {
            Serial.print("RESP");
          }
        } else {
          Serial.print(F(" *"));
        }
        Serial.println();

        // 7. Data Transmission (Wireless CSV Format for App)
        // Order: Stomach, Flex, Pitch(X), Roll(Y), Yaw(Z), BPM, AlertCode
        SerialBT.print(stomachActivity, 0); SerialBT.print(",");
        SerialBT.print(flexAngle, 0); SerialBT.print(",");
        SerialBT.print(lumbarX, 0); SerialBT.print(",");
        SerialBT.print(lumbarY, 0); SerialBT.print(",");
        SerialBT.print(lumbarZ, 0); SerialBT.print(",");
        SerialBT.print(respRateBPM, 0); SerialBT.print(",");
        
        // Send alert type as text for better readability
        if (activeAlert == ALERT_GASTRIC) {
          SerialBT.println("GASTRIC");
        } else if (activeAlert == ALERT_POSTURE_LUMBAR || activeAlert == ALERT_POSTURE_FLEX) {
          SerialBT.println("SPINAL");
        } else if (activeAlert == ALERT_RESPIRATORY) {
          SerialBT.println("RESPIRATORY");
        } else {
          SerialBT.println("NONE");
        }
      }
      break;
  }
}