/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║                      SAHAARA SENTINEL FIRMWARE                       ║
 * ║                  Smart Fall & Emergency Monitoring System            ║
 * ║                         ESP32 · BLE · MPU6500 · DS18B20              ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * BLE GATT Profile:
 *   Service UUID: 0000AA00-0000-1000-8000-00805f9b34fb
 *   Telemetry Char: 0000AA01-0000-1000-8000-00805f9b34fb (Notify/Read)
 *   Command Char:   0000AA02-0000-1000-8000-00805f9b34fb (Write)
 *
 * Commands:
 *   ARM      - Arm system (enable alerts & buzzer)
 *   DISARM   - Disarm system (disable alerts)
 *   CALIMU   - Calibrate IMU offsets (keep device flat & still for 5s)
 *   TEST     - Self-test (buzzer + LED)
 *   DISMISS  - Clear active fall/motionless alerts
 *   PING     - Returns {"pong":1} for connection test
 *   RESET    - ESP32 software reset
 *   BUZZ_OFF - Force stop buzzer
 *
 * Hardware Connections:
 *   DS18B20 Data -> GPIO 15 (with 4.7k pull-up)
 *   MPU-6500 SDA -> GPIO 21
 *   MPU-6500 SCL -> GPIO 22
 *   Buzzer       -> GPIO 2
 *   LED (status) -> GPIO 13
 *
 * Author: SAHAARA Sentinel Team
 * Version: 2.0
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════
#define PIN_DS18B20    15
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_BUZZER      2
#define PIN_LED        13

// ═══════════════════════════════════════════════════════════════════════════
//  BLE UUIDs
// ═══════════════════════════════════════════════════════════════════════════
#define SVC_UUID   "0000AA00-0000-1000-8000-00805f9b34fb"
#define TEL_UUID   "0000AA01-0000-1000-8000-00805f9b34fb"
#define CMD_UUID   "0000AA02-0000-1000-8000-00805f9b34fb"

// ═══════════════════════════════════════════════════════════════════════════
//  MPU-6500 REGISTERS (compatible with MPU6050/9250)
// ═══════════════════════════════════════════════════════════════════════════
#define MPU_ADDR_A      0x68
#define MPU_ADDR_B      0x69
#define REG_PWR_MGMT1   0x6B
#define REG_ACCEL_CONF  0x1C
#define REG_GYRO_CONF   0x1B
#define REG_ACCEL_OUT   0x3B
#define REG_GYRO_OUT    0x43
#define REG_WHOAMI      0x75

// Sensitivity (ACCEL ±2g, GYRO ±250°/s)
#define ACCEL_LSB_2G    16384.0f
#define GYRO_LSB_250    131.0f

// ═══════════════════════════════════════════════════════════════════════════
//  ALERT AUDIO FREQUENCIES (Hz)
// ═══════════════════════════════════════════════════════════════════════════
#define FREQ_HEARTBEAT   440     // System OK / arm confirmation
#define FREQ_FALL       3136     // Fall detected - urgent
#define FREQ_MOTIONLESS  523     // Motionless soft alert
#define FREQ_RAPID       880     // Rapid movement warning
#define FREQ_TEMP_WARN  1047     // Temperature warning
#define FREQ_TEMP_CRIT  2800     // Temperature critical

// ═══════════════════════════════════════════════════════════════════════════
//  BODY TEMPERATURE THRESHOLDS (°C)
// ═══════════════════════════════════════════════════════════════════════════
#define BODY_TEMP_LOW_WARN     35.0f
#define BODY_TEMP_LOW_CRIT     34.0f
#define BODY_TEMP_HIGH_WARN    37.5f
#define BODY_TEMP_HIGH_CRIT    38.5f
#define BODY_TEMP_EMERGENCY    39.5f
#define BODY_TEMP_SENSOR_LOW   10.0f
#define BODY_TEMP_SENSOR_HIGH  45.0f

// ═══════════════════════════════════════════════════════════════════════════
//  MOTION DETECTION THRESHOLDS
// ═══════════════════════════════════════════════════════════════════════════
#define TILT_WARN_DEG      30.0f
#define TILT_CRIT_DEG      50.0f
#define FALL_G_THRESH       2.5f      // g-force threshold for fall impact
#define STILL_G_MIN         0.85f     // Stillness gravity range
#define STILL_G_MAX         1.15f
#define STILL_GYRO_MAX      4.0f      // Max gyro magnitude for stillness
#define FALL_DEBOUNCE_MS    400UL     // ms to confirm fall
#define STILL_DEBOUNCE_MS  8000UL     // ms of stillness to trigger alert
#define RAPID_ANGVEL_THRESH 35.0f     // °/s threshold for rapid motion

// ═══════════════════════════════════════════════════════════════════════════
//  SYSTEM TIMING (ms)
// ═══════════════════════════════════════════════════════════════════════════
#define IMU_READ_INTERVAL     20      // 50 Hz
#define TEMP_READ_INTERVAL   2000     // 2 seconds
#define BROADCAST_INTERVAL   500      // 2 Hz telemetry
#define LED_BLINK_NORMAL    1200      // Normal blink interval

// ═══════════════════════════════════════════════════════════════════════════
//  GLOBAL STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════
struct IMUCalibration {
  float axOff = 0, ayOff = 0, azOff = 0;
  float gxOff = 0, gyOff = 0, gzOff = 0;
  bool  valid = false;
};

struct TelemetryData {
  // Temperature
  float    bodyTemp       = 36.5f;
  bool     tempSensorOk   = false;
  char     tempStatus[16] = "NORMAL";
  
  // IMU raw & derived
  float    ax = 0, ay = 0, az = 1.0f;
  float    gx = 0, gy = 0, gz = 0;
  float    pitch = 0, roll = 0;
  float    tilt  = 0;
  float    totalG     = 1.0f;
  float    angVel     = 0.0f;
  char     motion[12] = "STILL";
  
  // Alert flags
  bool     fall        = false;
  bool     motionless  = false;
  bool     tiltWarn    = false;
  bool     tiltCrit    = false;
  bool     rapidMotion = false;
  
  // Timing debounce
  unsigned long fallTs  = 0;
  unsigned long stillTs = 0;
  
  // Risk & alert levels
  float    riskIdx     = 0.0f;
  char     alertLevel[16] = "SAFE";
  bool     alarmActive = false;
  char     alertReason[32] = "NONE";
  
  // System state
  uint32_t uptime      = 0;
  bool     armed       = true;
  bool     calibrated  = false;
  bool     mpuOnline   = false;
  uint16_t packetSeq   = 0;
};

// Global instances
IMUCalibration imuCal;
TelemetryData  telem;
BLEServer*         pServer  = nullptr;
BLECharacteristic* pTelChar = nullptr;
BLECharacteristic* pCmdChar = nullptr;
bool bleConnected = false;

// ═══════════════════════════════════════════════════════════════════════════
//  BUZZER UTILITIES
// ═══════════════════════════════════════════════════════════════════════════
void beep(uint32_t frequency, uint32_t durationMs) {
  if (!telem.armed || frequency == 0) return;
  ledcAttach(PIN_BUZZER, frequency, 8);
  ledcWrite(PIN_BUZZER, 128);  // 50% duty cycle
  delay(durationMs);
  ledcWrite(PIN_BUZZER, 0);
  ledcDetach(PIN_BUZZER);
}

void beepPattern(uint32_t frequency, int repetitions, uint32_t onMs, uint32_t offMs) {
  for (int i = 0; i < repetitions; i++) {
    beep(frequency, onMs);
    if (i < repetitions - 1) delay(offMs);
  }
}

void selfTest() {
  Serial.println("[TEST] System self-test starting...");
  beepPattern(FREQ_HEARTBEAT, 2, 80, 60);
  delay(100);
  beepPattern(FREQ_FALL, 3, 150, 100);
  delay(100);
  beepPattern(FREQ_RAPID, 2, 100, 80);
  Serial.println("[TEST] Self-test complete");
}

void ledSet(bool on) { 
  digitalWrite(PIN_LED, on ? HIGH : LOW); 
}

// ═══════════════════════════════════════════════════════════════════════════
//  MPU-6500 FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════
bool initMPU() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  delay(150);
  
  uint8_t candidates[] = { MPU_ADDR_A, MPU_ADDR_B };
  uint8_t foundAddr = 0;
  
  for (auto addr : candidates) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      foundAddr = addr;
      break;
    }
  }
  
  if (!foundAddr) {
    Serial.println("[MPU] ERROR: No MPU device found - using simulation mode");
    return false;
  }
  
  // Wake up MPU (write 0 to power management)
  Wire.beginTransmission(foundAddr);
  Wire.write(REG_PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  // Configure accelerometer (±2g)
  Wire.beginTransmission(foundAddr);
  Wire.write(REG_ACCEL_CONF);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Configure gyroscope (±250°/s)
  Wire.beginTransmission(foundAddr);
  Wire.write(REG_GYRO_CONF);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
  
  Serial.printf("[MPU] Initialized at address 0x%02X\n", foundAddr);
  return true;
}

void calibrateIMU() {
  if (!telem.mpuOnline) {
    Serial.println("[CAL] Cannot calibrate - IMU offline");
    return;
  }
  
  Serial.println("[CAL] Keep device FLAT and STILL for 5 seconds...");
  beep(FREQ_HEARTBEAT, 300);
  delay(2000);
  
  double axSum = 0, aySum = 0, azSum = 0;
  double gxSum = 0, gySum = 0, gzSum = 0;
  const int samples = 600;  // 600 samples @ ~10ms = 6 seconds
  
  uint8_t addr = MPU_ADDR_A;
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) addr = MPU_ADDR_B;
  
  for (int i = 0; i < samples; i++) {
    // Read accelerometer
    Wire.beginTransmission(addr);
    Wire.write(REG_ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)6);
    if (Wire.available() >= 6) {
      int16_t axRaw = (Wire.read() << 8) | Wire.read();
      int16_t ayRaw = (Wire.read() << 8) | Wire.read();
      int16_t azRaw = (Wire.read() << 8) | Wire.read();
      axSum += axRaw / ACCEL_LSB_2G;
      aySum += ayRaw / ACCEL_LSB_2G;
      azSum += azRaw / ACCEL_LSB_2G;
    }
    
    // Read gyroscope
    Wire.beginTransmission(addr);
    Wire.write(REG_GYRO_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)6);
    if (Wire.available() >= 6) {
      int16_t gxRaw = (Wire.read() << 8) | Wire.read();
      int16_t gyRaw = (Wire.read() << 8) | Wire.read();
      int16_t gzRaw = (Wire.read() << 8) | Wire.read();
      gxSum += gxRaw / GYRO_LSB_250;
      gySum += gyRaw / GYRO_LSB_250;
      gzSum += gzRaw / GYRO_LSB_250;
    }
    delay(5);
  }
  
  imuCal.axOff = axSum / samples;
  imuCal.ayOff = aySum / samples;
  imuCal.azOff = (azSum / samples) - 1.0f;  // Expect 1g on Z
  imuCal.gxOff = gxSum / samples;
  imuCal.gyOff = gySum / samples;
  imuCal.gzOff = gzSum / samples;
  imuCal.valid = true;
  
  Serial.printf("[CAL] Offsets applied: axOff=%.3f, ayOff=%.3f, azOff=%.3f\n",
                imuCal.axOff, imuCal.ayOff, imuCal.azOff);
  beepPattern(FREQ_HEARTBEAT, 3, 80, 60);
  telem.calibrated = true;
}

void readMPU() {
  uint8_t addr = MPU_ADDR_A;
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) {
    addr = MPU_ADDR_B;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) return;  // Both addresses failed
  }
  
  // Read accelerometer
  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() >= 6) {
    int16_t axRaw = (Wire.read() << 8) | Wire.read();
    int16_t ayRaw = (Wire.read() << 8) | Wire.read();
    int16_t azRaw = (Wire.read() << 8) | Wire.read();
    
    telem.ax = axRaw / ACCEL_LSB_2G - imuCal.axOff;
    telem.ay = ayRaw / ACCEL_LSB_2G - imuCal.ayOff;
    telem.az = azRaw / ACCEL_LSB_2G - imuCal.azOff;
  }
  
  // Read gyroscope
  Wire.beginTransmission(addr);
  Wire.write(REG_GYRO_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() >= 6) {
    int16_t gxRaw = (Wire.read() << 8) | Wire.read();
    int16_t gyRaw = (Wire.read() << 8) | Wire.read();
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();
    
    telem.gx = gxRaw / GYRO_LSB_250 - imuCal.gxOff;
    telem.gy = gyRaw / GYRO_LSB_250 - imuCal.gyOff;
    telem.gz = gzRaw / GYRO_LSB_250 - imuCal.gzOff;
  }
  
  // Calculate derived values
  telem.totalG = sqrtf(telem.ax * telem.ax + telem.ay * telem.ay + telem.az * telem.az);
  telem.pitch = atan2f(-telem.ax, sqrtf(telem.ay * telem.ay + telem.az * telem.az)) * 180.0f / M_PI;
  telem.roll  = atan2f(telem.ay, telem.az) * 180.0f / M_PI;
  telem.tilt  = sqrtf(telem.pitch * telem.pitch + telem.roll * telem.roll);
  telem.angVel = sqrtf(telem.gx * telem.gx + telem.gy * telem.gy + telem.gz * telem.gz);
  
  // Motion classification
  if      (telem.angVel <  2.0f) strcpy(telem.motion, "STILL");
  else if (telem.angVel < 12.0f) strcpy(telem.motion, "WALKING");
  else if (telem.angVel < 35.0f) strcpy(telem.motion, "ACTIVE");
  else                           strcpy(telem.motion, "RAPID");
  
  telem.rapidMotion = (telem.angVel >= RAPID_ANGVEL_THRESH);
  telem.tiltWarn = (telem.tilt > TILT_WARN_DEG);
  telem.tiltCrit = (telem.tilt > TILT_CRIT_DEG);
  
  unsigned long now = millis();
  
  // Fall detection (impact + tilt)
  bool fallImpact = (telem.totalG > FALL_G_THRESH);
  if (fallImpact) {
    if (telem.fallTs == 0) telem.fallTs = now;
    telem.fall = ((now - telem.fallTs) >= FALL_DEBOUNCE_MS);
  } else {
    telem.fallTs = 0;
    telem.fall = false;
  }
  
  // Motionless detection (stable position, minimal movement)
  bool isStill = (!telem.fall && 
                  telem.totalG >= STILL_G_MIN && 
                  telem.totalG <= STILL_G_MAX &&
                  telem.angVel < STILL_GYRO_MAX);
  
  if (isStill) {
    if (telem.stillTs == 0) telem.stillTs = now;
    telem.motionless = ((now - telem.stillTs) >= STILL_DEBOUNCE_MS);
  } else {
    telem.stillTs = 0;
    telem.motionless = false;
  }
}

// Simulation mode for testing without MPU
static float simPhase = 0.0f;
void simulateMPU() {
  simPhase += 0.15f;
  telem.totalG = 1.0f + sinf(simPhase * 0.12f) * 0.08f;
  telem.angVel = 8.0f + sinf(simPhase * 0.25f) * 25.0f;
  telem.rapidMotion = (telem.angVel >= RAPID_ANGVEL_THRESH);
  telem.fall = false;
  telem.motionless = false;
  telem.tilt = sinf(simPhase * 0.1f) * 15.0f;
  
  if (telem.angVel < 2) strcpy(telem.motion, "STILL");
  else if (telem.angVel < 12) strcpy(telem.motion, "WALKING");
  else if (telem.angVel < 35) strcpy(telem.motion, "ACTIVE");
  else strcpy(telem.motion, "RAPID");
}

// ═══════════════════════════════════════════════════════════════════════════
//  DS18B20 TEMPERATURE SENSOR
// ═══════════════════════════════════════════════════════════════════════════
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);
unsigned long lastTempRead = 0;

void readBodyTemperature() {
  if (millis() - lastTempRead < TEMP_READ_INTERVAL) return;
  lastTempRead = millis();
  
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  
  if (temp != DEVICE_DISCONNECTED_C && temp > BODY_TEMP_SENSOR_LOW && temp < BODY_TEMP_SENSOR_HIGH) {
    telem.bodyTemp = temp;
    telem.tempSensorOk = true;
  } else {
    telem.tempSensorOk = false;
    return;
  }
  
  // Classify temperature status
  if      (telem.bodyTemp >= BODY_TEMP_EMERGENCY)  strcpy(telem.tempStatus, "EMERGENCY");
  else if (telem.bodyTemp >= BODY_TEMP_HIGH_CRIT)  strcpy(telem.tempStatus, "CRITICAL");
  else if (telem.bodyTemp >= BODY_TEMP_HIGH_WARN)  strcpy(telem.tempStatus, "FEVER");
  else if (telem.bodyTemp <= BODY_TEMP_LOW_CRIT)   strcpy(telem.tempStatus, "HYPOTHERMIA");
  else if (telem.bodyTemp <= BODY_TEMP_LOW_WARN)   strcpy(telem.tempStatus, "LOW");
  else                                              strcpy(telem.tempStatus, "NORMAL");
}

// ═══════════════════════════════════════════════════════════════════════════
//  RISK INDEX CALCULATION
// ═══════════════════════════════════════════════════════════════════════════
void calculateRisk() {
  float risk = 0;
  strcpy(telem.alertReason, "NONE");
  
  // Priority 1: Fall detected (highest)
  if (telem.fall) {
    risk += 85;
    strcpy(telem.alertReason, "FALL DETECTED");
  }
  // Priority 2: Rapid motion
  else if (telem.rapidMotion) {
    risk += 50;
    strcpy(telem.alertReason, "RAPID MOTION");
  }
  // Priority 3: Motionless
  else if (telem.motionless) {
    risk += 60;
    strcpy(telem.alertReason, "MOTIONLESS");
  }
  
  // Temperature contribution (max 30 points)
  if (telem.tempSensorOk) {
    if      (strcmp(telem.tempStatus, "EMERGENCY")  == 0) risk += 30;
    else if (strcmp(telem.tempStatus, "CRITICAL")   == 0) risk += 22;
    else if (strcmp(telem.tempStatus, "HYPOTHERMIA")== 0) risk += 25;
    else if (strcmp(telem.tempStatus, "FEVER")      == 0) risk += 14;
    else if (strcmp(telem.tempStatus, "LOW")        == 0) risk += 10;
  }
  
  // Tilt contribution (max 15)
  if      (telem.tiltCrit) risk += 15;
  else if (telem.tiltWarn) risk += 7;
  
  telem.riskIdx = constrain(risk, 0.0f, 100.0f);
  telem.alarmActive = (telem.riskIdx >= 25);
  
  // Set alert level string
  if      (telem.riskIdx < 25)  strcpy(telem.alertLevel, "SAFE");
  else if (telem.riskIdx < 45)  strcpy(telem.alertLevel, "CAUTION");
  else if (telem.riskIdx < 65)  strcpy(telem.alertLevel, "WARNING");
  else if (telem.riskIdx < 85)  strcpy(telem.alertLevel, "DANGER");
  else                          strcpy(telem.alertLevel, "CRITICAL");
}

// ═══════════════════════════════════════════════════════════════════════════
//  AUDIO ALARM MANAGER
// ═══════════════════════════════════════════════════════════════════════════
unsigned long lastAlarmBeep = 0;

void updateAlarm() {
  if (!telem.armed) return;
  
  unsigned long now = millis();
  
  // Priority 1: Fall - urgent fast beeps
  if (telem.fall && (now - lastAlarmBeep) > 200) {
    beep(FREQ_FALL, 250);
    lastAlarmBeep = now;
    return;
  }
  
  // Priority 2: Rapid motion - warning beeps
  if (telem.rapidMotion && (now - lastAlarmBeep) > 500) {
    beep(FREQ_RAPID, 200);
    lastAlarmBeep = now;
    return;
  }
  
  // Priority 3: Motionless - soft periodic beep (only if no fall/rapid)
  if (telem.motionless && !telem.fall && !telem.rapidMotion && (now - lastAlarmBeep) > 4000) {
    beep(FREQ_MOTIONLESS, 300);
    lastAlarmBeep = now;
    return;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLE COMMUNICATION
// ═══════════════════════════════════════════════════════════════════════════
void broadcastTelemetry() {
  if (!bleConnected || !pTelChar) return;
  
  char buffer[512];
  snprintf(buffer, sizeof(buffer),
    "{"
    "\"seq\":%u,"
    "\"up\":%lu,"
    "\"bt\":%.2f,"
    "\"btok\":%s,"
    "\"bts\":\"%s\","
    "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
    "\"p\":%.1f,\"r\":%.1f,\"tl\":%.1f,"
    "\"tg\":%.3f,\"av\":%.1f,"
    "\"mv\":\"%s\","
    "\"fl\":%s,"
    "\"ml\":%s,"
    "\"rm\":%s,"
    "\"ri\":%.1f,"
    "\"al\":\"%s\","
    "\"ar\":\"%s\","
    "\"arm\":%s"
    "}",
    telem.packetSeq++,
    (unsigned long)telem.uptime,
    telem.bodyTemp,
    telem.tempSensorOk ? "true" : "false",
    telem.tempStatus,
    telem.ax, telem.ay, telem.az,
    telem.pitch, telem.roll, telem.tilt,
    telem.totalG, telem.angVel,
    telem.motion,
    telem.fall ? "true" : "false",
    telem.motionless ? "true" : "false",
    telem.rapidMotion ? "true" : "false",
    telem.riskIdx,
    telem.alertLevel,
    telem.alertReason,
    telem.armed ? "true" : "false"
  );
  
  pTelChar->setValue((uint8_t*)buffer, strlen(buffer));
  pTelChar->notify();
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLE CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    ledSet(true);
    Serial.println("[BLE] Client connected");
    beep(FREQ_HEARTBEAT, 80);
  }
  
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    ledSet(false);
    Serial.println("[BLE] Client disconnected");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String command = pCharacteristic->getValue();
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[CMD] Received: %s\n", command.c_str());
    
    if (command == "ARM") {
      telem.armed = true;
      beep(FREQ_HEARTBEAT, 100);
      Serial.println("[CMD] System armed");
    }
    else if (command == "DISARM") {
      telem.armed = false;
      beep(FREQ_HEARTBEAT, 80);
      Serial.println("[CMD] System disarmed");
    }
    else if (command == "CALIMU") {
      calibrateIMU();
    }
    else if (command == "DISMISS") {
      telem.fall = false;
      telem.fallTs = 0;
      telem.stillTs = 0;
      telem.motionless = false;
      telem.rapidMotion = false;
      beep(FREQ_HEARTBEAT, 100);
      Serial.println("[CMD] Alerts dismissed");
    }
    else if (command == "TEST") {
      telem.armed = true;
      selfTest();
    }
    else if (command == "RESET") {
      Serial.println("[CMD] System reset initiated");
      delay(100);
      ESP.restart();
    }
    else if (command == "PING") {
      if (pTelChar) {
        pTelChar->setValue((uint8_t*)"{\"pong\":1}", 10);
        pTelChar->notify();
      }
      Serial.println("[CMD] PONG sent");
    }
    else if (command == "BUZZ_OFF") {
      // Force stop buzzer - clear interval via alarm logic
      telem.fall = false;
      telem.rapidMotion = false;
      telem.motionless = false;
      Serial.println("[CMD] Buzzer forced off");
    }
    else {
      Serial.printf("[CMD] Unknown command: %s\n", command.c_str());
    }
  }
};

// ═══════════════════════════════════════════════════════════════════════════
//  LED STATUS INDICATOR
// ═══════════════════════════════════════════════════════════════════════════
unsigned long lastLedBlink = 0;
bool ledState = false;

void updateLedIndicator() {
  unsigned long now = millis();
  uint32_t blinkInterval = LED_BLINK_NORMAL;
  
  // Faster blink for active alerts
  if (telem.riskIdx >= 65) blinkInterval = 80;
  else if (telem.riskIdx >= 25) blinkInterval = 250;
  else if (!bleConnected) blinkInterval = 500;
  
  if ((now - lastLedBlink) >= blinkInterval) {
    ledState = !ledState;
    if (bleConnected) {
      ledSet(ledState);
    } else {
      // When disconnected, blink to indicate waiting for connection
      ledSet(ledState);
    }
    lastLedBlink = now;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║                  SAHAARA SENTINEL v2.0                      ║");
  Serial.println("║           Smart Fall & Emergency Monitoring System          ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝\n");
  
  // Initialize pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  ledSet(false);
  
  // Initialize temperature sensor
  sensors.begin();
  Serial.println("[INIT] DS18B20 initialized");
  
  // Initialize MPU
  telem.mpuOnline = initMPU();
  if (telem.mpuOnline) {
    delay(2000);
    calibrateIMU();
  } else {
    Serial.println("[INIT] MPU not found - running in SIMULATION mode");
  }
  
  // Initial temperature read
  readBodyTemperature();
  Serial.printf("[INIT] Body temperature: %.2f °C (%s)\n", telem.bodyTemp, telem.tempStatus);
  
  // Initialize BLE
  BLEDevice::init("SAHAARA-Sentinel");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService* pService = pServer->createService(SVC_UUID);
  
  // Telemetry characteristic (Notify + Read)
  pTelChar = pService->createCharacteristic(
    TEL_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelChar->addDescriptor(new BLE2902());
  
  // Command characteristic (Write)
  pCmdChar = pService->createCharacteristic(
    CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CommandCallbacks());
  
  pService->start();
  
  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SVC_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("[BLE] Device: SAHAARA-Sentinel");
  Serial.printf("[BLE] Service UUID: %s\n", SVC_UUID);
  Serial.println("[BLE] Advertising started");
  
  // Startup indication
  beepPattern(FREQ_HEARTBEAT, 2, 80, 60);
  ledSet(true);
  delay(500);
  ledSet(false);
  
  Serial.println("\n✓ SYSTEM OPERATIONAL\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════
unsigned long lastImuRead = 0;
unsigned long lastBroadcast = 0;

void loop() {
  unsigned long now = millis();
  
  // Update uptime
  telem.uptime = now / 1000;
  
  // Read IMU at 50Hz
  if (now - lastImuRead >= IMU_READ_INTERVAL) {
    if (telem.mpuOnline) {
      readMPU();
    } else {
      simulateMPU();
    }
    lastImuRead = now;
  }
  
  // Read temperature periodically
  readBodyTemperature();
  
  // Calculate risk index after sensor updates
  calculateRisk();
  
  // Buzzer alarm control
  updateAlarm();
  
  // Broadcast telemetry to BLE clients
  if (now - lastBroadcast >= BROADCAST_INTERVAL) {
    broadcastTelemetry();
    lastBroadcast = now;
  }
  
  // LED status indicator
  updateLedIndicator();
  
  // Small delay for stability
  delay(5);
}