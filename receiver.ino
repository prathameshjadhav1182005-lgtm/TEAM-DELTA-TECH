// ============================================================
//  CareAlert — ESP32 Receiver Firmware
//  Hardware : ESP32 DevKit + NRF24L01 + Active/Passive Buzzer
//  BLE      : Nordic UART Service (NUS)
//             Device name : "CareAlert-Rx"
//  Dashboard: Open care_alert_ble_dashboard.html in Chrome,
//             tap "Connect BLE", pick "CareAlert-Rx"
//
//  Libraries needed (install via Arduino Library Manager):
//    - RF24          by TMRh20
//    - ArduinoJson   by Benoit Blanchon  (v6.x)
//    - ESP32 BLE     (built-in with esp32 board package)
//
//  Board: "ESP32 Dev Module"  (Tools > Board)
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoJson.h>

// BLE headers (included with the ESP32 Arduino core)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
//  PIN DEFINITIONS  — change to match your wiring
// ============================================================
#define NRF_CE_PIN    4    // NRF24L01 CE  → ESP32 GPIO 4
#define NRF_CSN_PIN   5    // NRF24L01 CSN → ESP32 GPIO 5
#define BUZZER_PIN    26   // Buzzer signal → ESP32 GPIO 26

// NRF24L01 default SPI pins on ESP32:
//   MOSI → GPIO 23
//   MISO → GPIO 19
//   SCK  → GPIO 18

// ============================================================
//  NRF24L01 CONFIG  — must match the transmitter exactly
// ============================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint8_t RF_ADDRESS[] = "CARE1";   // 5-byte address

// ============================================================
//  DATA PACKET  — must match the transmitter struct exactly
// ============================================================
struct DataPacket {
  uint8_t  patientID;   // 1–50
  bool     emergency;
  uint32_t timestamp;
};

DataPacket inPkt;

// ============================================================
//  PATIENT STATE  (tracks up to 50 patients in RAM)
// ============================================================
#define MAX_PATIENTS 50

struct Patient {
  bool     sosActive;
  uint32_t alertMillis;
};

Patient patients[MAX_PATIENTS];  // index 0 = patient ID 1

// ============================================================
//  BUZZER STATE
// ============================================================
bool          buzzerOn       = false;
unsigned long buzzerStart    = 0;
unsigned long lastBeepMillis = 0;
const uint32_t BUZZ_DURATION_MS = 15000; // buzz for 15 s
const uint32_t BEEP_INTERVAL_MS = 700;   // beep every 700 ms
const uint16_t BEEP_FREQ_HZ     = 2400;
const uint16_t BEEP_DURATION_MS = 250;

void startBuzzer() {
  buzzerOn    = true;
  buzzerStart = millis();
  tone(BUZZER_PIN, BEEP_FREQ_HZ, BEEP_DURATION_MS);
  lastBeepMillis = millis();
}

void updateBuzzer() {
  if (!buzzerOn) return;
  if (millis() - buzzerStart >= BUZZ_DURATION_MS) {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    return;
  }
  if (millis() - lastBeepMillis >= BEEP_INTERVAL_MS) {
    tone(BUZZER_PIN, BEEP_FREQ_HZ, BEEP_DURATION_MS);
    lastBeepMillis = millis();
  }
}

// ============================================================
//  BLE  — Nordic UART Service (NUS)
//  Service UUID : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//  TX Char UUID : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (notify)
//  RX Char UUID : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (write) ← optional
// ============================================================
#define BLE_DEVICE_NAME  "CareAlert-Rx"
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32→Phone
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Phone→ESP32

BLEServer*         bleServer  = nullptr;
BLECharacteristic* txChar     = nullptr;   // notify phone of SOS
BLECharacteristic* rxChar     = nullptr;   // receive ack from phone (optional)
bool               bleConnected = false;

// ---- BLE Server Callbacks ----
class BLEConnectCB : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override {
    bleConnected = true;
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(BLEServer* srv) override {
    bleConnected = false;
    Serial.println("[BLE] Client disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

// ---- Optional: handle "acknowledge" message from the dashboard ----
class RXCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    // ESP32 Arduino core 3.x returns Arduino String; older returns std::string.
    // Using getValue()+length()+c_str() works on both.
    String val = ch->getValue();
    if (val.length() == 0) return;
    Serial.print("[BLE RX] Received from phone: ");
    Serial.println(val);

    // Expected: {"ack":7}  — dashboard acknowledged patient 7
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, val.c_str());
    if (!err && doc.containsKey("ack")) {
      uint8_t id = doc["ack"];
      if (id >= 1 && id <= MAX_PATIENTS) {
        patients[id - 1].sosActive = false;
        Serial.printf("[BLE] Acknowledged SOS for patient #%d\n", id);
      }
    }
  }
};

// ---- Send SOS notification to the connected phone ----
void bleSendSOS(uint8_t patientID) {
  if (!bleConnected) {
    Serial.println("[BLE] No client connected — skipping notify");
    return;
  }

  // Build JSON:  {"id":7,"sos":true,"ts":123456}
  StaticJsonDocument<96> doc;
  doc["id"]  = patientID;
  doc["sos"] = true;
  doc["ts"]  = millis();

  char buf[96];
  size_t len = serializeJson(doc, buf);

  txChar->setValue((uint8_t*)buf, len);
  txChar->notify();

  Serial.printf("[BLE] Notified phone: %s\n", buf);
}

// ---- BLE setup ----
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BLEConnectCB());

  BLEService* service = bleServer->createService(NUS_SERVICE_UUID);

  // TX characteristic — ESP32 → phone  (notify)
  txChar = service->createCharacteristic(
    NUS_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  txChar->addDescriptor(new BLE2902());

  // RX characteristic — phone → ESP32  (write, for ACK messages)
  rxChar = service->createCharacteristic(
    NUS_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxChar->setCallbacks(new RXCallback());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);  // helps iOS connectivity
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as '" BLE_DEVICE_NAME "'");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CareAlert Receiver ===");

  // Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Startup beep sequence (confirms buzzer works)
  tone(BUZZER_PIN, 1000, 150); delay(200);
  tone(BUZZER_PIN, 1500, 150); delay(200);
  tone(BUZZER_PIN, 2000, 150); delay(200);
  Serial.println("[HW] Buzzer OK");

  // Clear patient state
  memset(patients, 0, sizeof(patients));

  // NRF24L01 init
  if (!radio.begin()) {
    Serial.println("[RF] NRF24L01 init FAILED — check wiring!");
    // Blink buzzer as error indicator
    for (int i = 0; i < 6; i++) {
      tone(BUZZER_PIN, 500, 100);
      delay(200);
    }
    while (1) delay(1000);
  }

  radio.setChannel(108);           // Channel 108 — above most 2.4 GHz WiFi
  radio.setPALevel(RF24_PA_HIGH);  // RF24_PA_LOW / MED / HIGH / MAX
  radio.setDataRate(RF24_250KBPS); // 250 KBPS → best range & reliability
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openReadingPipe(0, RF_ADDRESS);
  radio.startListening();

  Serial.println("[RF] NRF24L01 OK — listening on channel 108");

  // BLE init
  setupBLE();

  Serial.println("[SYS] Ready — waiting for alerts");
  Serial.println("==========================================");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  // ---- Check for incoming RF packet ----
  if (radio.available()) {
    radio.read(&inPkt, sizeof(inPkt));

    Serial.printf("[RF] Packet received — patientID=%d, emergency=%d\n",
                  inPkt.patientID, inPkt.emergency);

    if (inPkt.emergency && inPkt.patientID >= 1 && inPkt.patientID <= MAX_PATIENTS) {
      uint8_t idx = inPkt.patientID - 1;

      // Only trigger if not already active (avoids duplicate beeps)
      if (!patients[idx].sosActive) {
        patients[idx].sosActive   = true;
        patients[idx].alertMillis = millis();

        Serial.printf("[ALERT] SOS from Patient #%d\n", inPkt.patientID);

        // 1. Sound the buzzer
        startBuzzer();

        // 2. Notify the BLE dashboard
        bleSendSOS(inPkt.patientID);
      } else {
        // Repeat press — re-notify and re-buzz
        Serial.printf("[ALERT] Repeat SOS from Patient #%d\n", inPkt.patientID);
        patients[idx].alertMillis = millis();
        bleSendSOS(inPkt.patientID);
        if (!buzzerOn) startBuzzer();
      }
    }
  }

  // ---- Buzzer pulse management ----
  updateBuzzer();

  // ---- Periodic status print every 30 s ----
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 30000) {
    lastStatus = millis();
    int activeSOS = 0;
    for (int i = 0; i < MAX_PATIENTS; i++) {
      if (patients[i].sosActive) activeSOS++;
    }
    Serial.printf("[STATUS] BLE=%s | Active SOS=%d | Uptime=%lus\n",
                  bleConnected ? "connected" : "waiting",
                  activeSOS,
                  millis() / 1000);
  }
}