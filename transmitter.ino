// ============================================================
//  CareAlert — ESP32 Transmitter Firmware
//  Hardware : ESP32 DevKit + NRF24L01 + Push Button
//
//  How it works:
//    - Elderly/disabled patient wears or holds this device
//    - Pressing the button sends an SOS packet over NRF24L01
//    - Receiver (CareAlert_Receiver_BLE) catches it and
//      alerts the caregiver via buzzer + BLE dashboard
//
//  Libraries needed (install via Arduino Library Manager):
//    - RF24  by TMRh20
//
//  Board: "ESP32 Dev Module"  (Tools > Board)
//
//  !! IMPORTANT !!
//  Change PATIENT_ID below to a unique number (1–50)
//  for each transmitter device you build.
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================================================
//  PATIENT ID  — set a unique number (1–50) per device
// ============================================================
#define PATIENT_ID   1      // <-- change this for each wristband

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define NRF_CE_PIN    4     // NRF24L01 CE  → ESP32 GPIO 4
#define NRF_CSN_PIN   5     // NRF24L01 CSN → ESP32 GPIO 5
#define BUTTON_PIN    14    // Push button  → ESP32 GPIO 14 (other leg to GND)

// NRF24L01 SPI pins (fixed on ESP32, do not change):
//   MOSI → GPIO 23
//   MISO → GPIO 19
//   SCK  → GPIO 18

// ============================================================
//  NRF24L01 CONFIG  — must match receiver exactly
// ============================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint8_t RF_ADDRESS[] = "CARE1";   // 5-byte address — same on receiver

// ============================================================
//  DATA PACKET  — must match receiver struct exactly
// ============================================================
struct DataPacket {
  uint8_t  patientID;    // which patient pressed the button
  bool     emergency;    // always true when sent
  uint32_t timestamp;    // millis() at time of press
};

DataPacket pkt;

// ============================================================
//  DEBOUNCE & RETRY CONFIG
// ============================================================
const unsigned long DEBOUNCE_MS     = 500;   // ignore re-press within 500 ms
const unsigned long SEND_RETRY_MS   = 20;    // wait between retries
const uint8_t       MAX_RETRIES     = 5;     // try up to 5 times per press

// ============================================================
//  STATE
// ============================================================
unsigned long lastPressTime = 0;
bool          lastBtnState  = HIGH;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CareAlert Transmitter ===");
  Serial.printf("Patient ID : %d\n", PATIENT_ID);

  // Button pin — internal pull-up, press pulls LOW
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // NRF24L01 init
  if (!radio.begin()) {
    Serial.println("[RF] NRF24L01 init FAILED — check wiring!");
    // Flash serial and hang — no buzzer on transmitter side
    while (1) {
      Serial.println("[RF] Halted. Fix wiring and reset.");
      delay(2000);
    }
  }

  radio.setChannel(108);           // same channel as receiver (above WiFi range)
  radio.setPALevel(RF24_PA_HIGH);  // HIGH power for range; use MAX if needed
  radio.setDataRate(RF24_250KBPS); // 250 KBPS = best reliability and range
  radio.setAutoAck(true);          // receiver ACKs every packet
  radio.setRetries(5, 15);         // 5 retries, 15*250us = 3.75ms between each
  radio.openWritingPipe(RF_ADDRESS);
  radio.stopListening();           // transmitter mode

  // Pre-fill fixed fields
  pkt.patientID = PATIENT_ID;
  pkt.emergency = true;            // always true — this is an SOS device

  Serial.println("[RF] NRF24L01 OK");
  Serial.println("[SYS] Ready — waiting for button press");
  Serial.println("======================================");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  bool currentBtn = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // Detect a fresh button press (HIGH → LOW transition) with debounce
  if (currentBtn == LOW && lastBtnState == HIGH && (now - lastPressTime) > DEBOUNCE_MS) {
    lastPressTime = now;

    pkt.timestamp = now;

    Serial.println("----------------------------------");
    Serial.printf("[BTN] Button pressed — Patient #%d sending SOS\n", PATIENT_ID);

    // Try to send, retry up to MAX_RETRIES times
    bool sent = false;
    for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
      if (radio.write(&pkt, sizeof(pkt))) {
        sent = true;
        Serial.printf("[RF] Sent successfully on attempt %d\n", attempt);
        break;
      }
      Serial.printf("[RF] Attempt %d failed — retrying...\n", attempt);
      delay(SEND_RETRY_MS);
    }

    if (sent) {
      Serial.println("[SYS] SOS delivered to receiver");
    } else {
      Serial.println("[SYS] All attempts failed — receiver out of range?");
    }

    Serial.println("----------------------------------");
  }

  lastBtnState = currentBtn;

  // Small yield — keeps watchdog happy
  delay(10);
}