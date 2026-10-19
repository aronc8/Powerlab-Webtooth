/*
  PL8Bridge — transparent Bluetooth LE <-> PowerLab 8 serial bridge
  ------------------------------------------------------------------
  Target : ESP32-C3 (any ESP32 with BLE works; change the pins below)
  Core   : arduino-esp32 3.x  (Board: "ESP32C3 Dev Module", USB CDC On Boot: Enabled)

  What it does
    * Advertises as "PL8-Bridge" with the Nordic UART Service (NUS).
    * Every byte the phone writes to the RX characteristic goes out on the
      PowerLab's serial line; every byte heard on the line is notified back.
    * It does NOT interpret the protocol. The web app speaks the PowerLab
      protocol ("Ram", "SelP", "SelC", "SelE"...) exactly like Charge Control.

  PowerLab 8 serial port (from Revolectrix "Remote Control a Single PowerLab 8v2")
    * Single-wire, 19200 baud, 8N1, non-inverted, idle HIGH.
    * The charger has a 10k pull-up to 3.6 V on the data line.
    * Devices on the line must be OPEN-COLLECTOR (or have a diode on TX) so
      they never drive the line high.
    * Because it is one wire, we hear our own transmissions. That echo is
      expected — Charge Control relies on it too (every reply starts with the
      echoed command), so it is passed through untouched.

  Wiring (default, WIRING_DIODE):

      ESP32 TX (GPIO21) ──|<──┬── PowerLab DATA  (diode: cathode/band toward ESP32)
                              │
      ESP32 RX (GPIO20) ─[1k]─┘
      ESP32 GND ───────────────── PowerLab serial GND

    Use a small Schottky (BAT43/BAT85/1N5817). When TX is low it pulls the line
    low; when TX is high the diode blocks and the charger's pull-up wins.
    The 1k on RX limits current because 3.6 V sits right at the ESP32-C3's
    absolute-max input.

  Optional WIRING_ONE_PIN: a single GPIO in open-drain mode does both jobs —
    ESP32 GPIO21 ─[330R]─ PowerLab DATA. No diode needed.

  Power the ESP32 from its own 5 V source (USB or a buck converter). Only GND
  and DATA connect to the charger. Check the pinout of your charger's serial
  port with a meter first: DATA should idle at ~3.6 V relative to GND.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if !defined(CONFIG_NIMBLE_ENABLED) && !defined(CONFIG_BT_NIMBLE_ENABLED)
#include <BLE2902.h>   // Bluedroid needs the CCCD added by hand; NimBLE adds it itself
#endif

// ---------------- configuration ----------------
#define WIRING_DIODE      1     // two pins + diode (recommended)
#define WIRING_ONE_PIN    2     // one open-drain pin
#define WIRING            WIRING_DIODE

static const int  PIN_TX      = 21;
static const int  PIN_RX      = 20;    // ignored in WIRING_ONE_PIN mode
static const int  PIN_LED     = 8;     // on-board LED on most C3 boards (active low); -1 to disable
static const bool LED_ACTIVE_LOW = true;
static const uint32_t PL_BAUD = 19200;
static const char *DEVICE_NAME = "PL8-Bridge";

// Nordic UART Service UUIDs (same ones the Revolectrix Bump uses)
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone -> bridge (write)
#define NUS_TX_CHAR "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // bridge -> phone (notify)

// Spec: "a pause of more than 3 bytes should precede any network communication".
// 3 byte-times at 19200 ≈ 1.6 ms; use 3 ms.
static const uint32_t LINE_IDLE_BEFORE_TX_US = 3000;
// Flush buffered serial bytes to BLE after this much silence.
static const uint32_t RX_FLUSH_GAP_US = 4000;

#if WIRING == WIRING_ONE_PIN
#include "driver/gpio.h"
#include "soc/uart_periph.h"
#include "esp_rom_gpio.h"
#endif

HardwareSerial PL(1);

static BLEServer *server = nullptr;
static BLECharacteristic *txChar = nullptr;
static volatile bool connected = false;
static volatile bool needAdvertise = false;

// phone -> charger queue (filled from the BLE callback, drained in loop)
static const size_t QSIZE = 1024;
static uint8_t q[QSIZE];
static volatile size_t qHead = 0, qTail = 0;
static portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

static void qPush(const uint8_t *d, size_t n) {
  portENTER_CRITICAL(&qMux);
  for (size_t i = 0; i < n; i++) {
    size_t next = (qHead + 1) % QSIZE;
    if (next == qTail) break;               // full: drop
    q[qHead] = d[i];
    qHead = next;
  }
  portEXIT_CRITICAL(&qMux);
}
static int qPop() {
  int v = -1;
  portENTER_CRITICAL(&qMux);
  if (qTail != qHead) { v = q[qTail]; qTail = (qTail + 1) % QSIZE; }
  portEXIT_CRITICAL(&qMux);
  return v;
}

static void led(bool on) {
  if (PIN_LED < 0) return;
  digitalWrite(PIN_LED, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *) override { connected = false; needAdvertise = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    qPush(c->getData(), c->getLength());
  }
};

static void setupSerial() {
#if WIRING == WIRING_ONE_PIN
  PL.begin(PL_BAUD, SERIAL_8N1, PIN_TX, PIN_TX);
  // Route UART1 TX and RX through the same pad, configured open-drain, so we
  // can only ever pull the line low (the charger's pull-up drives it high).
  gpio_reset_pin((gpio_num_t)PIN_TX);
  gpio_set_pull_mode((gpio_num_t)PIN_TX, GPIO_FLOATING);
  gpio_set_direction((gpio_num_t)PIN_TX, GPIO_MODE_INPUT_OUTPUT_OD);
  esp_rom_gpio_connect_out_signal(PIN_TX, UART_PERIPH_SIGNAL(1, SOC_UART_TX_PIN_IDX), false, false);
  esp_rom_gpio_connect_in_signal(PIN_TX, UART_PERIPH_SIGNAL(1, SOC_UART_RX_PIN_IDX), false);
#else
  PL.begin(PL_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
#endif
  PL.setRxBufferSize(2048);
}

void setup() {
  Serial.begin(115200);
  if (PIN_LED >= 0) { pinMode(PIN_LED, OUTPUT); led(false); }
  setupSerial();

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService *svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
#if !defined(CONFIG_NIMBLE_ENABLED) && !defined(CONFIG_BT_NIMBLE_ENABLED)
  txChar->addDescriptor(new BLE2902());
#endif
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_CHAR, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("PL8Bridge ready, advertising as PL8-Bridge");
}

static uint8_t rxBuf[512];
static size_t rxLen = 0;
static uint32_t lastLineActivityUs = 0;

static size_t notifyChunk() {
  uint16_t mtu = server ? server->getPeerMTU(server->getConnId()) : 23;
  if (mtu < 23) mtu = 23;
  size_t n = mtu - 3;
  return n > sizeof(rxBuf) ? sizeof(rxBuf) : n;
}

static void flushToBle(size_t maxChunk) {
  size_t off = 0;
  while (off < rxLen) {
    size_t n = min(maxChunk, rxLen - off);
    if (connected) { txChar->setValue(rxBuf + off, n); txChar->notify(); }
    off += n;
    delay(2);  // let the BLE stack breathe between packets
  }
  rxLen = 0;
}

void loop() {
  uint32_t now = micros();

  // charger -> phone
  while (PL.available()) {
    int b = PL.read();
    if (b < 0) break;
    lastLineActivityUs = micros();
    rxBuf[rxLen++] = (uint8_t)b;
    if (rxLen >= notifyChunk()) flushToBle(notifyChunk());
  }
  if (rxLen && (micros() - lastLineActivityUs) > RX_FLUSH_GAP_US) flushToBle(notifyChunk());

  // phone -> charger (only after the line has been quiet for a moment)
  if (qTail != qHead && (micros() - lastLineActivityUs) > LINE_IDLE_BEFORE_TX_US) {
    uint8_t out[64];
    size_t n = 0;
    int b;
    while (n < sizeof(out) && (b = qPop()) >= 0) out[n++] = (uint8_t)b;
    PL.write(out, n);
    PL.flush();
    lastLineActivityUs = micros();
  }

  if (needAdvertise) {
    needAdvertise = false;
    delay(200);
    BLEDevice::startAdvertising();
  }

  // LED: slow blink while advertising, solid when a phone is connected
  led(connected ? true : ((millis() / 500) % 2));
  (void)now;
}
