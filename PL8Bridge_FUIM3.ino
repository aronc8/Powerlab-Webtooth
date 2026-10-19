/*
  PL8Bridge_FUIM3 — Bluetooth LE <-> FUIM3 (USB) bridge for the PowerLab 8
  ------------------------------------------------------------------------
  Target : ESP32-S3 (needs USB-OTG host; the ESP32-C3 cannot do this)
  Core   : arduino-esp32 3.x
  Board  : "ESP32S3 Dev Module"
           USB Mode: "Hardware CDC and JTAG"   USB CDC On Boot: "Disabled"
           (Serial debug output then comes out of the board's UART/COM port.)

  Hookup
    PowerLab 8 serial port ── FUIM3 ── USB cable ── ESP32-S3 native USB port
    The FUIM3 is powered from USB, so the S3's USB port must supply 5 V.
    See README for how to do that on a DevKitC-1.

  What it does
    * Advertises as "PL8-Bridge" with the Nordic UART Service, exactly like the
      direct-wired bridge, so the PL8 Remote app works unchanged.
    * Acts as a USB host for the FUIM3's FTDI chip: claims it, turns DTR on
      (Charge Control does this too) and passes bytes both ways untouched.
    * Works out the link speed by itself. Charge Control runs the FUIM3 at
      115200 when it detects one, the spec says 19200; the bridge watches for
      valid "Ram" status packets and switches between the two until it sees
      them, so it works either way.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if !defined(CONFIG_NIMBLE_ENABLED) && !defined(CONFIG_BT_NIMBLE_ENABLED)
#include <BLE2902.h>
#endif
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"

// ---------------- configuration ----------------
static const char *DEVICE_NAME = "PL8-Bridge";
// XIAO ESP32-S3: PIN_LED 21, LED_ACTIVE_LOW true.  DevKitC-1: leave -1 (its LED is an RGB WS2812).
static const int   PIN_LED = -1;            // set to an LED pin if your board has a plain one
static const bool  LED_ACTIVE_LOW = false;
static const bool  DEBUG = true;            // log to Serial (UART port)

#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static const uint16_t FTDI_VID = 0x0403;
// FTDI SIO vendor requests
enum { FTDI_RESET = 0, FTDI_MODEM_CTRL = 1, FTDI_FLOW_CTRL = 2, FTDI_SET_BAUD = 3, FTDI_SET_DATA = 4, FTDI_SET_LATENCY = 9 };
// Divisors for the 3 MHz base clock (FT232R / FT230X / FT231X)
struct BaudCfg { uint32_t baud; uint16_t value; uint16_t index; };
static const BaudCfg BAUDS[2] = { { 19200, 0x809C, 0 }, { 115200, 0x001A, 0 } };

#define DBG(...) do { if (DEBUG) Serial.printf(__VA_ARGS__); } while (0)

// ---------------- byte rings ----------------
template <size_t N> struct Ring {
  uint8_t d[N]; volatile size_t h = 0, t = 0; portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
  void push(const uint8_t *p, size_t n) { portENTER_CRITICAL(&m); for (size_t i = 0; i < n; i++) { size_t nx = (h + 1) % N; if (nx == t) break; d[h] = p[i]; h = nx; } portEXIT_CRITICAL(&m); }
  size_t pop(uint8_t *p, size_t max) { size_t n = 0; portENTER_CRITICAL(&m); while (n < max && t != h) { p[n++] = d[t]; t = (t + 1) % N; } portEXIT_CRITICAL(&m); return n; }
  bool empty() { return h == t; }
  void clear() { portENTER_CRITICAL(&m); h = 0; t = 0; portEXIT_CRITICAL(&m); }
};
static Ring<1024> toCharger;   // phone -> FUIM3
static Ring<4096> toPhone;     // FUIM3 -> phone

// ---------------- shared state ----------------
static BLEServer *server = nullptr;
static BLECharacteristic *txChar = nullptr;
static volatile bool bleConnected = false, needAdvertise = false;
static volatile uint32_t lastFromChargerMs = 0, lastPhoneWriteMs = 0;
static volatile bool fuimReady = false;
static volatile int  baudIdx = 0;
static volatile uint32_t lastValidFrameMs = 0;
static volatile bool requestBaudSwitch = false;
static volatile bool requestProbe = false;

// ---------------- PowerLab frame detector (for baud auto-detect) ----------------
static uint16_t crc16(const uint8_t *b, size_t n, uint16_t c) {
  for (size_t i = 0; i < n; i++) { uint8_t d = b[i]; for (int k = 0; k < 8; k++) { if ((d ^ c) & 1) c = (c >> 1) ^ 0x8408; else c >>= 1; d >>= 1; } }
  return c;
}
static uint8_t win[512]; static size_t winLen = 0;
static void detectFrames(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (winLen == sizeof(win)) { memmove(win, win + 256, 256); winLen = 256; }
    win[winLen++] = p[i];
  }
  // A status reply is 147 data bytes + CRC (init 2342), with or without the "Ram\0" echo in front.
  if (winLen < 149) return;
  const uint8_t *f = win + winLen - 149;
  if (((f[147] << 8) | f[148]) == crc16(f, 147, 2342)) { lastValidFrameMs = millis(); winLen = 0; }
}

// =====================================================================
//  USB host + minimal FTDI driver (runs entirely in usbTask)
// =====================================================================
static usb_host_client_handle_t client = nullptr;
static usb_device_handle_t dev = nullptr;
static volatile uint8_t newDevAddr = 0;
static volatile bool devGone = false;
static uint8_t ifNum = 0, epIn = 0, epOut = 0; static uint16_t mpsIn = 64, mpsOut = 64;
static usb_transfer_t *ctrlX = nullptr, *inX = nullptr, *outX = nullptr;
static volatile bool ctrlDone = false, outBusy = false;
static volatile bool probing = false;
static uint8_t probeBuf[400]; static volatile size_t probeLen = 0;

static void clientCb(const usb_host_client_event_msg_t *msg, void *) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) newDevAddr = msg->new_dev.address;
  else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) devGone = true;
}
static void ctrlCb(usb_transfer_t *) { ctrlDone = true; }
static void outCb(usb_transfer_t *) { outBusy = false; }
static void inCb(usb_transfer_t *x) {
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
    // FTDI prefixes every max-packet chunk with 2 modem-status bytes.
    for (int off = 0; off < x->actual_num_bytes; off += mpsIn) {
      int len = min((int)mpsIn, x->actual_num_bytes - off);
      if (len > 2) {
        const uint8_t *p = x->data_buffer + off + 2; size_t n = len - 2;
        if (probing) { size_t k = min(n, sizeof(probeBuf) - probeLen); memcpy(probeBuf + probeLen, p, k); probeLen += k; }
        else { toPhone.push(p, n); lastFromChargerMs = millis(); }
        detectFrames(p, n);
      }
    }
  }
  if (dev && x->status != USB_TRANSFER_STATUS_NO_DEVICE && x->status != USB_TRANSFER_STATUS_CANCELED) {
    x->num_bytes = mpsIn;
    usb_host_transfer_submit(x);
  }
}

static bool ftdiCtrl(uint8_t req, uint16_t value, uint16_t index) {
  usb_setup_packet_t *s = (usb_setup_packet_t *)ctrlX->data_buffer;
  s->bmRequestType = 0x40; s->bRequest = req; s->wValue = value; s->wIndex = index; s->wLength = 0;
  ctrlX->num_bytes = sizeof(usb_setup_packet_t);
  ctrlX->device_handle = dev; ctrlX->bEndpointAddress = 0; ctrlX->callback = ctrlCb; ctrlX->context = nullptr; ctrlX->timeout_ms = 1000;
  ctrlDone = false;
  if (usb_host_transfer_submit_control(client, ctrlX) != ESP_OK) return false;
  uint32_t t0 = millis();
  while (!ctrlDone && millis() - t0 < 1500) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
  return ctrlDone && ctrlX->status == USB_TRANSFER_STATUS_COMPLETED;
}

static bool ftdiSetBaud(int idx) {
  // wIndex carries the port number in its low byte on multi-port chips (FT2232 etc).
  bool ok = ftdiCtrl(FTDI_SET_BAUD, BAUDS[idx].value, BAUDS[idx].index | ifNum);
  if (ok) { baudIdx = idx; DBG("FUIM3 baud %lu\n", (unsigned long)BAUDS[idx].baud); }
  return ok;
}

static void closeDevice() {
  fuimReady = false;
  if (dev) {
    usb_host_endpoint_halt(dev, epIn); usb_host_endpoint_flush(dev, epIn);
    usb_host_interface_release(client, dev, ifNum);
    usb_host_device_close(client, dev);
    dev = nullptr;
  }
  if (inX) { usb_host_transfer_free(inX); inX = nullptr; }
  if (outX) { usb_host_transfer_free(outX); outX = nullptr; }
  outBusy = false;
}

static void openDevice(uint8_t addr) {
  if (usb_host_device_open(client, addr, &dev) != ESP_OK) { dev = nullptr; return; }
  const usb_device_desc_t *dd; usb_host_get_device_descriptor(dev, &dd);
  DBG("USB device %04x:%04x\n", dd->idVendor, dd->idProduct);
  if (dd->idVendor != FTDI_VID) { DBG("Not an FTDI device (FUIM3 is FTDI) - ignoring\n"); usb_host_device_close(client, dev); dev = nullptr; return; }

  const usb_config_desc_t *cd; usb_host_get_active_config_descriptor(dev, &cd);
  int off = 0;
  const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cd, 0, 0, &off);
  if (!intf) { closeDevice(); return; }
  ifNum = intf->bInterfaceNumber; epIn = epOut = 0;
  for (int i = 0; i < intf->bNumEndpoints; i++) {
    int eoff = off;
    const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cd->wTotalLength, &eoff);
    if (!ep || (ep->bmAttributes & 0x03) != 0x02) continue;   // bulk only
    if (ep->bEndpointAddress & 0x80) { epIn = ep->bEndpointAddress; mpsIn = ep->wMaxPacketSize; }
    else { epOut = ep->bEndpointAddress; mpsOut = ep->wMaxPacketSize; }
  }
  if (!epIn || !epOut || usb_host_interface_claim(client, dev, ifNum, 0) != ESP_OK) { DBG("FTDI endpoints/claim failed\n"); closeDevice(); return; }

  // FTDI setup: reset, 8N1, no flow control, DTR on / RTS off (as Charge Control), 2 ms latency
  bool ok = ftdiCtrl(FTDI_RESET, 0, ifNum)
         && ftdiCtrl(FTDI_SET_DATA, 0x0008, ifNum)
         && ftdiCtrl(FTDI_FLOW_CTRL, 0, ifNum)
         && ftdiCtrl(FTDI_MODEM_CTRL, 0x0101, ifNum)
         && ftdiCtrl(FTDI_MODEM_CTRL, 0x0200, ifNum)
         && ftdiCtrl(FTDI_SET_LATENCY, 2, ifNum)
         && ftdiSetBaud(baudIdx);
  if (!ok) { DBG("FTDI setup failed\n"); closeDevice(); return; }

  usb_host_transfer_alloc(mpsIn, 0, &inX);
  inX->device_handle = dev; inX->bEndpointAddress = epIn; inX->callback = inCb; inX->num_bytes = mpsIn; inX->timeout_ms = 0;
  usb_host_transfer_alloc(mpsOut, 0, &outX);
  outX->device_handle = dev; outX->bEndpointAddress = epOut; outX->callback = outCb; outX->timeout_ms = 1000;
  usb_host_transfer_submit(inX);
  toPhone.clear(); toCharger.clear();
  lastValidFrameMs = 0;
  fuimReady = true;
  requestProbe = true;
  DBG("FUIM3 ready\n");
}

static void sendToCharger(const uint8_t *p, size_t n) {
  if (!outX || n == 0) return;
  memcpy(outX->data_buffer, p, n);
  outX->num_bytes = n; outBusy = true;
  if (usb_host_transfer_submit(outX) != ESP_OK) outBusy = false;
}

// Ask the charger for a status packet and see if a valid reply comes back at this baud.
static bool probeAt(int idx) {
  if (!ftdiSetBaud(idx)) return false;
  probeLen = 0; probing = true;
  const uint8_t ram[4] = { 'R', 'a', 'm', 0 };
  while (outBusy) usb_host_client_handle_events(client, pdMS_TO_TICKS(5));
  sendToCharger(ram, 4);
  uint32_t t0 = millis(), before = lastValidFrameMs;
  while (millis() - t0 < 450 && lastValidFrameMs == before) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
  probing = false;
  bool ok = lastValidFrameMs != before;
  DBG("Probe at %lu: %s\n", (unsigned long)BAUDS[idx].baud, ok ? "charger answered" : "no valid reply");
  return ok;
}

static void usbLibTask(void *) {
  for (;;) { uint32_t f; usb_host_lib_handle_events(portMAX_DELAY, &f); }
}

static void usbClientTask(void *) {
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false; cc.max_num_event_msg = 5;
  cc.async.client_event_callback = clientCb; cc.async.callback_arg = nullptr;
  usb_host_client_register(&cc, &client);
  usb_host_transfer_alloc(64, 0, &ctrlX);
  uint32_t lastProbe = 0;
  for (;;) {
    usb_host_client_handle_events(client, pdMS_TO_TICKS(2));
    if (newDevAddr) { uint8_t a = newDevAddr; newDevAddr = 0; if (!dev) openDevice(a); }
    if (devGone) { devGone = false; DBG("USB device removed\n"); closeDevice(); }
    if (!fuimReady) continue;

    // phone -> charger
    if (!outBusy && !toCharger.empty()) {
      uint8_t buf[64]; size_t n = toCharger.pop(buf, min((size_t)mpsOut, sizeof(buf)));
      sendToCharger(buf, n);
    }

    uint32_t now = millis();
    bool recentValid = lastValidFrameMs && now - lastValidFrameMs < 5000;
    bool phoneActive = now - lastPhoneWriteMs < 2500;
    // While the app polls, a speed mismatch shows up as no valid replies: flip and try the other speed.
    if (phoneActive && !recentValid && now - lastProbe > 4000) { lastProbe = now; ftdiSetBaud(baudIdx ^ 1); }
    // When nothing is talking, find the speed actively.
    if (!phoneActive && (requestProbe || (!recentValid && now - lastProbe > 3000))) {
      requestProbe = false; lastProbe = now;
      if (!probeAt(baudIdx)) probeAt(baudIdx ^ 1);
    }
  }
}

// =====================================================================
//  BLE (Nordic UART Service)
// =====================================================================
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; DBG("Phone connected\n"); }
  void onDisconnect(BLEServer *) override { bleConnected = false; needAdvertise = true; DBG("Phone disconnected\n"); }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { toCharger.push(c->getData(), c->getLength()); lastPhoneWriteMs = millis(); }
};

static void led(bool on) { if (PIN_LED >= 0) digitalWrite(PIN_LED, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW); }

void setup() {
  Serial.begin(115200);
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);

  usb_host_config_t hc = {};
  hc.skip_phy_setup = false; hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
  if (usb_host_install(&hc) != ESP_OK) DBG("usb_host_install failed\n");
  xTaskCreatePinnedToCore(usbLibTask, "usblib", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(usbClientTask, "fuim3", 6144, nullptr, 3, nullptr, 0);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
#if !defined(CONFIG_NIMBLE_ENABLED) && !defined(CONFIG_BT_NIMBLE_ENABLED)
  txChar->addDescriptor(new BLE2902());
#endif
  BLECharacteristic *rx = svc->createCharacteristic(NUS_RX_CHAR, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  DBG("PL8Bridge (FUIM3) ready. Plug the FUIM3 into the S3's USB port.\n");
}

void loop() {
  // charger -> phone: send when a burst ends or a full BLE packet is ready
  static uint8_t buf[244]; static size_t len = 0;
  uint16_t mtu = server ? server->getPeerMTU(server->getConnId()) : 23;
  size_t chunk = min((size_t)(mtu > 23 ? mtu - 3 : 20), sizeof(buf));
  len += toPhone.pop(buf + len, chunk - len);
  if (len && (len >= chunk || millis() - lastFromChargerMs > 4)) {
    if (bleConnected) { txChar->setValue(buf, len); txChar->notify(); }
    len = 0;
    delay(2);
  } else {
    delay(1);
  }

  if (needAdvertise) { needAdvertise = false; delay(200); BLEDevice::startAdvertising(); }

  // LED: off = no FUIM3, slow blink = FUIM3 found / waiting for phone, solid = phone connected
  led(!fuimReady ? false : bleConnected ? true : ((millis() / 600) % 2));
}
