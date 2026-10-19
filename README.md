# PL8 Remote

Wireless remote control for a Revolectrix PowerLab 8 (v2 firmware, 3.xx).

- `app/index.html` is a web app that runs in Chrome or Edge and talks Bluetooth to the bridge.
- `firmware/PL8Bridge_FUIM3/` is the ESP32-S3 bridge that plugs into your **FUIM3** over USB. Use this one if you want to keep the FUIM3.
- `firmware/PL8Bridge/` is an alternative ESP32 / ESP32-C3 bridge that wires straight to the charger's serial port with no FUIM3.

Both bridges look the same to the app (Bluetooth name PL8-Bridge, Nordic UART service), so the app works with either one.

The app implements the PowerLab 8 serial protocol exactly as Revolectrix documents it in *Remote Control a Single PowerLab 8v2*, cross-checked against the decompiled Charge Control software (CHGM8Sv2.exe).

## Protocol summary

19200 baud, 8N1, single wire, open collector, 10 kΩ pull-up to 3.6 V inside the charger. Multi-byte values are MSB first. The line is one wire, so every reply arrives after an echo of the command, the same as with Charge Control.

| Send | Reply (after echo) | Purpose |
|---|---|---|
| `Ram` 0x00 | 147 data bytes + CRC16 (init 2342) | Status packet |
| `SelP` n | CRC16 of n (init 4372) | Select preset n (0–24), only when Mode = 0 |
| `Se` + pack + op | 0x05DC | Start. Pack `l`–`s` = 1–8 parallel packs. Op `C`/`D`/`M`/`Y` = charge/discharge/monitor/cycle; lowercase = balance leads only |
| `SelE` | 0x05DC | Enter: stop, acknowledge a safety screen (Mode 10), clear an error (Mode 99) |
| `Prst` | 7680 bytes + CRC16 (init 6372) | Read all preset memory (about 4 s) |

CRC16 is reflected CCITT (poly 0x8408) with the initial values above. `SelP 0` returns 0x56B4.

## FUIM3 bridge (ESP32-S3)

```
PowerLab 8 serial port ── FUIM3 ── USB cable ── ESP32-S3 native USB port ("USB", not "UART")
```

The FUIM3 is an FTDI USB-serial adapter, so the ESP32 has to be a USB **host**. Only the S2 and S3 can do that. The ESP32-C3 and the classic ESP32 can't.

**Power the FUIM3.** A USB host has to supply 5 V on the port. Dev boards usually don't do this by default:
- **ESP32-S3-DevKitC-1:** bridge the solder jumper marked **USB-OTG** (next to the USB port) so the board's 5 V rail feeds the USB connector. Then power the board through the **UART** USB port or the 5V pin, not through the port the FUIM3 is on.
- **Other S3 boards:** use a USB-OTG adapter with a power-injection input, or feed 5 V to the connector's VBUS pin. The FUIM3 needs well under 100 mA.

**Seeed XIAO ESP32-S3:** this board works. It has one USB-C port, which goes straight to the S3's native USB, so it's both the programming port and the FUIM3 port.
- **Power:** feed regulated 5 V into the **5V** pin and GND. On the XIAO that pin is tied to the USB-C VBUS, so the FUIM3 gets its 5 V from it. Confirm with a meter before plugging in the FUIM3.
- **Adapter:** connect the FUIM3 cable through a USB-C (male) to USB-A (female) OTG adapter.
- **Board settings:** **XIAO_ESP32S3**, USB Mode **Hardware CDC and JTAG**, USB CDC On Boot **Disabled**. This board defaults to Enabled, so change it.
- **LED:** set `PIN_LED = 21` and `LED_ACTIVE_LOW = true` for the on-board LED.
- **Re-flashing:** once this firmware is running, the USB port belongs to the FUIM3. To upload again, unplug everything, hold **BOOT**, plug the XIAO into the computer, then release.
- **Debug log:** comes out on pins D6 (TX) and D7 (RX) at 115200. You need a 3.3 V USB-serial adapter to read it. Without one, everything still works.
- **Antenna:** fit the included antenna to the U.FL connector, or Bluetooth range will be a few feet.

**Arduino IDE settings (DevKitC-1):** Board **ESP32S3 Dev Module**, USB Mode **Hardware CDC and JTAG**, USB CDC On Boot **Disabled**. Debug messages appear at 115200 on the UART port. They show the FUIM3's USB ID, the link speed it settled on, and each phone connect and disconnect.

**What the firmware does with the FUIM3:**
- It claims any FTDI device (vendor ID 0403) and sets 8N1 with no flow control.
- It turns DTR on and RTS off, as Charge Control does.
- It sets a 2 ms latency timer so replies come back quickly.
- It strips FTDI's 2 status bytes from each USB packet and passes everything else through unchanged, including the line echo.
- **Link speed:** Charge Control talks to a detected FUIM3 at 115200 baud, while the spec says 19200. The bridge sends a status request at each speed and keeps whichever gets a valid, checksummed reply. If the app is polling and valid replies stop for 4 seconds, it switches to the other speed. If you power-cycle the charger or unplug the FUIM3, it re-detects by itself.

## Wiring the direct bridge (ESP32-C3, no FUIM3)

```
ESP32 GPIO21 (TX) ──|<──┬── PowerLab serial DATA     diode band toward the ESP32 (BAT43 / BAT85 / 1N5817)
ESP32 GPIO20 (RX) ─[1k]─┘
ESP32 GND ────────────────── PowerLab serial GND
```

1. Find the DATA and GND pins on the charger's serial port with a meter. DATA idles at about 3.6 V.
2. Power the ESP32 from its own 5 V supply. Connect only DATA and GND to the charger.
3. For a single-pin hookup with no diode, set `#define WIRING WIRING_ONE_PIN` and use GPIO21 → 330 Ω → DATA.
4. On a classic ESP32 (no GPIO20), change `PIN_TX` / `PIN_RX` to free pins such as 17 / 16.

Don't connect the FUIM3 and the bridge at the same time; they would share the line.

## Flashing

Use the Arduino IDE with the esp32 core 3.x installed. No extra libraries are needed for either bridge.
- **FUIM3 bridge:** use the S3 settings above. It compiles to about 620 KB.
- **Direct bridge:** pick **ESP32C3 Dev Module** and set **USB CDC On Boot: Enabled**. It compiles to about 640 KB.

The bridge advertises as **PL8-Bridge**. Its LED blinks while it waits and stays on when a phone is connected.

## HC-05 (Bluetooth serial) and USB

The **Serial / HC-05** button opens a serial port through Chrome's Web Serial API. It works with:
- **A paired HC-05,** or any Bluetooth Classic SPP adapter. This works in Chrome or Edge 117+ on Windows, macOS, Linux and ChromeOS, and in Chrome 137+ on Android.
- **A FUIM3 plugged straight into a computer** by USB, desktop only. The app turns DTR on like Charge Control and opens the port at 19200, the PowerLab's specified speed. If your FUIM3 doesn't answer at 19200, set Session → Serial speed to **Auto**, which also tries 115200. Charge Control's code switches a detected FUIM3 to 115200.

### HC-05 setup

1. **Data-mode speed:** set it to match the line it's wired to. Use `AT+UART=19200,0,0` when it's on the PowerLab's serial line, per the Revolectrix spec, or `AT+UART=115200,0,0` if your design puts it where Charge Control would talk to the FUIM3 at 115200. The HC-05 ships at 9600, so leaving it unchanged won't work.
2. **Single-wire line:** if it's wired directly to the PowerLab's data line, the HC-05's TX drives the line both ways, so use the same diode + 1 kΩ hookup as the direct ESP32 bridge. Its pins are 3.3 V logic; the charger's line idles at 3.6 V.
3. **Pairing:** pair it in your phone's or computer's Bluetooth settings first. The PIN is usually 1234 or 0000. Then press **Serial / HC-05** and pick it from the list.
4. **Echo:** it doesn't matter whether your adapter passes the line echo through. The app accepts replies either way.

The app polls once a second, and Bluetooth serial adds some delay but no lost data. If replies time out, open **Serial traffic** in the Session panel. "bad reply" entries usually mean the HC-05's speed doesn't match the line.

## Running the app

Web Bluetooth needs an https page. Two easy options:

- **GitHub Pages:** put `app/index.html` in a repo, enable Pages, then open the URL in Chrome on your phone. "Add to Home screen" makes it behave like an app.
- **Local:** `npx serve app` and open `http://localhost:3000` on the same computer.

It works in Chrome or Edge on Android, Windows, macOS, Linux and ChromeOS. On iPhone, use the Bluefy browser.

Press **Connect bridge** and pick PL8-Bridge. Presets load automatically. The app polls status once a second and shows the cells, IR, a voltage/current chart, and alerts when a charge finishes, an error occurs, or a safety screen is waiting. Without a bridge it runs a built-in simulated PowerLab.

## Notes

- Safety screens are never acknowledged automatically. You confirm them from the app.
- If Bluetooth drops, the charger keeps running on its own and the app keeps trying to reconnect.
- Starting a charge is refused if the selected preset is empty, hidden or not validated. The charger does the same check.
- IR shows only after the charger has measured it, about 3 minutes into a charge from below 75%.
