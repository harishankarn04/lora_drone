# Project context (read this first)

Handover for anyone, human or AI assistant, picking up this project in a fresh session.
Last updated: 2026-10-23.

## What this is
A **LoRa beacon for drone homing**. A ground beacon transmits on 866 MHz. The drone carries a small receiver, detects the beacon, estimates distance and trend from signal strength (RSSI), and flies towards it. Close-range two-way talk is optional.

- **Long-term target:** a Pixhawk 2.4.8 with an ESP32 companion (I2C/SPI to the Pixhawk). This is not started.
- **Current phase:** characterise the radio link with E220 modules: range indoors and outdoors with line of sight, packet integrity, RSSI-to-distance, behaviour during signal loss and reacquisition.
- **Thesis:** the owner is writing a thesis on this work. `THESIS_NOTES.md` is the running engineering log of every decision, measurement, bug and source. **Keep it updated** after every change or finding, tagging items [measured] / [simulated] / [sourced] / [assumed].

## Hardware
| Item | Details |
|---|---|
| Radio ×2 | Ebyte **E220-900T22D** (LLCC68), 22 dBm, UART, omni antennas |
| **White ESP32** = tester (laptop) | ESP32-D0WD-V3, MAC `1c:c3:ab:b3:c4:68`, CP2102 USB |
| **Black ESP32** = beacon | ESP32-D0WDQ6 rev 1.1, MAC `fc:e8:c0:e1:bb:00`, CP2102 USB |
| Arduino UNO R4 Minima | Alternative beacon (5 V; uses open-drain outputs to drive the 3.3 V E220 with jumpers only) |

- Both CP2102 bridges report serial "0001", so macOS port names (`/dev/cu.usbserial-0001` / `-5`) **swap with plug order**. Identify boards by MAC (`esptool chip-id`), and **never** run esptool while another program holds the port: that leaves the ESP32 stuck in its bootloader.
- **ESP32 ↔ E220 wiring (both ESP32s):** E220 TXD → GPIO26, RXD ← GPIO27, AUX → GPIO25, M0 ← GPIO32, M1 ← GPIO33, VCC → 5V, GND → GND. The UNO wiring is in `USAGE.md`.

## Repository layout
| Path | What |
|---|---|
| `firmware/E220_Tester_ESP32/` | ESP32 sketch: tester **or** beacon via `#define ROLE` |
| `firmware/E220_Tester_ESP32/beacon_link.h` | Hardware-independent link logic: packet, framer, ping scheduler (RFC 6298 RTO, duty-cycle spacing), Kalman link tracker (TRACKING / COASTING / LOST). Shared with the simulator; meant to be reused on the Pixhawk companion. |
| `firmware/E220_Beacon_UNO_R4/` | UNO R4 beacon (responder or one-way via `BEACON_MODE`), plus a `mute <ms>` command for dropout tests |
| `tools/range_logger.py` | Laptop logger: live view and CSV in `tools/logs/` |
| `tools/dropout_test.py` | Hardware dropout test (mutes the UNO beacon over USB; times COAST/LOST/reacquire) |
| `tools/sim/` | Host simulation that compiles `beacon_link.h` (`./run_sim.sh`) |
| `tools/logs/` | Raw field logs. Oct 16 logs use the old 32-byte firmware and a 15-column CSV. |
| `USAGE.md` | Wiring tables, setup, field procedure, CSV format |
| `THESIS_NOTES.md` | Full engineering log with reasoning and references |

Library: **"EByte LoRa E220 library" v1.0.8** by Renzo Mischianti (Arduino Library Manager). It is not vendored, and the owner wants it installed through the IDE.

## Key decisions (details and reasons in THESIS_NOTES §5)
- **Packet:** 10 bytes (magic, net id, type, seq16, rssiUp, beaconRx, CRC16), length-framed. The E220 appends 1 RSSI byte to every received packet.
- **Radio settings:** CH 16 = **866.125 MHz** (Indian 865–867 band), 2.4 kbps air rate, 22 dBm, module UART at 115200 in normal mode (config mode is always 9600).
- **Legal limit:** India SRD rules 2021 cap the **duty cycle at 2.5 %** (tracking devices; 1 % for non-specific). TX spacing = `max(150 ms, SRTT/2) / 2.5 %`, about 6 s per ping at 2.4 kbps. At long range this cap, not the code, sets reacquisition time.
- **AUX does NOT indicate RF transmit time** (it only covers UART intake), so airtime is bounded conservatively instead of measured.
- **Operating concept (decided):** **one-way beacon, receive-only drone.** Ping/pong is kept only as a test tool. The owner asked to **confirm the link in ping/pong first**, then switch to one-way. The ESP32 beacon role still lacks a one-way mode; the UNO has one (`BEACON_MODE 1`) and the tester has `mode listen`.

## Current status (2026-10-23)
- All firmware compiles. The new firmware is **not yet verified over the air**.
- **Open fault:** the black ESP32's E220 is **not entering normal mode**.
  - Evidence (THESIS_NOTES §7 item 9): config works, but the mode-0-only RSSI command fails. While black ran, white measured −18 dBm continuous in-band energy, versus −97 dBm with black held in reset.
  - In WOR-transmit mode, black's module jammed the channel and sent pongs with a ~2 s preamble.
  - Diagnosis: the **M0 (GPIO32) / M1 (GPIO33) jumpers on black are not effectively connected**. They may sit one pin off on GPIO34/35, which are input-only, or be loose. The owner was asked to check them and has not reported back yet.
- Oct 16 walk tests with the old firmware (UNO beacon + white) worked: 446/456 and 186/196 packets OK, RSSI −24 … −77 dBm.

## Next steps (in order)
1. Fix black's M0/M1 wiring, then re-test ping/pong at about 1 m with both boards on the laptop. Read both serial ports at once; white should print `>>> LINK UP`.
2. *(Suggested)* At boot, have the firmware verify the module really is in normal mode (the RSSI command answers) and refuse to run otherwise, so a mis-moded module can never jam the band.
3. Add one-way mode to the ESP32 beacon role; the white tester uses `mode listen`.
4. Field range tests: beacon high on a mast. Calibrate with `cal 10` / `cal 30` / `cal 60`, put a `mark` at each checkpoint, then compare a small receiver antenna.
5. Measure real airtime (USRP B210 SDR available) to tighten the duty-cycle bound.
6. Later: adaptive power control (required by Table II), and Pixhawk integration reusing `beacon_link.h`.

## Working conventions (from the owner)
- **YAGNI:** implement only what is needed, with no speculative features. Comments only where someone needs to find what to change.
- Explain decisions with evidence, and record them in THESIS_NOTES.md.
- The owner flashes with the Arduino IDE. arduino-cli also works:
  `arduino-cli compile -u -p <port> -b esp32:esp32:esp32:UploadSpeed=115200 firmware/E220_Tester_ESP32`. Use 115200 upload speed: 921600 corrupted uploads through the USB hub.
