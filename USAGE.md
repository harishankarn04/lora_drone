# E220-900T22D range test (UNO R4 Minima beacon + ESP32 tester)

```
 [UNO R4 Minima + E220]  stationary, power bank only       <-- 866 MHz LoRa -->   [ESP32 + E220] -- USB -- laptop
   E220_Beacon_UNO_R4                                                              E220_Tester_ESP32 + tools/range_logger.py
```

* **Packet integrity**: every packet is 32 bytes (sequence number, timestamp, 14-byte known pattern, CRC16).
  The receiver checks CRC and pattern, and counts lost, corrupted, late and duplicate packets plus PER %.
* **Two-way mode** (default): the ESP32 sends a PING and the UNO answers with a PONG. You get RSSI in **both** directions,
  the round-trip time, and *which leg* lost a packet (the UNO reports how many pings it has received).
* **One-way mode**: the UNO transmits on its own once a second, the way the drone beacon will.
* **Distance**: estimated from RSSI with a log-distance model you calibrate on site (`cal <m>`).
* **Direction**: `scan` turns you into a manual direction finder in 45° steps (details below).

## 1. Radio settings (already set in both sketches)

| Setting | Value | Why |
|---|---|---|
| Channel | 16 → **866.125 MHz** | Inside the Indian licence-free band (865–867 MHz). The module default, 868.125 MHz, falls outside it. |
| Air data rate | 2.4 kbps | Slowest rate, so the longest range |
| TX power | 22 dBm | Maximum for the T22D |
| RSSI byte | enabled | The module appends the RSSI of each received packet |
| UART | 115200 8N1 in normal mode, transparent mode | 4× fewer ms per byte than 9600. Config mode is always 9600, handled automatically. |
| Duty cycle | **2.5 %** of measured on-air time (`DUTY_CYCLE_PCT`) | India SRD rules 2021, 865–868 MHz: Table II tracking devices 2.5 % / 500 mW ERP (adaptive power control required), Table I non-specific 1 % / 25 mW ERP. |

Both sketches write this config into the module at boot. **If you change it, change it in both sketches.**

### Latency and reacquisition: what limits it

Each TX is spaced at *measured on-air time ÷ duty cycle*, which keeps it legal and gives a fixed, known update period.
At 2.4 kbps one 10-byte packet takes about 170 ms on air (estimated; the ESP32 prints the measured value), so the
legal period is about 6.8 s. **The duty cycle, not the code, sets reacquisition time at long range.** To reacquire faster
you need less airtime per packet, which means a higher `RADIO_AIR_RATE` (shorter range). Exceeding the duty cycle is
not an option.

Run `tools/sim/run_sim.sh` to simulate repeated dropouts with the firmware's own `beacon_link.h`.
Run `tools/dropout_test.py` to measure the same on the real boards (it mutes the UNO over USB).

## 2. Wiring

E220-900T22D pin order on the module header: `M0 M1 RXD TXD AUX VCC GND`.

### UNO R4 Minima (stationary beacon): plain jumper wires, no resistors needed

| E220 pin | UNO R4 pin | Notes |
|---|---|---|
| VCC | **5V** | 5 V gives full 22 dBm. Add a 100–470 µF capacitor across VCC–GND if you have one (optional). |
| GND | GND | |
| TXD | **D0** (Serial1 RX) | 3.3 V into the UNO, which is safe |
| RXD | **D1** (Serial1 TX) | open-drain, see below |
| AUX | **D2** | 3.3 V into the UNO, which is safe |
| M0 | **D7** | open-drain, see below |
| M1 | **D6** | open-drain, see below |

**How the 5 V vs 3.3 V problem is solved in software:** the sketch puts D1, D7 and D6 into the RA4M1's
**NMOS open-drain** mode (`USE_OPEN_DRAIN 1`). In that mode the UNO can only pull a line LOW or let it go. HIGH
comes from the E220's own internal pull-ups to 3.3 V, so 5 V never reaches the module.
(The core's `OUTPUT_OPENDRAIN` turns out to be PMOS/high-side, the wrong polarity, so the sketch sets the
PFS register directly.) If you later add a level shifter, set `USE_OPEN_DRAIN 0`.

Caveat on the other direction: the E220 drives 3.3 V into D0/D2. On a 5 V RA4M1 the datasheet's guaranteed
logic-HIGH threshold is above 3.3 V, but in practice these pins read 3.3 V UART reliably. To check on the bench: open the
UNO's Serial Monitor (115200). You should see `setConfiguration: Success` at boot and then `PING seq=...` lines while
the ESP32 is running. If config fails or no PINGs appear, add a level shifter.

USB-C is only for flashing. In the field, power the UNO from a USB-C power bank. Some power banks switch off at low current; if yours does, use one with an "always on" or low-current mode.

### If the UNO says `radio=FAIL`: read its STATUS line

The UNO prints `PINTEST` at boot and a `STATUS` line every 2 s (Serial Monitor at 115200), so you can open the monitor at any time.

| What STATUS / PINTEST shows | Meaning | Fix |
|---|---|---|
| `D0(E220 TXD)=0` and `D2(AUX)=0` | Module unpowered or miswired, or the UNO isn't reading 3.3 V as HIGH | Check VCC/GND and that TXD goes to D0 and AUX to D2 (not swapped with RXD) |
| D0=1 and AUX=1, but released `D1/M0/M1=0` | The module's pull-ups are too weak to lift the open-drain lines | Set `USE_OPEN_DRAIN 2` (adds the UNO's ~30 kΩ pull-up, microamps only) |
| All pins 1 but `getConfiguration` fails | RX and TX are swapped | Swap the D0/D1 jumpers |

**Fallback that needs no UNO control of M0/M1:** plug the beacon's E220 into the ESP32 once and boot it. That writes
the config (ch 16, 2.4 kbps, RSSI on) into the module's flash. Then move the module back to the UNO, jumper
**M0 → GND and M1 → GND**, leave D6/D7 unconnected, and set `SKIP_RADIO_CONFIG 1`.

### Knowing whether the link is up (ESP32 side)

* `>>> LINK UP`: the first reply arrived
* `>>> LINK COASTING`: one expected packet missed. The last estimate is held while its uncertainty grows.
* `>>> LINK LOST`: no packet for `max(3 s, 3 × update period)`. The estimate is **invalid** and no distance is printed until reacquired.
* `>>> LINK RECOVERED / RESTORED after X ms`: back from COASTING / LOST. After LOST the filter restarts from the new reading and is never blended with the old one.
* `>>> NO CONTACT with beacon yet`: repeats every 10 s until the first reply arrives

### ESP32 (mobile tester on the laptop), 3.3 V logic: connect directly

The sketch chooses pins automatically from the board you select in the Arduino IDE. It also prints them at boot.

| E220 pin | Classic ESP32 DevKit (WROOM/WROVER) | ESP32-S3 DevKit | ESP32-C3 |
|---|---|---|---|
| VCC | **5V / VIN** (from USB) | 5V | 5V |
| GND | GND | GND | GND |
| TXD | **GPIO26** | GPIO18 | GPIO4 |
| RXD | **GPIO27** | GPIO17 | GPIO5 |
| AUX | **GPIO25** | GPIO8 | GPIO6 |
| M0 | **GPIO32** | GPIO5 | GPIO7 |
| M1 | **GPIO33** | GPIO6 | GPIO10 |

On the classic ESP32 the code avoids GPIO16/17 on purpose: on WROVER/PSRAM ("R" in the part number) modules those pins belong to the PSRAM.

### Antenna

* **Never power the E220 without its antenna connected.** Transmitting at 22 dBm into an open SMA can damage the PA.
* Keep both white omni antennas **vertical**, in the same polarization.
* Lift the stationary beacon **at least 1–1.5 m off the ground** (tripod, pole, box). A beacon lying on the ground can lose half its range to ground absorption and loss of Fresnel-zone clearance. Hold the laptop-side antenna up and away from the laptop and your body as well.

## 3. Software

1. Library: in the Arduino IDE open *Library Manager*, search **"EByte LoRa E220 library"** (by Renzo Mischianti) and install **1.0.8**.
   Install only that one. Don't also install the "Alteriom_EByte_LoRa_E220" fork; two copies cause duplicate-library errors.
2. Flash `E220_Beacon_UNO_R4` with board **Arduino UNO R4 Minima** (the `arduino:renesas_uno` core is installed).
   * LED blinks 3× = radio configured OK. LED blinks fast forever = module not answering, so check wiring and M0/M1.
   * For the one-way beacon test, set `#define BEACON_MODE 1`.
3. Flash `E220_Tester_ESP32` with board **ESP32 Dev Module** (yours is an ESP32-D0WD-V3 with 4 MB flash, on a CP210x USB chip, at `/dev/cu.usbserial-0001`).
4. Run the logger on the laptop. It needs pyserial, which is already installed here:
   ```bash
   cd tools && python3 range_logger.py
   ```
   It writes `logs/range_<time>.csv` (one row per packet) and a raw `.log`. Press Ctrl-C to stop and get a summary per marker.
   The Arduino Serial Monitor at 115200 baud also works, but nothing gets saved.

## 4. Field procedure

1. **Bench check (1–2 m apart):** you should see `OK` lines with PER at 0 %. At very short range RSSI may saturate around −20 dBm. That's normal.
2. **Calibrate distance** (outdoors, line of sight). Do it at 2–3 known distances, measured with tape or Google Maps/GPS:
   * Walk to 10 m, wait about 10 packets, type `cal 10`
   * Walk to 30 m, type `cal 30`
   * Walk to 60 m, type `cal 60`
   With 2 or more points, the code fits both the reference RSSI and the path-loss exponent `n`. These values are saved in ESP32 flash.
   Calibrate indoors separately, or set `n 3` by hand (rough guide: about 2.0 in an open field, 2.7–3.5 indoors or through walls).
3. **Walk test:** put a marker at every checkpoint, for example `mark 100m LOS`, `mark 200m LOS`, `mark behind building`.
   Keep walking until you see repeated `TIMEOUT`s. The last distance with PER under about 10 % is your usable range.
   The ESP32 prints a `#` summary every 10 packets. It shows RSSI min/avg/max per direction, RTT, and how many packets were lost on the uplink versus the downlink.
4. **Indoor test:** start again with `reset` and use markers like `mark 2 walls`, `mark floor below`.
5. **Rough reference:** sensitivity at 2.4 kbps is about −124…−129 dBm. Once RSSI drops below about −115 dBm, losses climb quickly.

### Direction (`scan`)

With a single omni antenna, RSSI alone can't give a true bearing, because an omni antenna hears equally in all directions. The tool works around that in two ways:

* **Trend**, on every packet: `CLOSER` / `FARTHER` / `STEADY` is the slope of the filtered RSSI over the last ~12 packets. Walking while watching it already gets you "hot/cold" homing.
* **Body-shadow scan:** type `scan`. Hold the module against your chest and turn on the spot in eight 45° steps, pressing Enter at each one. Your body blocks roughly 5–15 dB behind you. The tool prints the strongest heading, a power-weighted bearing, and "weakest heading + 180°". If the spread is below 4 dB it reports *inconclusive*, which is common in multipath or indoors.
* For a real direction finder on the drone, the same `scan` code works far better with a **directional antenna** (a small 868 MHz Yagi or patch, about 10–20 dB front-to-back ratio) or with two to four antennas compared against each other.

## 5. Interpreting the CSV

`t_ms, mode, seq, status(OK|TIMEOUT|LOST), rtt_ms, rssi_down_dbm (beacon→you), rssi_up_dbm (you→beacon), rssi_avg_dbm, dist_m, trend, attempts, ok, lost, bad (CRC failures), per_pct, link (TRACKING|COASTING|LOST), age_ms (since last good packet)`

* `TIMEOUT`: no pong. The `#` summary tells you whether it was the uplink or the downlink that failed.
* `bad`: frames that failed the CRC16 check. This is rare, because the LoRa hardware CRC already discards corrupted packets, so they show up as `TIMEOUT`/`LOST`.
* RSSI-based distance is approximate. Expect ±30–50 % in the open and worse indoors. Use it as a trend or for coarse ranging, not for positioning.
