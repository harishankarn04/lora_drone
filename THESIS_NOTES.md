# Thesis notes: LoRa (E220-900T22D) beacon for drone homing

A running engineering log: what was built, why, what was measured, and what went wrong.
Each item is tagged **[measured]** (observed on hardware), **[simulated]** (host simulation
using the firmware's own code), **[sourced]** (from a document or paper) or **[assumed]**
(an engineering estimate not yet verified). Dates are when the work was done.

---

## 1. Goal and scope

- **Long-term goal:** a LoRa beacon on the ground that a drone (Pixhawk 2.4.8 with an ESP32 companion) can detect, range and home on.
- **Current phase:** characterise the E220 modules. That means range indoors and with outdoor line of sight, packet integrity, RSSI-based distance, coarse direction, link latency, and behaviour during signal loss.
- **Out of scope for now:** flight-controller integration. It is planned later over I2C/SPI from the Pixhawk side, so the link logic is kept in a hardware-independent header (`beacon_link.h`) that can be reused.

## 2. Hardware inventory

| Item | Details | How identified |
|---|---|---|
| LoRa modules (×2) | Ebyte E220-900T22D (Semtech LLCC68), 22 dBm, UART interface, white omni antennas | Part number |
| ESP32 "white" (tester) | ESP32-D0WD-V3 rev 3.1, 4 MB flash, CP2102 USB-UART, MAC 1c:c3:ab:b3:c4:68 | `esptool flash-id` **[measured]** |
| ESP32 "black" (beacon, added 2026-10-18) | ESP32-D0WDQ6 rev 1.1, 4 MB flash, CP2102, MAC fc:e8:c0:e1:bb:00 | `esptool flash-id` **[measured]** |
| Arduino UNO R4 Minima | Renesas RA4M1, 5 V logic, native USB-C | `arduino-cli board list` **[measured]** |
| Driver library | xreef "EByte LoRa E220 library" v1.0.8 (Arduino Library Manager) | |

macOS assigns the `/dev/cu.usbserial-*` names by plug order, and both boards' CP2102 chips report serial "0001". Identify boards by **MAC**, not by port name (see §7.6).

## 3. Regulatory constraints (India) [sourced]

Source: *Use of Low Power Equipment in the Frequency Band 865–868 MHz for Short Range Devices (Exemption from Licence) Rules, 2021*, Gazette of India. These supersede the 2005 RFID-only rules.

| Category | Max power | Duty cycle |
|---|---|---|
| Table I, non-specific SRD (telemetry, telecommand, data) | 25 mW e.r.p. | 1 % |
| Table II, tracking, tracing and data acquisition | 500 mW e.r.p., **adaptive power control required** | 2.5 % (10 % for network access points) |
| Table IV, RFID interrogators | 2 W e.r.p. on 4 fixed channels | ≤ 4 s continuous TX |

- The duty cycle is defined over an observation interval. The tables reference ETSI EN 300 220, whose default observation interval is 1 hour **[assumed — check EN 300 220-1]**.
- Our use (beacon for locating and homing) best fits **Table II**, so the firmware defaults to `DUTY_CYCLE_PCT = 2.5`. Table I would require 1 % and about 14 dBm ERP.
- **ERP check:** 22 dBm module + about 3–5 dBi antenna − 2.15 dB (dBi→dBd) ≈ 23–25 dBm ERP ≈ 200–320 mW. That is under 500 mW, so Table II's power limit is met, but **adaptive power control is not implemented yet**. That is a compliance gap to address before real deployment.
- **Channel:** CH 16 = 850.125 + 16 = 866.125 MHz, inside 865–867 MHz. The module default (868.125 MHz) is outside the Indian band.
- **Secondary sources disagree** (one blog says 1 %; the LoRaWAN regional parameters say no limit for IN865). Use the gazette as the primary reference.
- **Key finding:** the original firmware's 1.5 s ping-pong produced about **20 % duty cycle** **[simulated]**, 8× over the Table II limit for continuous use.

## 4. System architecture

```
UNO R4 + E220 (stationary beacon)  <-- 866 MHz LoRa -->  ESP32 + E220 (tester, USB to laptop)
   E220_Beacon_UNO_R4.ino                                   E220_Tester_ESP32.ino + beacon_link.h
                                                            tools/range_logger.py (CSV log)
```

- **Two-way mode (ping/pong):** gives uplink and downlink RSSI, RTT, and which direction lost a packet. The beacon reports its received-ping counter, so a missed reply can be attributed to the uplink or the downlink.
- **One-way mode:** the beacon transmits on its own, which is what the final drone beacon will do. The drone never transmits, so all of the duty-cycle budget goes to the beacon.
- **ESP32 role switch:** the same sketch works as tester or beacon via `#define ROLE`, and the wiring is identical.

## 5. Design decisions and reasoning

### 5.1 UART framing and packet format
- **Own framing instead of the library's receive functions.** We need the raw RSSI byte the module appends, plus resynchronisation after garbage.
- **Packet reduced from 32 to 10 bytes (2026-10-18).**
  - The old 14-byte test pattern was redundant: the LLCC68 hardware CRC already discards corrupted frames, and in practice 0 pattern errors were ever seen **[measured]**.
  - The tester keeps its own send time, so the 4-byte timestamp was unnecessary.
  - LoRa airtime grows with payload in 4·SF-bit blocks (Semtech time-on-air formula **[sourced]**), so fewer bytes means fewer symbols. Under a fixed duty-cycle budget that directly means more updates per hour.
  - A CRC16 was kept, to catch UART-level corruption on the open-drain line.
- **Length-based framing instead of a 15 ms idle gap.** The frame is processed the moment its last byte arrives, which removes about 15 ms per hop.

### 5.2 Module UART 9600 → 115200 baud
- **Gain:** 33 bytes take about 34 ms at 9600 versus about 3 ms at 115200. There are 4 UART transfers per round trip, so this saves roughly 120 ms per RTT **[assumed from baud arithmetic]**.
- **Mechanics:** the E220's config mode always uses 9600 baud (the library enforces it). So the sketch configures at 9600, sets the module's normal-mode UART to 115200, then switches the MCU UART.
- **Risk:** on the UNO, TX is open-drain with the module's internal pull-up, so edges rise slowly. Whether 115200 works reliably through it is **not yet verified**.

### 5.3 5 V UNO to 3.3 V E220 without resistors
- **Problem:** the UNO R4 has 5 V I/O and the E220 has 3.3 V I/O, and no resistors or level shifter were available.
- **Solution:** set D1 (TX), D7 (M0) and D6 (M1) to **NMOS open-drain** through the RA4M1 PFS register (the NCODR bit). The UNO only pulls the lines LOW; HIGH comes from the E220's internal pull-ups. The datasheet lists RXD as a pull-up input and M0/M1 as weak pull-ups **[sourced: Mischianti/Ebyte]**.
- **Finding:** the Arduino R4 core's `OUTPUT_OPENDRAIN` sets **PMOS** open-drain (high-side), the wrong polarity, so the register had to be written directly **[measured from the core source]**.
- **Other direction:** the E220's 3.3 V outputs into the 5 V RA4M1 are below the datasheet's guaranteed VIH (0.8·VCC = 4 V), but they read reliably in practice (PINTEST showed D0 = 1, AUX = 1) **[measured]**.

### 5.4 Non-blocking I/O
- **Removed:** `flush()`, which blocked about 33 ms at 9600, and the busy-wait on AUX of up to 2 s.
- **Now:** TX is refused while AUX is low, and the loop simply retries. On the ESP32, the USB serial TX buffer was raised to 4 KB so debug output never stalls the radio loop.
- **Concurrency choice:** there are no RTOS tasks. The loop is a polled state machine, the UART drivers buffer RX in hardware/ISR, and per-packet CPU work is in the microseconds. Latency is dominated by airtime (hundreds of ms), so threads would add complexity without measurable gain. Revisit if a heavy consumer (MAVLink) shares the loop.

### 5.5 One file vs separate files
- **Runtime:** identical. Arduino compiles the sketch and its headers into one image, with no call or link overhead.
- **Choice:** the timing-critical logic (framer, scheduler, tracker) lives in `beacon_link.h`, which has no Arduino dependencies. That lets the **same code** be compiled into the host simulation, and it eases the later Pixhawk/companion port. The sketches stay as one `.ino` per board for easy flashing.

### 5.6 TX spacing from the duty-cycle budget
- **Rule:** `interval = on_air_estimate × 100 / duty%`, with `on_air_estimate = max(AIRTIME_FLOOR_MS, SRTT/2)`.
- **First attempt (rejected):** measure on-air time from the AUX pin. On hardware, AUX stays low only while the module takes in UART bytes: <1 ms at 115200 and about 13 ms at 9600, which matches 10 bytes of UART time. It does not cover the RF transmission. The bogus "1 ms" reading collapsed the spacing to the 100 ms floor, a duty-cycle violation that only the timeout masked **[measured 2026-10-18]**.
- **Why SRTT/2:** it is an upper bound on one packet's airtime, because it also includes UART time, module latency and the beacon's turnaround. The true duty cycle can therefore only be *below* the target. In simulation, the target was 2.5 % and the true duty was 1.57 % **[simulated]**.
- **Floor:** 150 ms. All common LoRa SF/BW pairs giving about 2.4 kbps (SF9/125k, SF10/250k, SF11/500k) have a 4.096 ms symbol, so a 10-byte packet is ≈ 144 ms **[derived from the Semtech formula]**.
- **No published table:** Ebyte does not publish the SF/BW behind each air-rate setting **[sourced: search found no table]**.
- **Cost of the conservative bound:** a 10.8 s period instead of the ~6.8 s that true airtime would allow. Measuring the real airtime with an SDR (the lab has a USRP B210) would let the floor be tightened with evidence.
- **Constant rate, no bursts:** the update period is fixed and known. That makes worst-case staleness predictable, which is a safety property.
- **Considered and rejected for now:** a token bucket over 1 h. It would allow fast retries during an outage using saved budget, but recovery time would then depend on history, which makes behaviour less predictable.

### 5.7 Timeout: RFC 6298 RTO, no exponential backoff
- **RTO:** SRTT + max(G, 4·RTTVAR), with α = 1/8 and β = 1/4 **[sourced: RFC 6298]**. It adapts to the real RTT instead of the fixed 1.3 s.
- **No backoff:** RFC backoff exists for congestion. This is one point-to-point link with fixed spacing, so backing off would only delay reacquisition.
- **No ambiguity:** each ping has a unique sequence number, so RTT samples are never ambiguous (Karn's retransmission-ambiguity problem does not arise).

### 5.8 Link state and estimate safety
- **States:** NO_CONTACT → TRACKING → COASTING (one expected packet missed) → LOST (estimate invalid).
- **Filter:** a 1-D Kalman filter on RSSI with a random-walk model (q = 4 dB²/s, r = 9 dB²). During gaps the estimate is held and its variance grows with time, following the approach of Sinopoli et al. 2004, *Kalman Filtering with Intermittent Observations*, IEEE TAC 49(9) **[sourced]**.
- **After LOST**, the filter is **re-initialised** from the first new measurement. The old exponential average blended about 70 % of the stale pre-outage value into the first new reading.
- **LOST threshold:** max(3 s, 3 × update period + slack). It is never shorter than what the duty cycle permits, so the link cannot flap between states every cycle.
- **No stale distance:** a held estimate is never printed as a distance (the column is empty on loss lines), and `age_ms` is logged with every loss line.

### 5.9 Operating concept: one-way beacon, receive-only drone (decided 2026-10-18)
- **Requirement (from the project lead):** the drone only needs to *detect* the beacon and home on it. Two-way communication is optional and only useful at close range.
- **Consequences:**
  - The drone never transmits, so the whole duty-cycle budget goes to the beacon. With ping/pong, each side spends budget per update.
  - No round trip, so worst-case detection latency ≈ beacon period + one hop, instead of ping period + RTT.
  - Drone hardware gets simpler: receive only, so no TX power, no TX current spikes and no TX EMI on the airframe.
  - Two-way ping/pong is kept **only as a test tool**, because it measures uplink and downlink separately.
- **Asymmetric design:** a strong, well-sited beacon and a small receiver.
  - **The legal limit caps beacon EIRP, not receiver gain.** Under Table II, ERP ≤ 500 mW (27 dBm ERP ≈ 29.15 dBm EIRP). A bigger PA (e.g. E220-900T30D, 30 dBm) or a high-gain beacon antenna cannot raise radiated power beyond that cap. With the 22 dBm module, the cap is reached with a ~7 dBi antenna.
  - **Receive antenna gain is not regulated,** because it doesn't radiate. Each dB of receiver gain buys range like a dB of TX power. A small drone antenna therefore costs range: about 6 dB lost halves free-space range [derived: FSPL ∝ 20·log10 d].
  - **Beacon siting matters more than power:** antenna height (first Fresnel zone clearance) and an unobstructed mast.
  - **High-gain omnis have an overhead null.** A collinear antenna (≈ 6–8 dBi) squeezes its pattern towards the horizon, so a drone above or near the beacon can drop into the null exactly when it is homing in. A low-gain dipole (≈ 2 dBi) is the safer beacon antenna for the final approach [sourced: antenna theory; verify with the chosen antenna's datasheet pattern].
  - **The drone's noise floor** (ESCs, motors, Pixhawk, ESP32) can cost more dB than the antenna size. Mount the antenna away from ESCs and power wiring.
- **Status:** the receiver side already exists (tester `mode listen`). The beacon-side one-way mode exists on the UNO (`BEACON_MODE 1`) but not yet on the ESP32 beacon role. Nothing is measured yet.

### 5.10 Legal ways to extend range under the EIRP cap [sourced, 2026-10-18]
- **Antenna height and line of sight (the dominant factor).** LoRaWAN balloon flights at about 38 km altitude were heard 702 km and 741 km away with 25 mW transmitters ([Hackster](https://www.hackster.io/news/a-new-lorawan-distance-record-577c8bc11d7b)). A measurement study found about 4 km LOS coverage with nodes at only 1.5 m, and that UAV height and antenna orientation drive coverage (arXiv 1902.11243 / NSF par 10117953).
- **Receive-side gain is not regulated.** An LNA + SAW preamp in front of the receiver adds sensitivity and rejects out-of-band signals. Example: a 16 dB LNA + SAW at a 0.75 dB system noise figure ([The Pi Hut preamp](https://thepihut.com/products/868mhz-filtered-preamplifier)). A field test at 868 MHz over 2 km saw −94 vs −109 dBm and 4–8 % vs 19–23 % loss for boards with vs without a front-end LNA ([Heltec](https://wiki.heltec.org/news/v4-receive-sensitivity/v4-receive-sensitivity)).
  - Caveats: the gain over the LLCC68's own front end is likely only a few dB unless there is cable loss. Receive-only preamps must be bypassed whenever the module transmits (fine for a one-way, receive-only drone).
  - A SAW filter also helps against the 4G Band 20 downlink (up to 862 MHz), which sits close to India's 865–867 MHz.
- **Sensitivity from modulation.** A slower air rate (higher SF) increases sensitivity, at the cost of airtime and therefore the update rate ([Semtech blog](https://blog.semtech.com/long-range-with-lora)). In practice SF10 gave usable 30 s updates, while SF12 slowed to minutes for little extra range in a city ([inovex](https://www.inovex.de/de/?p=51597)).
- **Use beacon EIRP up to the cap, and no further.** 22 dBm + ≤ 7 dBi ≈ 500 mW ERP (Table II requires adaptive power control).
- **Reported air-to-ground/air-to-air tracker ranges:** 9.3 km to 40 km in a vendor test ([TTN forum](https://thethingsnetwork.org/forum/t/extreme-range-test-for-lora-trackers-air-to-air-communication-range-up-to-40km/13375)). Treat these as anecdotal.

## 6. Measurements

### 6.1 Baseline, old firmware (2026-10-16) [measured]
Setup: 32-byte packet, UART 9600, 2.4 kbps air rate, 22 dBm, boards about 0.5 m apart indoors.

| Metric | Value |
|---|---|
| RTT | avg 909 ms (min 908, max 913) over 11 pings |
| RSSI downlink / uplink | −10…−13 dBm (receiver near saturation at this distance) |
| PER | 0 % (11/11) |
| Loss split | uplink 0, downlink 0 |

- **Implied per-hop latency:** about 455 ms. With 33 B UART in and out ≈ 68 ms and 15 ms of framing, airtime plus module overhead ≈ 372 ms per hop **[derived]**.

### 6.1b Walk tests, old firmware (2026-10-16 afternoon) [measured]
UNO R4 beacon + white ESP32 tester, old firmware (32 B, 9600, 1.5 s pings). Real distances were not recorded and the distance model was uncalibrated (A = −20, n = 2.5), so use these runs for RSSI and PER only.
Raw files: `tools/logs/range_20261016_*.csv/.log` (old 15-column CSV format).

| Run | Packets OK / sent | PER | RSSI (avg of up/down) | Notes |
|---|---|---|---|---|
| 12:21–12:33 | 446 / 456 | 1.8 % (end of run) | −24 … −54 dBm | one outage of 13.8 s (8 missed) |
| 12:41 | 5 / 13 | 80 % | −26 … −40 dBm | a 26 s outage, cause not recorded |
| 12:44–12:53 | 186 / 196 | 4.5 % | −29 … −77 dBm | one outage of 15.3 s |

This shows both modules (white's, and the one then on the UNO) linked reliably at these signal levels with the old firmware.

### 6.2 New firmware: pending hardware measurement
To measure: on-air time per packet (printed by the ESP32), RTT, reacquisition across repeated dropouts (`tools/dropout_test.py`), and whether 115200 baud works through the UNO's open-drain line.

## 7. Problems encountered (root causes)

1. **100 % timeouts on the first test.** The UNO received pings (RSSI −9 dBm) but the library reported a failed config save ("Save mode returned not recognized"), and the sketch halted on it. **Fix:** verify config by reading it back, and never halt; retry instead **[measured]**.
2. **Silent failure.** The beacon only printed at boot, so the fault was invisible. **Fix:** a pin self-test plus a STATUS line every 2 s.
3. **Charge-only USB cable.** The board LED lit but no serial port appeared; macOS listed only the hub's own devices. The cause was the cable, not the hub or drivers.
4. **Flash upload corruption.** esptool failed at the default 921600 upload baud ("Inflate error"). It worked at 115200, which points to signal integrity through the hub/cable.
5. **Crash in error handling (old and new code).** On an early `getConfiguration()` failure, the library returns an uninitialised data pointer, and `close()` freed it. That caused a heap assert and a reboot loop. **Fix:** don't free on failure.
6. **Ports swapped after replugging (2026-10-18).** Both CP2102 bridges report serial "0001", so after a replug the original board became `usbserial-5`, and new firmware was flashed to the second ESP32 (its module not yet wired). That produced a misleading "Timeout". **Lesson:** identify boards by MAC.
7. **`link` name clash.** A global variable `link` collided with POSIX `link()` on ESP32; it was renamed `linkState`.
8. **Arduino auto-prototypes.** A helper function placed before a struct definition broke compilation because the IDE inserts prototypes above the first function. The helpers had to be ordered after the types.
9. **No packets between the two ESP32s at very close range (2026-10-18).** Both boards configured identically (config register dump compared field by field), yet even a raw "HELLO" in transparent mode failed in **both directions**, at 22 dBm and at 10 dBm, and at UART 9600 and 115200. A few early pings did decode, at −4 to −6 dBm, the top of the RSSI scale. Firmware is ruled out (the raw test bypasses it), and both modules worked that morning at about 0.5 m with the old firmware. Leading hypothesis: receiver blocking or near-field coupling with the antennas a few cm apart. To be confirmed by re-testing at 1–2 m **[measured; cause unconfirmed]**.
   **Update 2026-10-19:** with the boards across a small room (a few m, reflective), the white tester got 0/12 replies. The proximity hypothesis is weakened. The white side is healthy (module configured, pings every 6 s). The next step is to read the black beacon's own log, to see whether pings arrive there at all, and to swap modules between boards so the fault can be traced to a board or a module **[measured]**.
   **Update 2026-10-19 (black side):** black (beacon) booted and configured fine but received **0 pings in 90 s** with white pinging every 6 s across the room. One capture showed black resetting repeatedly just after the radio config line, with the cause not captured; later captures over 90 s showed no resets. Status: firmware, distance and config are ruled out, so the fault is in one module's RF path (antenna/connector, damaged PA/LNA, or crystal offset) or in black's power. **Next:** swap the E220 modules between the boards; if the fault follows a module, that module is bad. Optional: use the USRP B210 to check whether each module emits at 866.125 MHz and measure its frequency offset **[measured; cause unconfirmed]**.
   **Update 2026-10-19 (after repositioning antennas):** white rebooted cleanly into the firmware and pinged every 6 s, with black across the room on a power bank and antennas repositioned. Result: 0/12 replies. That makes antenna placement in the room an unlikely cause **[measured]**.
   **ROOT CAUSE FOUND (2026-10-19): black's E220 is not in normal mode, because its M0/M1 control lines are not effectively driven [measured].**
   - **Method 1 (wiring check):** reading the module config 27 times in a row gave 27/27 good. The UART wires (TXD/RXD) and AUX are fine.
   - **Method 2 (channel-energy probe):** the E220 "read RSSI" command (C0 C1 C2 C3 00 02) returns the current channel energy, and it only works in normal mode (mode 0). On white it worked every time (0 failures), showing a noise floor of about −97 dBm when quiet. On black it failed 100 % of the time, so black's module was not in mode 0.
   - **Method 3 (A/B jamming test):** white measured **−18 dBm continuous** in-band energy while black was running, **−97 dBm** while black's ESP32 was held in reset (EN low via USB RTS, so its GPIOs float and the module goes to sleep/config). That is about 80 dB attributable to black. Black's module was in **mode 1 (WOR transmit, M0 = 1, M1 = 0)**: every UART byte was sent over the air with a ~2 s wake-up preamble. The probe's 6-byte command every 50 ms therefore became near-continuous transmission.
   - **After the next reboot** black stayed silent but still failed the probe, i.e. a different wrong mode (likely mode 2, WOR receive). The mode varies per boot, which indicates a floating control input.
   - **Explains every earlier symptom:**
     - Config always succeeded, because config mode needs M0 = M1 = 1, which the module's pull-ups provide anyway.
     - Black decoded only every 4th ping, because it was busy transmitting long WOR-preamble pongs and cannot receive while transmitting.
     - White never got a pong in time (each pong took > 2 s on air, longer than the 1.5 s RTO).
     - Black itself jammed the channel.
   - **Likely physical cause:** the M0 (GPIO32) and/or M1 (GPIO33) jumpers are not connected to those pins. On the ESP32 DevKit, GPIO32/33 sit next to GPIO34/35, which are input-only; a jumper one pin off cannot drive the line. A loose jumper would do the same.
   - **Lessons:**
     1. "Config OK" does not prove the M0/M1 wiring, because config mode is the pull-up default state.
     2. The E220 ambient-RSSI command plus an A/B reset test is a cheap, effective spectrum diagnostic, with no SDR needed.
     3. A mis-moded module can jam the band, which is a regulatory and safety risk. **Future firmware should verify normal mode at boot** (e.g. the RSSI command must answer) and refuse to run otherwise.

10. **ESP32 left in bootloader by a port conflict (2026-10-19).** Running `esptool chip-id` while `range_logger.py` held the same serial port reset the ESP32 into download mode, and esptool then failed. The board stayed at "waiting for download" and no firmware ran. **Lesson:** never run esptool or flashing while another program has the port open. Recover by pressing EN/RST or replugging.

## 8. Simulation (2026-10-18) [simulated]

- **Method:** `tools/sim/link_sim.cpp` compiles the firmware's own `beacon_link.h`. It drives the scheduler and tracker against a link model: per-hop latency, measured or assumed on-air time, and 5 % random loss per hop.
- **Dropouts:** 56 outages of 0.3, 1, 2, 5, 10, 30 and 60 s, eight of each, at random phase.
- **Old firmware:** its rules are modelled (fixed 1.5 s interval, 1.3 s timeout, LOST at 5 s).
- **Assumptions [assumed]:** new hop = 271 ms and true on-air = 170 ms. These assume SF9/BW125-like airtime at the E220 "2.4k" setting and must be replaced with measured values.
- **Run 1 (AUX-measured airtime)** was superseded after the AUX finding in §5.6. Run 2 uses `max(150 ms, SRTT/2)`; the true airtime is used only to account for duty cycle.

| Configuration (run 2) | True duty (1 h max) | Period | Loss→COASTING p50 | Loss→LOST p50 | Reacquire p50 / p95 / max |
|---|---|---|---|---|---|
| Old firmware | 20.0 % | 1.5 s | n/a | 4.3 s | 1.34 / 1.89 / 4.75 s |
| New code, 20 % duty target (code gains only) | 12.6 % | 1.36 s | 1.1 s | 3.8 s | 0.94 / 1.59 / 6.06 s |
| New code, 2.5 % target (shipped) | **1.57 %** | 10.8 s | 5.0 s | 29.9 s | 5.5 / 16.6 / 21.2 s |

- **Run 1, for reference (if airtime were known exactly):** 2.5 % → period 6.8 s, reacquisition p50 4.4 s; 20 % → reacquisition p50 0.67 s.

- **Safety checks:** in all cases, 0 ms of stale-but-valid estimates and 0 blends of stale values after LOST.
- **Framer:** recovered all good frames around garbage, a false magic byte, a corrupted CRC and a stale partial frame.
- **Interpretation:**
  - Even while using *less* true duty than the old firmware (12.6 % vs 20 %), the new code reacquires faster (p50 0.94 s vs 1.34 s). That comes from adaptive timeouts and shorter packets.
  - At 2.4 kbps the **legal duty cycle dominates**: reacquisition is bounded by the update period (≈ on-air time / 0.025). Near-instant reacquisition at long range needs less airtime per packet, i.e. a higher air rate (shorter range).
  - This range/latency/regulation trade-off is a central result for the thesis.

## 9. Open questions and next steps

- Measure on-air time for each E220 air-rate setting (2.4k … 62.5k) and build the trade-off table of airtime → legal update period → reacquisition → measured range.
- Field range tests indoors and with outdoor LOS, with distance calibration (`cal` at 10/30/60 m).
- Adaptive power control (needed for Table II compliance).
- Confirm the EN 300 220 observation interval and the device category with WPC or a compliance expert.
- Pixhawk 2.4.8 integration over I2C/SPI, reusing `beacon_link.h`.

## 10. References

1. Gazette of India, *Use of Low Power Equipment in the Frequency Band 865–868 MHz for Short Range Devices (Exemption from Licence) Rules, 2021*.
2. V. Paxson et al., *RFC 6298: Computing TCP's Retransmission Timer*, IETF, 2011.
3. B. Sinopoli et al., "Kalman Filtering with Intermittent Observations," *IEEE Trans. Automatic Control*, 49(9):1453–1464, 2004.
4. Semtech, LoRa time-on-air formula (AN1200.13 / SX127x datasheet), as reproduced in D. Magrin et al., ns-3 LoRaWAN module, arXiv:1811.05829.
5. Ebyte, *E220-900T22D User Manual* v1.0; R. Mischianti, "Ebyte LoRa E220 LLCC68 … specs and basic use" (pin types).
6. ETSI EN 300 220 (SRD 25–1000 MHz), referenced by the Indian rules.
