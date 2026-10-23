# lora_drone

A LoRa (Ebyte E220-900T22D, 866 MHz) beacon for drone homing: range, link integrity, RSSI distance and dropout/reacquisition testing.

- **New here, or a fresh AI session?** Read [CONTEXT.md](CONTEXT.md) first.
- **How to wire, flash and run tests:** [USAGE.md](USAGE.md)
- **Engineering log for the thesis:** [THESIS_NOTES.md](THESIS_NOTES.md)

## Quick start
1. Arduino IDE: install the **"EByte LoRa E220 library"** (Library Manager) and the ESP32 and/or "Arduino UNO R4" board packages.
2. Flash `firmware/E220_Tester_ESP32` to both ESP32s. On the beacon board set `#define ROLE ROLE_BEACON`; on the laptop board keep `ROLE_TESTER`.
3. Plug the tester into the laptop and power the beacon. Then:
   ```bash
   pip install pyserial
   cd tools && python3 range_logger.py -p /dev/cu.usbserial-0001
   ```
   You should see `>>> LINK UP`, then `DATA` lines. Logs are saved in `tools/logs/`.
4. Simulation (no hardware needed): `tools/sim/run_sim.sh`
