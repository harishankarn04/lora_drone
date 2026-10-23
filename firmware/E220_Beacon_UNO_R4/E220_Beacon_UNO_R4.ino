/*
 * E220-900T22D range test — STATIONARY BEACON (Arduino UNO R4 Minima)
 *
 * Leave this board on the ground with just power (USB-C power bank is fine).
 *
 * BEACON_MODE 0 (default) = RESPONDER: answers every PING from the ESP32 with a
 *                           PONG. Gives two-way stats (uplink + downlink RSSI,
 *                           which leg lost a packet, round-trip time).
 * BEACON_MODE 1           = ONE-WAY BEACON: transmits as often as DUTY_CYCLE_PCT allows
 *                           (spacing from the measured on-air time), like the final
 *                           drone beacon will. Put the ESP32 in "mode listen".
 * Serial command (USB, for dropout tests): "mute <ms>" = ignore radio + stay silent.
 *
 * Wiring (see USAGE.md in the repo root for the full table) - plain jumper wires, no resistors:
 *   E220 TXD -> D0 (Serial1 RX)      E220 RXD <- D1 (Serial1 TX)
 *   E220 AUX -> D2                   E220 M0  <- D7
 *   E220 M1  <- D6                   E220 VCC -> 5V      E220 GND -> GND
 *
 * 5 V -> 3.3 V without resistors: the UNO R4 runs at 5 V but the E220 I/O is 3.3 V.
 * D1, D7 and D6 are switched to NMOS open-drain: the UNO only ever pulls them LOW,
 * and the E220's own internal pull-ups bring them HIGH to 3.3 V. So no 5 V ever
 * reaches the module. (Set USE_OPEN_DRAIN 0 only if you add a level shifter.)
 *
 * Library: xreef EByte LoRa E220 (https://github.com/xreef/EByte_LoRa_E220_Series_Library)
 */

// Must be defined before the library include (sets 850 MHz base for channel math
// and the 22 dBm power table of the E220-900T22D).
#define FREQUENCY_868
#define E220_22
#include "LoRa_E220.h"

// ------------------------- user settings ----------------------------------
#ifndef BEACON_MODE
#define BEACON_MODE          0      // 0 = responder (two-way), 1 = one-way beacon
#endif

// ---- LINK TIMING (change here) ----
// India SRD rules 2021: 2.5 % duty cycle (tracking devices), 1 % (non-specific SRD).
#define DUTY_CYCLE_PCT     2.5f
#define AIRTIME_FLOOR_MS   150      // on-air time of one 10-byte packet at "2.4k" (upper bound)
#define MIN_INTERVAL_MS    100      // one-way beacon: floor on spacing
#define MODULE_UART_BAUD   115200   // E220 UART in normal mode (config mode is always 9600)

// ---- RADIO SETTINGS: MUST BE IDENTICAL ON BOTH BOARDS ----
#define RADIO_CHANNEL   16                    // 850.125 + 16 = 866.125 MHz (India 865-867 MHz ISM)
#define RADIO_AIR_RATE  AIR_DATA_RATE_010_24  // 2.4 kbps = longest range
#define RADIO_POWER     POWER_22              // 22 dBm (max of the T22D)
#define NET_ID          0x42                  // ignore packets from other people's modules

// ---- pins ----
#define PIN_AUX  2
#define PIN_M0   7
#define PIN_M1   6
#define STATUS_LED  LED_BUILTIN
// 1 = direct jumper wiring, D1/D7/D6 open-drain, HIGH comes from the E220's pull-ups.
// 2 = same, plus the UNO's weak internal pull-up (~30k to 5V, only microamps) in case the
//     module's own pull-ups are too weak (STATUS shows m0/m1/tx = 0 while released).
// 0 = push-pull 5V outputs: ONLY with a level shifter / divider.
#define USE_OPEN_DRAIN 1
// 1 = don't touch the module config (use it if the module was configured on the ESP32 and
//     M0+M1 are jumpered to GND). See README "fallback".
#define SKIP_RADIO_CONFIG 0
// ---------------------------------------------------------------------------

#define E220_UART Serial1
// M0/M1 are driven by this sketch (open-drain), not by the library: the library
// would set them to push-pull 5 V outputs in begin().
LoRa_E220 e220(&E220_UART, PIN_AUX, UART_BPS_RATE_9600);


// ------------- packet format (same as ESP32 beacon_link.h, 10 bytes) -------------
enum : uint8_t { PKT_PING = 1, PKT_PONG = 2, PKT_BEACON = 3 };

struct __attribute__((packed)) Packet {
  uint8_t  magic;      // 0xB3
  uint8_t  netId;
  uint8_t  type;
  uint16_t seq;        // PONG: echo of PING seq
  uint8_t  rssiUp;     // PONG: raw RSSI byte measured on the PING
  uint16_t beaconRx;   // PONG: PINGs received since boot
  uint16_t crc;        // CRC16-CCITT over the bytes above
};
static_assert(sizeof(Packet) == 10, "packet must be 10 bytes");
static const uint8_t MAGIC = 0xB3;
static const size_t  FRAME_LEN = sizeof(Packet) + 1;   // + RSSI byte appended by module

uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}

int rssiDbm(uint8_t raw) { return -(256 - (int)raw); }

// ---------------------------------------------------------------------------
uint16_t pingsRx = 0, beaconSeq = 0;
uint32_t badFrames = 0;
uint8_t  rxBuf[FRAME_LEN];
size_t   rxLen = 0;
uint32_t lastByteMs = 0, lastPingMs = 0, muteUntil = 0;
bool     radioOk = false;
String   cfgError = "not tried";

// Non-blocking TX: refuses while the module is busy (AUX low); never waits.
Packet   txQueued; bool txWaiting = false;
bool     txPending = false, txSawLow = false;
uint32_t txStartMs = 0, nextBeaconMs = 0;

bool txPacket(Packet &p) {
  if (txPending || digitalRead(PIN_AUX) == LOW) return false;
  p.magic = MAGIC; p.netId = NET_ID;
  p.crc = crc16((uint8_t *)&p, sizeof(p) - 2);
  E220_UART.write((uint8_t *)&p, sizeof(p));
  txPending = true; txSawLow = false; txStartMs = millis();
  return true;
}

// AUX is low only while the module takes in the UART bytes (not during RF TX), so it
// only gates the next write; on-air time comes from AIRTIME_FLOOR_MS.
void pollTxDone() {
  if (!txPending) return;
  bool aux = digitalRead(PIN_AUX);
  if (!aux) txSawLow = true;
  if ((txSawLow && aux) || millis() - txStartMs > 50) {
    txPending = false;
    digitalWrite(STATUS_LED, LOW);
  }
}

// RA4M1 NMOS open-drain. The core's OUTPUT_OPENDRAIN enables PMOS open-drain
// (high-side only), which is the wrong polarity, so set the PFS bit ourselves.
void pinOpenDrain(pin_size_t pin, bool keepPeripheral) {
  bsp_io_port_pin_t p = g_pin_cfg[pin].pin;
  uint32_t pull = (USE_OPEN_DRAIN == 2) ? IOPORT_CFG_PULLUP_ENABLE : 0;
  R_BSP_PinAccessEnable();
  if (keepPeripheral)   // e.g. UART TX: keep the SCI function, only add NCODR
    R_PFS->PORT[p >> 8].PIN[p & 0xFF].PmnPFS |= IOPORT_CFG_NMOS_ENABLE | pull;
  else                  // GPIO output, starts released (HIGH via module pull-up)
    R_PFS->PORT[p >> 8].PIN[p & 0xFF].PmnPFS =
        IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_NMOS_ENABLE | IOPORT_CFG_PORT_OUTPUT_HIGH | pull;
  R_BSP_PinAccessDisable();
}

// Raw pin level (works for GPIO and for peripheral pins like the UART).
int pinLevel(pin_size_t pin) {
  bsp_io_port_pin_t p = g_pin_cfg[pin].pin;
  return (R_PFS->PORT[p >> 8].PIN[p & 0xFF].PmnPFS & (1u << 1)) ? 1 : 0;   // PIDR bit
}

// Config mode needs 9600; normal mode runs at MODULE_UART_BAUD. Re-begin resets the
// pin function, so open-drain on D1 is re-applied every time.
void moduleUart(uint32_t baud) {
  E220_UART.end();
  E220_UART.begin(baud);
#if USE_OPEN_DRAIN
  pinOpenDrain(1, true);
#endif
}

void modePins(MODE_TYPE m) {   // M0/M1: normal = 0/0, config = 1/1
  bool hi = (m == MODE_3_PROGRAM);
  digitalWrite(PIN_M0, hi ? HIGH : LOW);
  digitalWrite(PIN_M1, hi ? HIGH : LOW);
  delay(40);
}

bool configureRadio() {
#if SKIP_RADIO_CONFIG
  cfgError = "skipped (SKIP_RADIO_CONFIG=1)";
  moduleUart(MODULE_UART_BAUD);   // module was configured elsewhere with this baud
  return true;
#endif
  moduleUart(9600);
  modePins(MODE_3_PROGRAM);
  ResponseStructContainer c = e220.getConfiguration();
  if (c.status.code != E220_SUCCESS) {
    cfgError = "getConfiguration: " + c.status.getResponseDescription();
    // no c.close(): on early failure the library leaves c.data uninitialised (free() would crash)
    Serial.print(F("getConfiguration failed: code ")); Serial.print((int)c.status.code);
    Serial.print(F(" ")); Serial.println(c.status.getResponseDescription());
    modePins(MODE_0_NORMAL);
    moduleUart(9600);
    return false;
  }
  Configuration cfg = *(Configuration *)c.data;
  c.close();

  cfg.ADDH = 0x00;
  cfg.ADDL = 0x00;
  cfg.CHAN = RADIO_CHANNEL;
  cfg.SPED.uartBaudRate = MODULE_UART_BAUD == 115200 ? UART_BPS_115200 : UART_BPS_9600;
  cfg.SPED.uartParity   = MODE_00_8N1;
  cfg.SPED.airDataRate  = RADIO_AIR_RATE;
  cfg.OPTION.subPacketSetting  = SPS_200_00;
  cfg.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_ENABLED;
  cfg.OPTION.transmissionPower = RADIO_POWER;
  cfg.TRANSMISSION_MODE.enableRSSI        = RSSI_ENABLED;   // module appends RSSI byte to every RX packet
  cfg.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  cfg.TRANSMISSION_MODE.enableLBT         = LBT_DISABLED;
  cfg.TRANSMISSION_MODE.WORPeriod         = WOR_2000_011;

  e220.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
  // The save reply is sometimes misparsed by the library ("not recognized") even though
  // the write worked, so verify by reading the config back instead.
  c = e220.getConfiguration();
  Configuration *v = (Configuration *)c.data;
  bool ok = c.status.code == E220_SUCCESS && v->CHAN == RADIO_CHANNEL &&
            v->SPED.airDataRate == RADIO_AIR_RATE && v->TRANSMISSION_MODE.enableRSSI == RSSI_ENABLED;
  if (c.status.code != E220_SUCCESS) c.data = nullptr;   // see above: never free an uninitialised pointer
  c.close();
  modePins(MODE_0_NORMAL);
  moduleUart(ok ? MODULE_UART_BAUD : 9600);
  cfgError = ok ? "OK" : "read-back mismatch";
  Serial.print(F("radio config: ")); Serial.println(cfgError);
  return ok;
}

// One line every 2 s so a Serial Monitor opened at any time shows what is going on.
void printStatus() {
  Serial.print(F("STATUS up=")); Serial.print(millis() / 1000);
  Serial.print(F("s radio=")); Serial.print(radioOk ? F("OK") : F("FAIL"));
  Serial.print(F(" (")); Serial.print(cfgError); Serial.print(F(")"));
  // Idle levels: E220 TXD and AUX should read 1. m0/m1 read 1 only while released (config).
  Serial.print(F(" | pins D0(E220 TXD)=")); Serial.print(pinLevel(0));
  Serial.print(F(" D1(E220 RXD)=")); Serial.print(pinLevel(1));
  Serial.print(F(" D2(AUX)=")); Serial.print(pinLevel(PIN_AUX));
  Serial.print(F(" M0=")); Serial.print(pinLevel(PIN_M0));
  Serial.print(F(" M1=")); Serial.print(pinLevel(PIN_M1));
  Serial.print(F(" | pings=")); Serial.print(pingsRx);
  Serial.print(F(" bad=")); Serial.print(badFrames);
  if (pingsRx) { Serial.print(F(" last_ping=")); Serial.print((millis() - lastPingMs) / 1000); Serial.print(F("s ago")); }
  else Serial.print(F(" (no ping received yet)"));
  Serial.println();
}

// Released-pin test: with M0/M1/TX released, the E220's pull-ups should make them read 1.
void pinSelfTest() {
  modePins(MODE_3_PROGRAM);           // release M0/M1
  delay(5);
  Serial.print(F("PINTEST released: D1(TX)=")); Serial.print(pinLevel(1));
  Serial.print(F(" M0=")); Serial.print(pinLevel(PIN_M0));
  Serial.print(F(" M1=")); Serial.print(pinLevel(PIN_M1));
  Serial.print(F(" | module outputs: D0(TXD)=")); Serial.print(pinLevel(0));
  Serial.print(F(" D2(AUX)=")); Serial.println(pinLevel(PIN_AUX));
  Serial.println(F("PINTEST expect all 1. D0/AUX=0 -> module unpowered/miswired or 3.3V not read as HIGH."
                   " TX/M0/M1=0 with D0=1 -> module pull-ups too weak: try USE_OPEN_DRAIN 2."));
  modePins(MODE_0_NORMAL);
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't block when on a power bank

#if USE_OPEN_DRAIN
  pinOpenDrain(PIN_M0, false);
  pinOpenDrain(PIN_M1, false);
#else
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
#endif
  e220.begin();                       // starts Serial1 (D0/D1)
#if USE_OPEN_DRAIN
  pinOpenDrain(1, true);              // D1 = Serial1 TX -> open-drain
#endif
  Serial.println(F("\nE220 beacon (UNO R4) booting"));
  pinSelfTest();
  radioOk = configureRadio();
  while (E220_UART.available()) E220_UART.read();

  Serial.print(F("E220 beacon mode="));
  Serial.println(BEACON_MODE ? F("ONE-WAY BEACON") : F("RESPONDER"));
  if (radioOk)
    for (int i = 0; i < 3; i++) { digitalWrite(STATUS_LED, HIGH); delay(150); digitalWrite(STATUS_LED, LOW); delay(150); }
  printStatus();
}

void onPacket(const Packet &p, uint8_t rssiRaw) {
  if (p.type != PKT_PING || (int32_t)(millis() - muteUntil) < 0) return;
  pingsRx++;
  lastPingMs = millis();
#if BEACON_MODE == 0
  txQueued = p;                 // keeps seq
  txQueued.type     = PKT_PONG;
  txQueued.rssiUp   = rssiRaw;
  txQueued.beaconRx = pingsRx;
  txWaiting = !txPacket(txQueued);
  digitalWrite(STATUS_LED, HIGH);
#endif
  Serial.print(F("PING seq=")); Serial.print(p.seq);
  Serial.print(F(" rssi=")); Serial.print(rssiDbm(rssiRaw)); Serial.print(F(" dBm"));
  Serial.print(F(" rx=")); Serial.print(pingsRx);
  Serial.print(F(" bad=")); Serial.println(badFrames);
}

// Length-based framing: a frame is handled the moment its last byte arrives.
// Resyncs on magic + netId; a partial frame older than 3 ms is dropped as garbage.
void onByte(uint8_t b) {
  if (rxLen && millis() - lastByteMs > 3) rxLen = 0;
  lastByteMs = millis();
  rxBuf[rxLen++] = b;
  while (rxLen) {
    if (rxBuf[0] != MAGIC || (rxLen > 1 && rxBuf[1] != NET_ID)) { memmove(rxBuf, rxBuf + 1, --rxLen); continue; }
    if (rxLen < FRAME_LEN) return;
    Packet p;
    memcpy(&p, rxBuf, sizeof(p));
    if (p.crc != crc16(rxBuf, sizeof(p) - 2)) { badFrames++; memmove(rxBuf, rxBuf + 1, --rxLen); continue; }
    rxLen = 0;
    onPacket(p, rxBuf[sizeof(p)]);
    return;
  }
}

void pollCommands() {   // "mute <ms>": simulate a beacon dropout (tools/dropout_test.py)
  static char buf[24]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0; n = 0;
    if (!strncmp(buf, "mute ", 5)) {
      muteUntil = millis() + strtoul(buf + 5, nullptr, 10);
      txWaiting = false;
      Serial.print(F("MUTED until ")); Serial.println(muteUntil);
    }
  }
}

void loop() {
  static uint32_t lastStatus = 0, lastRetry = 0;
  while (E220_UART.available()) onByte(E220_UART.read());
  pollTxDone();
  if (txWaiting) txWaiting = !txPacket(txQueued);
  pollCommands();

  if (millis() - lastStatus >= 2000) { lastStatus = millis(); printStatus(); }
  if (!radioOk && millis() - lastRetry >= 5000) {        // keep retrying instead of halting
    lastRetry = millis();
    radioOk = configureRadio();
    while (E220_UART.available()) E220_UART.read();
  }
  if (!radioOk) digitalWrite(STATUS_LED, (millis() / 100) % 2);   // fast blink = config failed

#if BEACON_MODE == 1
  // Constant rate from the duty-cycle budget: spacing = on-air time / duty.
  if ((int32_t)(millis() - nextBeaconMs) >= 0 && (int32_t)(millis() - muteUntil) >= 0) {
    Packet b = {};
    b.type = PKT_BEACON;
    b.seq  = beaconSeq + 1;
    if (txPacket(b)) {
      beaconSeq++;
      uint32_t gap = (uint32_t)ceilf(AIRTIME_FLOOR_MS * 100.0f / DUTY_CYCLE_PCT);
      nextBeaconMs = millis() + (gap > MIN_INTERVAL_MS ? gap : MIN_INTERVAL_MS);
      digitalWrite(STATUS_LED, HIGH);
    }
  }
#endif
}
