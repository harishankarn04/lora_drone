/*
 * E220-900T22D range test — ESP32 (tester on the laptop, or beacon via ROLE).
 * Serial Monitor 115200, "Newline". Commands (tester): mode ping | mode listen |
 * cal <m> | calclear | n <exp> | mark <text> | scan | stats | reset | help
 * Library: xreef EByte LoRa E220 (Library Manager: "EByte LoRa E220 library")
 */

// ========================= CHANGE HERE: board role =========================
#define ROLE_TESTER 0
#define ROLE_BEACON 1
#define ROLE ROLE_TESTER          // ROLE_BEACON on the stationary board (same wiring)
// ===========================================================================

#define FREQUENCY_868
#define E220_22
#include "LoRa_E220.h"
#include <Preferences.h>
#include <math.h>
#include "beacon_link.h"

// ---- RADIO: MUST MATCH ON BOTH BOARDS ----
#define RADIO_CHANNEL   16                    // 866.125 MHz (India 865-867)
#define RADIO_AIR_RATE  AIR_DATA_RATE_010_24  // 2.4k = max range, longest airtime
#define RADIO_POWER     POWER_22
#define NET_ID          0x42

// ---- LINK TIMING / SAFETY (change here) ----
// India SRD rules 2021 (865-868 MHz): 2.5 % duty cycle for tracking devices (Table II),
// 1 % for non-specific SRDs (Table I). Spacing = on-air estimate / duty, where the estimate is
// max(AIRTIME_FLOOR_MS, RTT/2). Raise the floor if you change RADIO_AIR_RATE to a slower one.
#define DUTY_CYCLE_PCT   2.5f
#define AIRTIME_FLOOR_MS 150      // ~10-byte LoRa packet at the "2.4k" setting (4.1 ms symbols)
#define MIN_INTERVAL_MS  100      // floor on ping spacing
#define LOST_AFTER_MS    3000     // estimate invalid after this long without a packet (raised
                                  // automatically if the duty-limited update period is longer)
#define LOST_MISSES      3        // ...or after this many expected packets are missed
#define MODULE_UART_BAUD 115200   // E220 UART in normal mode (config mode is always 9600)
#define SUMMARY_EVERY    10
#define SCAN_SAMPLES     6

// ---- pins ("RX" = where E220 TXD goes) ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define PIN_RX 18
  #define PIN_TX 17
  #define PIN_AUX 8
  #define PIN_M0 5
  #define PIN_M1 6
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define PIN_RX 4
  #define PIN_TX 5
  #define PIN_AUX 6
  #define PIN_M0 7
  #define PIN_M1 10
#else   // classic ESP32 DevKit
  #define PIN_RX 26
  #define PIN_TX 27
  #define PIN_AUX 25
  #define PIN_M0 32
  #define PIN_M1 33
#endif

#define E220_UART Serial1
LoRa_E220 e220(PIN_RX, PIN_TX, &E220_UART, PIN_AUX, PIN_M0, PIN_M1, UART_BPS_RATE_9600);
Preferences prefs;

using bl::Packet;
bl::Framer framer(NET_ID, 3);
bl::PingScheduler sched({DUTY_CYCLE_PCT, MIN_INTERVAL_MS, AIRTIME_FLOOR_MS, 1500, 50, 3000});
bl::Tracker trk(4.0f, 9.0f);   // q = 4 dB^2/s (walking), r = 9 dB^2 (fading, ~3 dB sigma)

// ------------------------------- state -------------------------------------
bool listenMode = false;

struct Stats {
  uint32_t sent, ok, timeouts, late, lost;
  uint32_t rttSum, rttMin, rttMax;
  int32_t  dSum, uSum; int dMin, dMax, uMin, uMax; uint32_t uCnt;
  bool     haveBase; uint32_t sentBase; uint16_t beaconBase; uint32_t okBase;
  uint32_t upLost, downLost;
  uint16_t lastBeaconSeq; bool haveBeaconSeq;
  uint32_t procMaxUs, procSumUs, procN;     // frame complete -> DATA line written
} st;

// TX in progress until AUX goes low..high (module has taken the bytes)
bool txPending = false, txSawLow = false; uint32_t txStartMs = 0;

// link state printing
bl::Link prevLink = bl::Link::NO_CONTACT;
uint32_t outageStartMs = 0, missStreak = 0, lastLinkMsg = 0;
uint32_t beaconPeriodMs = 0, lastBeaconRxMs = 0;

// distance model: rssi = A - 10*n*log10(d)
float modelA = -20.0f, modelN = 2.5f;
struct CalPt { float d, rssi; };
CalPt cal[8]; uint8_t calCount = 0;

float histT[12], histR[12]; uint8_t histN = 0, histHead = 0;
float recent[10]; uint8_t recentN = 0, recentHead = 0;

bool scanActive = false, scanArmed = false;
uint8_t scanSector = 0, scanCount = 0;
float scanSum = 0, scanRes[8];

char cmdBuf[64]; uint8_t cmdLen = 0;

// ---------------------------------------------------------------------------
void resetStats() {
  memset(&st, 0, sizeof(st));
  st.rttMin = UINT32_MAX; st.dMax = st.uMax = -200;
  histN = histHead = recentN = recentHead = 0;
}

void saveModel() { prefs.putFloat("A", modelA); prefs.putFloat("n", modelN); }

float estimateDistance(float rssi) { return powf(10.0f, (modelA - rssi) / (10.0f * modelN)); }

void fitModel() {
  if (calCount == 0) return;
  if (calCount == 1) {
    modelA = cal[0].rssi + 10.0f * modelN * log10f(cal[0].d);
  } else {
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < calCount; i++) {
      float x = log10f(cal[i].d), y = cal[i].rssi;
      sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float den = calCount * sxx - sx * sx;
    if (fabsf(den) < 1e-4f) {
      modelA = sy / calCount + 10.0f * modelN * (sx / calCount);
    } else {
      modelN = constrain(-((calCount * sxy - sx * sy) / den) / 10.0f, 1.5f, 6.0f);
      modelA = (sy + 10.0f * modelN * sx) / calCount;
    }
  }
  saveModel();
  Serial.printf("# model: RSSI(1m)=%.1f dBm, n=%.2f  (%u cal points)\n", modelA, modelN, calCount);
}

const char *trend() {
  if (histN < 5) return "-";
  float st_ = 0, sr = 0, stt = 0, str = 0;
  for (uint8_t i = 0; i < histN; i++) { st_ += histT[i]; sr += histR[i]; stt += histT[i] * histT[i]; str += histT[i] * histR[i]; }
  float den = histN * stt - st_ * st_;
  if (den <= 0) return "-";
  float slope = (histN * str - st_ * sr) / den;
  return slope > 0.4f ? "CLOSER" : slope < -0.4f ? "FARTHER" : "STEADY";
}

uint32_t lostCount() { return listenMode ? st.lost : st.timeouts; }
uint32_t attemptCount() { return listenMode ? st.ok + st.lost : st.sent; }
float perPct() { uint32_t a = attemptCount(); return a ? 100.0f * (a - st.ok) / a : 0; }

// ---- link timing: derived from the real update period, never shorter than it ----
uint32_t updatePeriodMs() { return listenMode ? beaconPeriodMs : sched.interval(); }
void applyLinkTimeouts() {
  uint32_t period = updatePeriodMs();
  uint32_t slack = listenMode ? period / 2 : sched.rto();
  uint32_t lost = LOST_MISSES * period + slack;
  if (lost < LOST_AFTER_MS) lost = LOST_AFTER_MS;
  trk.setTimeouts(period ? period + slack : LOST_AFTER_MS, lost);
}

void reportLink(uint32_t now) {
  bl::Link s = trk.poll(now);
  if (s != prevLink) {
    if (s == bl::Link::TRACKING) {
      if (prevLink == bl::Link::NO_CONTACT) Serial.printf(">>> LINK UP (RSSI %.0f dBm)\n", trk.rssi());
      else Serial.printf(">>> LINK %s after %lu ms outage, %lu missed (RSSI %.0f dBm)\n",
                         prevLink == bl::Link::LOST ? "RESTORED" : "RECOVERED",
                         now - outageStartMs, missStreak, trk.rssi());
      missStreak = 0;
    } else if (s == bl::Link::COASTING) {
      outageStartMs = trk.lastMs();
      Serial.printf(">>> LINK COASTING: no packet for %lu ms, holding last estimate\n", trk.ageMs(now));
    } else if (s == bl::Link::LOST) {
      if (prevLink == bl::Link::TRACKING) outageStartMs = trk.lastMs();
      histN = histHead = 0;   // trend history is stale now
      Serial.printf(">>> LINK LOST: no packet for %lu ms. Estimate INVALID until reacquired.\n", trk.ageMs(now));
    }
    prevLink = s; lastLinkMsg = now;
  } else if (now - lastLinkMsg >= 10000 && (s == bl::Link::LOST || s == bl::Link::NO_CONTACT)) {
    lastLinkMsg = now;
    Serial.printf(">>> %s for %lu s (%lu missed)\n", s == bl::Link::LOST ? "STILL LOST" : "NO CONTACT yet",
                  (now - (s == bl::Link::LOST ? outageStartMs : 0)) / 1000, missStreak);
  }
}

void scanFeed(float rssi);

void onGoodSample(uint16_t pktSeq, int rtt, int downDbm, float upDbm, uint32_t rxUs) {
  uint32_t now = millis();
  float avg = isnan(upDbm) ? downDbm : (downDbm + upDbm) / 2.0f;
  trk.measure(avg, now);
  reportLink(now);

  histT[histHead] = now / 1000.0f; histR[histHead] = trk.rssi();
  histHead = (histHead + 1) % 12; if (histN < 12) histN++;
  recent[recentHead] = avg; recentHead = (recentHead + 1) % 10; if (recentN < 10) recentN++;

  st.dSum += downDbm; st.dMin = (st.ok == 1) ? downDbm : min(st.dMin, downDbm); st.dMax = max(st.dMax, downDbm);
  if (!isnan(upDbm)) {
    int u = (int)upDbm; st.uSum += u; st.uCnt++;
    st.uMin = (st.uCnt == 1) ? u : min(st.uMin, u); st.uMax = max(st.uMax, u);
  }
  Serial.printf("DATA,%lu,%s,%u,OK,%d,%d,%s,%.1f,%.1f,%s,%lu,%lu,%lu,%lu,%.1f,%s,0\n",
                now, listenMode ? "listen" : "ping", pktSeq, rtt, downDbm,
                isnan(upDbm) ? "" : String((int)upDbm).c_str(), avg,
                estimateDistance(trk.rssi()), trend(),
                attemptCount(), st.ok, lostCount(), framer.bad, perPct(), bl::linkName(trk.state()));
  uint32_t proc = micros() - rxUs;
  st.procSumUs += proc; st.procN++; if (proc > st.procMaxUs) st.procMaxUs = proc;
  scanFeed(avg);
}

void onLoss(uint16_t pktSeq, const char *status, uint32_t count) {
  uint32_t now = millis();
  missStreak += count;
  reportLink(now);
  // distance column is left empty: a held/stale estimate is never printed as a measurement
  Serial.printf("DATA,%lu,%s,%u,%s,,,,,,%s,%lu,%lu,%lu,%lu,%.1f,%s,%lu\n",
                now, listenMode ? "listen" : "ping", pktSeq, status, trend(),
                attemptCount(), st.ok, lostCount(), framer.bad, perPct(),
                bl::linkName(trk.state()), trk.ageMs(now));
}

void printSummary() {
  Serial.printf("# ---- %s | attempts=%lu ok=%lu lost=%lu crc_bad=%lu  PER=%.1f%%  link=%s\n",
                listenMode ? "LISTEN" : "PING", attemptCount(), st.ok, lostCount(), framer.bad, perPct(),
                bl::linkName(trk.state()));
  if (!listenMode)
    Serial.printf("#      timing: on-air <= %lu ms (est.), ping every %lu ms (%.1f%% duty), RTO %lu ms\n",
                  sched.txMs(), sched.interval(), DUTY_CYCLE_PCT, sched.rto());
  else
    Serial.printf("#      timing: beacon period %lu ms\n", beaconPeriodMs);
  if (st.ok) {
    Serial.printf("#      RSSI down avg=%.1f min=%d max=%d dBm\n", (float)st.dSum / st.ok, st.dMin, st.dMax);
    if (st.uCnt)
      Serial.printf("#      RSSI up   avg=%.1f min=%d max=%d dBm\n", (float)st.uSum / st.uCnt, st.uMin, st.uMax);
    if (!listenMode && st.rttMin != UINT32_MAX)
      Serial.printf("#      RTT avg=%lu min=%lu max=%lu ms   late=%lu\n", st.rttSum / st.ok, st.rttMin, st.rttMax, st.late);
    if (!listenMode && st.haveBase)
      Serial.printf("#      lost on uplink=%lu, on downlink=%lu\n", st.upLost, st.downLost);
    Serial.printf("#      rx->DATA latency avg=%lu max=%lu us\n", st.procSumUs / st.procN, st.procMaxUs);
    if (trk.valid())
      Serial.printf("#      est. distance=%.1f m (RSSI %.1f +/- %.1f dB) trend=%s  (A=%.1f n=%.2f)\n",
                    estimateDistance(trk.rssi()), trk.rssi(), trk.sigma(millis()), trend(), modelA, modelN);
  }
}

// ------------------------------ direction scan ------------------------------
void scanPrompt() {
  Serial.printf("# SCAN %u/8: face %u deg (clockwise from start), antenna vertical at your chest, press ENTER\n",
                scanSector + 1, scanSector * 45);
}

void scanFinish() {
  scanActive = false;
  float mx = -999, mn = 999; uint8_t best = 0, worst = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (scanRes[i] > mx) { mx = scanRes[i]; best = i; }
    if (scanRes[i] < mn) { mn = scanRes[i]; worst = i; }
  }
  float sx = 0, sy = 0, floorMw = powf(10, mn / 10);
  for (uint8_t i = 0; i < 8; i++) {
    float w = powf(10, scanRes[i] / 10) - floorMw, a = i * 45 * DEG_TO_RAD;
    sx += w * cosf(a); sy += w * sinf(a);
  }
  float hdg = atan2f(sy, sx) * RAD_TO_DEG; if (hdg < 0) hdg += 360;
  for (uint8_t i = 0; i < 8; i++) Serial.printf("#   %3u deg : %6.1f dBm%s\n", i * 45, scanRes[i], i == best ? "  <- strongest" : (i == worst ? "  <- weakest" : ""));
  Serial.printf("SCAN,%lu,%.0f,%u,%u,%.1f\n", millis(), hdg, best * 45, worst * 45, mx - mn);
  if (mx - mn < 4.0f) Serial.println(F("# Spread < 4 dB: direction INCONCLUSIVE."));
  else Serial.printf("# Beacon most likely at ~%.0f deg (strongest %u deg; body shadow ~%u deg).\n",
                     hdg, best * 45, (worst * 45 + 180) % 360);
}

void scanFeed(float rssi) {
  if (!scanActive || !scanArmed) return;
  scanSum += rssi;
  if (++scanCount < SCAN_SAMPLES) return;
  scanRes[scanSector] = scanSum / scanCount;
  Serial.printf("#   %u deg -> %.1f dBm\n", scanSector * 45, scanRes[scanSector]);
  scanArmed = false;
  if (++scanSector >= 8) scanFinish(); else scanPrompt();
}

// ------------------------------- radio -------------------------------------
bool configureRadio() {
  ResponseStructContainer c = e220.getConfiguration();
  if (c.status.code != E220_SUCCESS) {
    // no c.close(): on early failure the library leaves c.data uninitialised (free() would crash)
    Serial.print(F("getConfiguration failed: code ")); Serial.print((int)c.status.code);
    Serial.print(F(" ")); Serial.println(c.status.getResponseDescription());
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
  cfg.TRANSMISSION_MODE.enableRSSI        = RSSI_ENABLED;
  cfg.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  cfg.TRANSMISSION_MODE.enableLBT         = LBT_DISABLED;
  cfg.TRANSMISSION_MODE.WORPeriod         = WOR_2000_011;

  e220.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
  // verify by reading back (the save reply is sometimes misparsed by the library)
  c = e220.getConfiguration();
  Configuration *v = (Configuration *)c.data;
  bool ok = c.status.code == E220_SUCCESS && v->CHAN == RADIO_CHANNEL &&
            v->SPED.airDataRate == RADIO_AIR_RATE && v->TRANSMISSION_MODE.enableRSSI == RSSI_ENABLED;
  if (c.status.code != E220_SUCCESS) c.data = nullptr;   // see above: never free an uninitialised pointer
  if (ok)
    Serial.printf("# radio: ch=%u (%s) air=%s power=%s uart=%s\n", v->CHAN, v->getChannelDescription().c_str(),
                  v->SPED.getAirDataRateDescription().c_str(), v->OPTION.getTransmissionPowerDescription().c_str(),
                  v->SPED.getUARTBaudRateDescription().c_str());
  c.close();
  return ok;
}

// Non-blocking TX: refuses while the module is busy (AUX low); never waits.
bool txPacket(Packet &p) {
  if (txPending || digitalRead(PIN_AUX) == LOW) return false;
  bl::seal(p, NET_ID);
  E220_UART.write((const uint8_t *)&p, sizeof(p));
  txPending = true; txSawLow = false; txStartMs = millis();
  return true;
}

// Clears txPending once the module has taken the bytes (AUX low..high), or after 50 ms
// (at 115200 the AUX pulse can be shorter than one loop pass).
void pollTxDone(uint32_t now) {
  if (!txPending) return;
  bool aux = digitalRead(PIN_AUX);
  if (!aux) txSawLow = true;
  if ((txSawLow && aux) || now - txStartMs > 50) txPending = false;
}

// ---- beacon role: answer every PING; if the module is busy, send as soon as it is free ----
uint16_t beaconPings = 0;
Packet pendingPong; bool pongQueued = false;
void beaconHandle(const Packet &p, uint8_t rssiRaw) {
  if (p.type != bl::PKT_PING) return;
  beaconPings++;
  pendingPong = p;
  pendingPong.type = bl::PKT_PONG;
  pendingPong.rssiUp = rssiRaw;
  pendingPong.beaconRx = beaconPings;
  pongQueued = !txPacket(pendingPong);
  Serial.printf("PING seq=%u rssi=%d dBm rx=%u\n", p.seq, bl::rssiDbm(rssiRaw), beaconPings);
}

void handlePacket(const Packet &p, uint8_t rssiRaw, uint32_t rxUs) {
#if ROLE == ROLE_BEACON
  beaconHandle(p, rssiRaw);
  return;
#endif
  uint32_t now = millis();
  int down = bl::rssiDbm(rssiRaw);

  if (!listenMode && p.type == bl::PKT_PONG) {
    int32_t rtt = sched.pong(p.seq, now);
    if (rtt < 0) { st.late++; return; }
    st.ok++;
    st.rttSum += rtt; st.rttMin = min(st.rttMin, (uint32_t)rtt); st.rttMax = max(st.rttMax, (uint32_t)rtt);
    if (!st.haveBase || p.beaconRx < st.beaconBase) {         // first pong or beacon rebooted
      st.haveBase = true; st.sentBase = st.sent; st.beaconBase = p.beaconRx; st.okBase = st.ok;
      st.upLost = st.downLost = 0;
    } else {
      uint32_t delivered = (uint16_t)(p.beaconRx - st.beaconBase);
      uint32_t sentSince = st.sent - st.sentBase, okSince = st.ok - st.okBase;
      st.upLost   = sentSince > delivered ? sentSince - delivered : 0;
      st.downLost = delivered > okSince ? delivered - okSince : 0;
    }
    applyLinkTimeouts();
    onGoodSample(p.seq, rtt, down, (float)bl::rssiDbm(p.rssiUp), rxUs);
  } else if (listenMode && p.type == bl::PKT_BEACON) {
    uint16_t gap = 0;
    if (st.haveBeaconSeq) {
      uint16_t d = p.seq - st.lastBeaconSeq;          // 16-bit wrap-safe
      if (d > 1 && d < 1000) gap = d - 1;             // larger jump = beacon rebooted: resync
      if (gap && beaconPeriodMs == 0) beaconPeriodMs = (now - lastBeaconRxMs) / (gap + 1);
      else if (!gap && lastBeaconRxMs) {
        uint32_t dt = now - lastBeaconRxMs;
        beaconPeriodMs = beaconPeriodMs ? (beaconPeriodMs * 7 + dt) / 8 : dt;
      }
    }
    st.haveBeaconSeq = true; st.lastBeaconSeq = p.seq; lastBeaconRxMs = now;
    if (gap) { st.lost += gap; onLoss(p.seq - 1, "LOST", gap); }
    st.ok++;
    applyLinkTimeouts();
    onGoodSample(p.seq, 0, down, NAN, rxUs);
  } else {
    return;
  }
  if ((st.ok + st.timeouts + st.lost) % SUMMARY_EVERY == 0) printSummary();
}

// ------------------------------- commands ----------------------------------
void printHelp() {
  Serial.println(F("# commands: mode ping | mode listen | cal <meters> | calclear | n <exp> |"
                   " mark <text> | scan | stats | reset | help"));
}

void handleCommand(char *line) {
  while (*line == ' ') line++;
  if (scanActive && *line == '\0') {
    scanArmed = true; scanSum = 0; scanCount = 0;
    Serial.printf("#   sampling %u deg ... hold still\n", scanSector * 45);
    return;
  }
  if (*line == '\0') return;

  if (!strcmp(line, "help")) printHelp();
  else if (!strcmp(line, "stats")) printSummary();
  else if (!strcmp(line, "reset")) { resetStats(); Serial.println(F("# stats reset")); }
  else if (!strcmp(line, "mode ping")) { listenMode = false; resetStats(); applyLinkTimeouts(); Serial.println(F("# mode PING (UNO BEACON_MODE 0)")); }
  else if (!strcmp(line, "mode listen")) { listenMode = true; beaconPeriodMs = 0; resetStats(); applyLinkTimeouts(); Serial.println(F("# mode LISTEN (UNO BEACON_MODE 1)")); }
  else if (!strncmp(line, "mark", 4)) Serial.printf("MARK,%lu,%s\n", millis(), line + 4 + (line[4] == ' '));
  else if (!strcmp(line, "calclear")) { calCount = 0; modelA = -20.0f; modelN = 2.5f; saveModel(); Serial.println(F("# calibration cleared")); }
  else if (!strncmp(line, "cal ", 4)) {
    float d = atof(line + 4);
    if (d <= 0.5f) { Serial.println(F("# cal: give distance in meters > 0.5")); return; }
    if (recentN < 5) { Serial.println(F("# cal: need at least 5 good packets first")); return; }
    float s = 0; for (uint8_t i = 0; i < recentN; i++) s += recent[i];
    if (calCount < 8) cal[calCount++] = { d, s / recentN };
    Serial.printf("CAL,%lu,%.1f,%.1f\n", millis(), d, s / recentN);
    fitModel();
  }
  else if (!strncmp(line, "n ", 2)) { modelN = constrain(atof(line + 2), 1.5f, 6.0f); saveModel(); Serial.printf("# n=%.2f\n", modelN); }
  else if (!strcmp(line, "scan")) {
    scanActive = true; scanArmed = false; scanSector = 0;
    Serial.println(F("# Direction scan: turn in 45 deg steps; your body shields the omni antenna."));
    scanPrompt();
  }
  else Serial.printf("# unknown command '%s' (type help)\n", line);
}

void pollCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
}

// --------------------------------------------------------------------------
void setup() {
  Serial.setTxBufferSize(4096);    // log output never blocks the radio loop
  Serial.begin(115200);
  delay(1500);
  Serial.println(ROLE == ROLE_BEACON ? F("\n# E220 BEACON (ESP32)") : F("\n# E220 range tester (ESP32)"));
  Serial.printf("# pins: E220 TXD->GPIO%d  E220 RXD<-GPIO%d  AUX->GPIO%d  M0<-GPIO%d  M1<-GPIO%d\n",
                PIN_RX, PIN_TX, PIN_AUX, PIN_M0, PIN_M1);

  prefs.begin("e220rt", false);
  modelA = prefs.getFloat("A", modelA);
  modelN = prefs.getFloat("n", modelN);

  e220.begin();
  while (!configureRadio()) {
    Serial.println(F("# E220 config FAILED - check wiring/power. Retrying in 3 s"));
    delay(3000);
  }
  e220.setMode(MODE_0_NORMAL);
  E220_UART.updateBaudRate(MODULE_UART_BAUD);
  while (E220_UART.available()) E220_UART.read();

  resetStats();
  applyLinkTimeouts();
  if (ROLE == ROLE_BEACON) return;
  Serial.println(F("CSV,t_ms,mode,seq,status,rtt_ms,rssi_down_dbm,rssi_up_dbm,rssi_avg_dbm,dist_m,trend,attempts,ok,lost,bad,per_pct,link,age_ms"));
  printHelp();
}

void loop() {
  uint32_t now = millis();

  while (E220_UART.available()) {
    Packet p; uint8_t rssi;
    if (framer.push(E220_UART.read(), now, p, rssi)) handlePacket(p, rssi, micros());
  }

  pollTxDone(now);
  if (ROLE == ROLE_BEACON) {
    if (pongQueued) pongQueued = !txPacket(pendingPong);
    return;
  }

  pollCommands();
  reportLink(now);
  if (listenMode) return;

  if (sched.timedOut(now)) {
    st.timeouts++;
    onLoss(sched.seq(), "TIMEOUT", 1);
    if ((st.ok + st.timeouts) % SUMMARY_EVERY == 0) printSummary();
  }
  if (sched.due(now) && !txPending && digitalRead(PIN_AUX) == HIGH) {
    Packet p = {};
    p.type = bl::PKT_PING;
    p.seq = sched.send(now);
    if (txPacket(p)) st.sent++;
  }
}
