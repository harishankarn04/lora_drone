// Host simulation of the ping/pong link using the firmware's own beacon_link.h.
// Injects repeated dropouts and measures loss detection, reacquisition, stale-estimate
// safety and duty cycle. Build/run: tools/sim/run_sim.sh
#include "../../firmware/E220_Tester_ESP32/beacon_link.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <deque>
#include <random>
#include <algorithm>

struct Params {
  const char *name;
  uint32_t hopMs;      // ping write -> pong bytes fully at tester, one direction (UART+air+module+framing)
  uint32_t txMs;       // true on-air time of one packet (firmware only sees max(150, RTT/2))
  float dutyPct;       // 0 = old firmware (fixed 1500 ms interval)
  float randomLoss;    // background packet loss per hop
};

struct Outage { uint32_t start, end; };

struct Result {
  std::vector<uint32_t> toCoast, toLost, reacq;
  uint32_t staleViolations = 0, blendedAfterLost = 0, missedLost = 0;
  double testerDutyMaxPct = 0;
  uint32_t periodMs = 0, lostAfterMs = 0;
};

static uint32_t pct(std::vector<uint32_t> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, (size_t)(p * (v.size() - 1) + 0.5))];
}

// ---- model of the OLD firmware rules (fixed 1500 ms ping, 1300 ms timeout, LOST after 5 s, EMA) ----
struct OldLink {
  uint32_t lastGood = 0; bool everUp = false, lost = false; float ema = NAN;
  bool valid(uint32_t now) const { return everUp && now - lastGood <= 5000; }
};

Result run(const Params &P, const std::vector<Outage> &outs, uint32_t simMs, uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> U(0, 1);
  std::normal_distribution<float> fade(0, 3.0f);
  auto inOutage = [&](uint32_t t) { for (auto &o : outs) if (t >= o.start && t < o.end) return true; return false; };

  const bool old = P.dutyPct == 0;
  bl::PingScheduler sched({old ? 100.0f : P.dutyPct, old ? 1500u : 100u, old ? P.txMs : 150u,
                           old ? 1300u : 1500u, old ? 1300u : 50u, old ? 1300u : 3000u});
  bl::Tracker trk(4.0f, 9.0f);
  OldLink ol;
  Result R;

  struct Ev { uint32_t t; uint16_t seq; float rssi; };
  std::deque<Ev> pongs;                 // pongs in flight to the tester
  std::deque<uint32_t> txLog;           // tester TX start times (for duty cycle)
  uint32_t txDoneAt = 0; bool txBusy = false;
  bl::Link prev = bl::Link::NO_CONTACT;
  bool oldPrevValid = false;
  size_t oi = 0; bool waitingReacq = false; uint32_t reacqFrom = 0;
  bool sawCoast = false, sawLost = false; uint32_t outStart = 0;
  float trueRssi = -95;

  auto applyTimeouts = [&]() {           // mirrors applyLinkTimeouts() in the sketch
    uint32_t period = sched.interval(), slack = sched.rto();
    uint32_t lost = 3 * period + slack; if (lost < 3000) lost = 3000;
    trk.setTimeouts(period + slack, lost);
    R.periodMs = period; R.lostAfterMs = lost;
  };
  applyTimeouts();

  for (uint32_t t = 0; t < simMs; t++) {
    bool active = oi < outs.size() && t >= outs[oi].start && t < outs[oi].end;
    if (oi < outs.size() && t == outs[oi].start) { outStart = t; sawCoast = sawLost = false; }
    if (oi < outs.size() && t == outs[oi].end) { waitingReacq = true; reacqFrom = t; oi++; }

    // ---- tester TX ----
    if (txBusy && t >= txDoneAt) txBusy = false;   // P.txMs = true airtime (duty accounting only)
    sched.timedOut(t);
    if (!txBusy && sched.due(t)) {
      uint16_t seq = sched.send(t);
      txBusy = true; txDoneAt = t + P.txMs; txLog.push_back(t);
      uint32_t atBeacon = t + P.hopMs, back = atBeacon + P.hopMs;
      bool up = !inOutage(atBeacon) && U(rng) >= P.randomLoss;
      bool down = !inOutage(back) && U(rng) >= P.randomLoss;
      if (up && down) pongs.push_back({back, seq, trueRssi + fade(rng)});
    }
    while (!pongs.empty() && pongs.front().t <= t) {
      Ev e = pongs.front(); pongs.pop_front();
      if (sched.pong(e.seq, t) < 0) continue;
      bool wasLost = trk.state() == bl::Link::LOST;
      trk.measure(e.rssi, t);
      if (wasLost && trk.rssi() != e.rssi) R.blendedAfterLost++;
      applyTimeouts();
      ol.lastGood = t; ol.everUp = true;
      if (waitingReacq) { R.reacq.push_back(t - reacqFrom); waitingReacq = false; }
    }

    // ---- state + safety checks ----
    bl::Link s = trk.poll(t);
    bool valid = old ? ol.valid(t) : trk.valid();
    uint32_t age = old ? t - ol.lastGood : trk.ageMs(t);
    uint32_t lostAfter = old ? 5000 : R.lostAfterMs;
    if (valid && age > lostAfter + 1) R.staleViolations++;
    if (active) {
      if (!old && !sawCoast && s == bl::Link::COASTING) { R.toCoast.push_back(t - outStart); sawCoast = true; }
      bool lostNow = old ? (oldPrevValid && !valid) : (s == bl::Link::LOST && prev != bl::Link::LOST);
      if (!sawLost && lostNow) { R.toLost.push_back(t - outStart); sawLost = true; }
    }
    prev = s; oldPrevValid = valid;

    // ---- duty cycle over a sliding 1 h window (EN 300 220 observation period) ----
    while (!txLog.empty() && t - txLog.front() > 3600000) txLog.pop_front();
    if (t > 3600000 && t % 1000 == 0)
      R.testerDutyMaxPct = std::max(R.testerDutyMaxPct, 100.0 * txLog.size() * P.txMs / 3600000.0);
    trueRssi = -95 + 10 * sinf(t / 60000.0f);
  }
  return R;
}

int main(int argc, char **argv) {
  // Hop latency and on-air time: old = derived from the measured 909 ms RTT (32 B @ 9600);
  // new = 10 B @ 115200 assuming LoRa SF9/BW125-like airtime at the E220 "2.4k" setting
  // (Ebyte doesn't publish SF/BW). Override with measured values: link_sim <hopMs> <txMs>.
  uint32_t newHop = argc > 2 ? atoi(argv[1]) : 271, newTx = argc > 2 ? atoi(argv[2]) : 170;
  Params ps[] = {
    {"OLD (32B@9600, 1.5s fixed)", 454, 300, 0, 0.05f},
    {"NEW code, same 20% duty as OLD (isolates code gains)", newHop, newTx, 20.0f, 0.05f},
    {"NEW (10B@115200, 2.5% duty)", newHop, newTx, 2.5f, 0.05f},
  };
  std::vector<uint32_t> durs = {300, 1000, 2000, 5000, 10000, 30000, 60000};
  std::mt19937 rng(42);
  std::vector<Outage> outs; uint32_t t = 120000;
  for (int rep = 0; rep < 8; rep++)
    for (uint32_t d : durs) { t += 60000 + rng() % 30000; outs.push_back({t, t + d}); t += d; }
  uint32_t simMs = std::max<uint32_t>(t + 120000, 3700000);

  printf("%zu outages (%s ms x8, random phase) + 5%% random loss per hop, %.1f h simulated\n\n",
         outs.size(), "300/1000/2000/5000/10000/30000/60000", simMs / 3.6e6);
  for (auto &P : ps) {
    Result R = run(P, outs, simMs, 7);
    printf("== %s\n", P.name);
    if (P.dutyPct) printf("   update period %u ms, LOST after %u ms\n", R.periodMs, R.lostAfterMs);
    printf("   loss -> COASTING   p50 %5u  p95 %5u ms  (n=%zu)\n", pct(R.toCoast, .5), pct(R.toCoast, .95), R.toCoast.size());
    printf("   loss -> LOST       p50 %5u  p95 %5u ms  (n=%zu)\n", pct(R.toLost, .5), pct(R.toLost, .95), R.toLost.size());
    printf("   reacquisition      p50 %5u  p95 %5u  max %5u ms  (n=%zu)\n", pct(R.reacq, .5), pct(R.reacq, .95), pct(R.reacq, 1), R.reacq.size());
    printf("   stale-valid ms: %u   blended-after-LOST: %u   max 1h duty (tester): %.2f%%\n\n",
           R.staleViolations, R.blendedAfterLost, R.testerDutyMaxPct);
  }

  // ---- framer: garbage, split frames, corrupted CRC ----
  bl::Framer f(0x42, 3);
  bl::Packet p = {}; p.type = bl::PKT_PONG; p.seq = 7; bl::seal(p, 0x42);
  std::vector<uint8_t> frame((uint8_t *)&p, (uint8_t *)&p + sizeof(p)); frame.push_back(0xA0);
  std::vector<uint8_t> stream = {0x00, 0xB3, 0x11, 0xFF};           // garbage incl. a false magic
  stream.insert(stream.end(), frame.begin(), frame.end());
  auto bad = frame; bad[4] ^= 0x01; stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), frame.begin(), frame.end());
  int got = 0; bl::Packet o; uint8_t r;
  for (uint8_t b : stream) if (f.push(b, 0, o, r)) got++;
  int gotAfterGap = 0;                                               // partial frame then a long gap
  for (size_t i = 0; i < 5; i++) f.push(frame[i], 100, o, r);
  for (size_t i = 0; i < frame.size(); i++) if (f.push(frame[i], 200, o, r)) gotAfterGap++;
  printf("framer: %d/2 good frames recovered around garbage + 1 corrupted (crc_bad=%u); "
         "after stale partial: %d/1\n", got, f.bad, gotAfterGap);
  return 0;
}
