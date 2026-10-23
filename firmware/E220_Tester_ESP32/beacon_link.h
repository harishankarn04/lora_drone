// Beacon link logic: packet format, framing, ping scheduling and link tracking.
// Plain C++ (no Arduino calls) so the same code runs on the ESP32 and in tools/sim.
// All times are milliseconds from a caller-supplied clock.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace bl {

// ---------------- packet (10 bytes; the E220 appends 1 RSSI byte) ----------------
constexpr uint8_t MAGIC = 0xB3;
enum : uint8_t { PKT_PING = 1, PKT_PONG = 2, PKT_BEACON = 3 };

struct __attribute__((packed)) Packet {
  uint8_t  magic;
  uint8_t  netId;
  uint8_t  type;
  uint16_t seq;
  uint8_t  rssiUp;     // PONG: raw RSSI byte the beacon measured on the PING
  uint16_t beaconRx;   // PONG: PINGs the beacon has received since boot
  uint16_t crc;        // CRC16-CCITT over the bytes above
};
static_assert(sizeof(Packet) == 10, "packet must be 10 bytes");
constexpr size_t FRAME_LEN = sizeof(Packet) + 1;

inline uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}
inline void seal(Packet &p, uint8_t netId) {
  p.magic = MAGIC; p.netId = netId;
  p.crc = crc16((const uint8_t *)&p, sizeof(p) - 2);
}
inline int rssiDbm(uint8_t raw) { return -(256 - (int)raw); }

// ---------------- framer ----------------
// Emits a frame the moment its last byte arrives (no idle-gap wait). Resyncs on the
// magic + netId bytes; a partial frame older than gapMs is dropped as garbage.
class Framer {
 public:
  Framer(uint8_t netId, uint32_t gapMs) : net_(netId), gap_(gapMs) {}
  bool push(uint8_t b, uint32_t nowMs, Packet &out, uint8_t &rssiRaw) {
    if (len_ && nowMs - last_ > gap_) len_ = 0;
    last_ = nowMs;
    buf_[len_++] = b;
    while (len_) {
      if (buf_[0] != MAGIC || (len_ > 1 && buf_[1] != net_)) { drop(1); continue; }
      if (len_ < FRAME_LEN) return false;
      if (crc16(buf_, sizeof(Packet) - 2) != (uint16_t)(buf_[8] | buf_[9] << 8)) { bad++; drop(1); continue; }
      memcpy(&out, buf_, sizeof(Packet));
      rssiRaw = buf_[sizeof(Packet)];
      len_ = 0;
      return true;
    }
    return false;
  }
  uint32_t bad = 0;   // frames that failed CRC
 private:
  void drop(size_t n) { memmove(buf_, buf_ + n, len_ - n); len_ -= n; }
  uint8_t buf_[FRAME_LEN]; size_t len_ = 0; uint32_t last_ = 0;
  uint8_t net_; uint32_t gap_;
};

// ---------------- TX spacing from the duty-cycle budget ----------------
// Spacing so that on-air time / interval <= dutyPct. Constant rate, no bursts: the
// worst-case update period is fixed and known in advance.
// On-air time cannot be read from the E220: AUX is low only while the module takes in
// UART bytes, not during the RF transmission (measured 2026-10-18). So it is bounded from
// above: max(airtime floor, SRTT/2). SRTT/2 also contains UART and module time, so it
// over-estimates and the real duty cycle can only be lower than the target.
inline uint32_t dutyInterval(uint32_t txMs, float dutyPct, uint32_t minMs) {
  uint32_t d = (uint32_t)ceilf(txMs * 100.0f / dutyPct);
  return d > minMs ? d : minMs;
}

// ---------------- ping scheduler (tester) ----------------
// RTO per RFC 6298 (SRTT/RTTVAR, alpha 1/8, beta 1/4). No exponential backoff: this is
// one point-to-point link with fixed spacing, so there is no congestion to back off from
// and backing off would only delay reacquisition. Every ping has a unique seq, so RTT
// samples are never ambiguous (Karn's problem does not arise).
class PingScheduler {
 public:
  struct Cfg { float dutyPct; uint32_t minIntervalMs, airtimeFloorMs, rtoInitMs, rtoMinMs, rtoMaxMs; };
  explicit PingScheduler(const Cfg &c) : c_(c), txMs_(c.airtimeFloorMs), rto_(c.rtoInitMs) {}

  bool due(uint32_t now) const { return !outstanding_ && (int32_t)(now - next_) >= 0; }
  uint16_t send(uint32_t now) {
    sent_ = now; outstanding_ = true;
    next_ = now + interval();
    return ++seq_;
  }
  // RTT in ms if this pong answers the outstanding ping, else -1 (late or duplicate).
  int32_t pong(uint16_t seq, uint32_t now) {
    if (!outstanding_ || seq != seq_) return -1;
    outstanding_ = false;
    uint32_t r = now - sent_;
    if (!haveRtt_) { srtt_ = r; rttvar_ = r / 2.0f; haveRtt_ = true; }
    else { rttvar_ = 0.75f * rttvar_ + 0.25f * fabsf(srtt_ - r); srtt_ = 0.875f * srtt_ + 0.125f * r; }
    uint32_t half = (uint32_t)(srtt_ / 2);
    txMs_ = half > c_.airtimeFloorMs ? half : c_.airtimeFloorMs;
    float rto = srtt_ + fmaxf(10.0f, 4.0f * rttvar_);
    rto_ = rto < c_.rtoMinMs ? c_.rtoMinMs : rto > c_.rtoMaxMs ? c_.rtoMaxMs : (uint32_t)rto;
    return (int32_t)r;
  }
  bool timedOut(uint32_t now) {
    if (outstanding_ && now - sent_ >= rto_) { outstanding_ = false; return true; }
    return false;
  }
  uint32_t interval() const { return dutyInterval(txMs_, c_.dutyPct, c_.minIntervalMs); }
  uint32_t rto() const { return rto_; }
  uint32_t txMs() const { return txMs_; }
  uint16_t seq() const { return seq_; }
 private:
  Cfg c_;
  uint32_t txMs_, rto_, sent_ = 0, next_ = 0;
  uint16_t seq_ = 0;
  bool outstanding_ = false, haveRtt_ = false;
  float srtt_ = 0, rttvar_ = 0;
};

// ---------------- link tracker ----------------
// 1-D Kalman filter on RSSI (random-walk model). While packets are missing the
// estimate is held and its variance grows (Sinopoli et al. 2004, intermittent
// observations). After LOST the filter is re-initialised from the first new
// measurement, never blended with the stale pre-outage value.
enum class Link : uint8_t { NO_CONTACT, TRACKING, COASTING, LOST };
inline const char *linkName(Link s) {
  return s == Link::TRACKING ? "TRACKING" : s == Link::COASTING ? "COASTING" : s == Link::LOST ? "LOST" : "NO_CONTACT";
}

class Tracker {
 public:
  Tracker(float qDb2PerS, float rDb2) : q_(qDb2PerS), r_(rDb2) {}
  // coastAfter: one expected packet missed. lostAfter: estimate no longer usable.
  void setTimeouts(uint32_t coastAfterMs, uint32_t lostAfterMs) { coast_ = coastAfterMs; lost_ = lostAfterMs; }

  void measure(float z, uint32_t now) {
    if (state_ == Link::NO_CONTACT || state_ == Link::LOST) { x_ = z; p_ = r_; }
    else {
      float p = p_ + q_ * (now - t_) / 1000.0f;
      float k = p / (p + r_);
      x_ += k * (z - x_); p_ = (1 - k) * p;
    }
    t_ = now; state_ = Link::TRACKING;
  }
  Link poll(uint32_t now) {
    uint32_t age = now - t_;
    if (state_ == Link::TRACKING && age > coast_) state_ = Link::COASTING;
    if ((state_ == Link::TRACKING || state_ == Link::COASTING) && age > lost_) state_ = Link::LOST;
    return state_;
  }
  Link state() const { return state_; }
  bool valid() const { return state_ == Link::TRACKING || state_ == Link::COASTING; }
  float rssi() const { return x_; }
  float sigma(uint32_t now) const { return sqrtf(p_ + q_ * (now - t_) / 1000.0f); }
  uint32_t ageMs(uint32_t now) const { return now - t_; }
  uint32_t lastMs() const { return t_; }
 private:
  float q_, r_, x_ = 0, p_ = 0;
  uint32_t t_ = 0, coast_ = 0, lost_ = 0;
  Link state_ = Link::NO_CONTACT;
};

}  // namespace bl
