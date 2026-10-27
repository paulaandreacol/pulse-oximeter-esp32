// PulseOxDSP.h - signal processing for the ESP32 pulse oximeter.
//
// Hardware-free on purpose: the .ino drives the LEDs and ADC, this file turns
// the red / IR / ambient readings into pulse waveforms, heart rate, SpO2,
// perfusion index and a status flag. Because it has no Arduino dependencies
// it can also be compiled and tested on a PC (see host_test/).
//
// Filter corners match the LTspice analog chain (HPF 0.5 Hz, LPF 5 Hz), but
// they are applied digitally to each wavelength separately, because the LEDs
// are time-multiplexed and the analog chain cannot separate them.
#pragma once
#include <math.h>
#include <stdint.h>

namespace pox {

// ---------------------------------------------------------------------------
// 2nd-order IIR section (RBJ cookbook, transposed direct form II)
// ---------------------------------------------------------------------------
class Biquad {
 public:
  void lowpass(float fs, float fc, float q = 0.70710678f) { design(fs, fc, q, false); }
  void highpass(float fs, float fc, float q = 0.70710678f) { design(fs, fc, q, true); }

  float step(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }

  // Start in steady state for a constant input x, so the filter does not ring
  // when a finger is first inserted.
  void reset(float x) {
    float y = x * (b0 + b1 + b2) / (1.0f + a1 + a2);
    z1 = y - b0 * x;
    z2 = b2 * x - a2 * y;
  }

 private:
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;

  void design(float fs, float fc, float q, bool high) {
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float c = cosf(w0), alpha = sinf(w0) / (2.0f * q), a0 = 1.0f + alpha;
    if (high) {
      b0 = (1.0f + c) / 2.0f / a0;
      b1 = -(1.0f + c) / a0;
    } else {
      b0 = (1.0f - c) / 2.0f / a0;
      b1 = (1.0f - c) / a0;
    }
    b2 = b0;
    a1 = -2.0f * c / a0;
    a2 = (1.0f - alpha) / a0;
  }
};

// One wavelength: DC level (for the SpO2 ratio) and band-passed pulse (AC).
class Channel {
 public:
  void setup(float fs, float hpHz, float lpHz, float dcHz) {
    hp1.highpass(fs, hpHz);
    lp1.lowpass(fs, lpHz);
    dcAlpha = 1.0f - expf(-2.0f * (float)M_PI * dcHz / fs);
  }
  void reset(float x) {
    hp1.reset(x);
    lp1.reset(0.0f);
    dc = x;
  }
  float step(float x) {
    dc += dcAlpha * (x - dc);
    ac = lp1.step(hp1.step(x));
    return ac;
  }
  float dc = 0, ac = 0;

 private:
  Biquad hp1, lp1;
  float dcAlpha = 0.01f;
};

// Small fixed-size history used for median / spread of per-beat values.
template <int N>
class History {
 public:
  void push(float v) {
    buf[head] = v;
    head = (head + 1) % N;
    if (count < N) count++;
  }
  void clear() { count = head = 0; }
  int size() const { return count; }
  float median() const {
    if (count == 0) return NAN;
    float s[N];
    for (int i = 0; i < count; i++) s[i] = buf[i];
    for (int i = 1; i < count; i++)  // insertion sort, N is tiny
      for (int j = i; j > 0 && s[j - 1] > s[j]; j--) {
        float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
      }
    return (count & 1) ? s[count / 2] : 0.5f * (s[count / 2 - 1] + s[count / 2]);
  }
  // Robust spread relative to the median (1.4826 * MAD / median, which equals
  // std/mean for normally distributed data). One bad beat barely moves it.
  float spread() const {
    if (count < 4) return 0;
    float med = median();
    History<N> dev;
    for (int i = 0; i < count; i++) dev.push(fabsf(buf[i] - med));
    return 1.4826f * dev.median() / med;
  }

 private:
  float buf[N] = {};
  int head = 0, count = 0;
};

// ---------------------------------------------------------------------------
// Status flags - a small hand-built decision tree (brief: "identify anomalous
// measurements"). Thresholds are screening limits, not clinical diagnosis.
// ---------------------------------------------------------------------------
enum class Status : uint8_t {
  Saturated,     // ADC near full scale: lower LED current or detector gain
  NoFinger,
  Searching,     // finger present, waiting for enough clean beats
  PoorSignal,    // red and IR pulses disagree, or perfusion too low (motion, noise)
  Irregular,     // beat intervals vary > 15 %: motion or arrhythmia, numbers unreliable
  SpO2Critical,  // < 90 %
  SpO2Low,       // 90-94 %
  Tachycardia,   // > 100 bpm
  Bradycardia,   // < 50 bpm
  Normal
};

inline const char* statusText(Status s) {
  switch (s) {
    case Status::Saturated:    return "SATURATED";
    case Status::NoFinger:     return "NO_FINGER";
    case Status::Searching:    return "SEARCHING";
    case Status::PoorSignal:   return "POOR_SIGNAL";
    case Status::SpO2Critical: return "SPO2_CRITICAL";
    case Status::SpO2Low:      return "SPO2_LOW";
    case Status::Tachycardia:  return "TACHYCARDIA";
    case Status::Bradycardia:  return "BRADYCARDIA";
    case Status::Irregular:    return "IRREGULAR";
    default:                   return "NORMAL";
  }
}

// ---------------------------------------------------------------------------
// Heartbeat detector for a band-passed pulse waveform (systole pointing up).
// Used on the digitally filtered IR pulse, and separately on the output of the
// op-amp filter chain (AMPLIFIED_OUT), so the two can be compared.
//
// Slope sum function (Zong et al., 2003): total of the positive slopes over
// the last ~128 ms. It peaks on the steep systolic upstroke and stays small on
// the dicrotic wave and on noise, so it double-counts far less than plain peak
// picking. A beat is the slope sum rising through a fraction of the typical
// beat's slope-sum peak (median of the last 5), at least one refractory period
// (60/maxBpm) after the previous beat. A decaying envelope is used instead at
// start-up, and whenever beats are overdue (e.g. the signal got weaker).
// ---------------------------------------------------------------------------
class BeatDetector {
 public:
  void setup(float fs_, float detectHz, float minBpm_, float maxBpm, float threshold_) {
    fs = fs_;
    minBpm = minBpm_;
    threshold = threshold_;
    lp.lowpass(fs, detectHz);
    envDecay = expf(-logf(2.0f) / (1.5f * fs));  // envelope half-life 1.5 s
    refractory = (uint32_t)(fs * 60.0f / maxBpm);
    len = (int)lroundf(0.128f * fs);  // 128 ms window
    if (len < 2) len = 2;
    if (len > kMax) len = kMax;
    reset();
  }

  // Start over, e.g. when a finger is inserted. x0 = current input level.
  void reset(float x0 = 0) {
    intervals.clear();
    peaks.clear();
    n = lastBeatN = 0;
    lastBeatPos = env = prevS = beatPeak = 0;
    lp.reset(x0);
    prevX = x0;
    for (int k = 0; k < kMax; k++) buf[k] = 0;
    idx = 0;
    intervalValid = false;
  }

  // Feed a sample without detecting (while upstream filters settle).
  void prime(float x) {
    n++;
    prevS = slopeSum(x);
  }

  // Feed a sample. Returns true when a beat starts on this sample; then
  // intervalValid says whether a new beat interval was recorded.
  bool update(float x) {
    n++;
    float s = slopeSum(x);
    env = fmaxf(env * envDecay, s);
    beatPeak = fmaxf(beatPeak, s);
    float ref = env;
    if (peaks.size() >= 2) {
      float overdue = 1.5f * intervals.median() * fs;
      if (lastBeatN != 0 && (n - lastBeatN) < overdue) ref = peaks.median();
    }
    float thr = threshold * ref;
    bool rising = s >= thr && prevS < thr && s > 0;
    float before = prevS;
    prevS = s;
    intervalValid = false;
    if (!rising || (lastBeatN != 0 && (n - lastBeatN) < refractory)) return false;

    // interpolate where between the last two samples the threshold was crossed
    float pos = (float)(n - 1) + (thr - before) / (s - before);
    if (lastBeatN != 0) {
      float interval = (pos - lastBeatPos) / fs;
      if (interval <= 60.0f / minBpm) {
        intervals.push(interval);
        peaks.push(beatPeak);
        intervalValid = true;
      }
    }
    lastBeatN = n;
    lastBeatPos = pos;
    beatPeak = s;
    return true;
  }

  int count() const { return intervals.size(); }
  float bpm() const { return 60.0f / intervals.median(); }  // median of last 8 intervals
  float spread() const { return intervals.spread(); }
  bool recent(float seconds) const {
    return lastBeatN != 0 && (n - lastBeatN) < (uint32_t)(fs * seconds);
  }
  bool intervalValid = false;

 private:
  static const int kMax = 32;
  Biquad lp;  // extra smoothing for detection only
  History<8> intervals;
  History<5> peaks;
  float fs = 50, minBpm = 30, threshold = 0.5f, envDecay = 0.99f;
  uint32_t n = 0, lastBeatN = 0, refractory = 0;
  float lastBeatPos = 0, env = 0, prevX = 0, prevS = 0, beatPeak = 0;
  float buf[kMax] = {};
  int len = 6, idx = 0;

  float slopeSum(float x) {
    float y = lp.step(x);
    float d = y - prevX;
    prevX = y;
    buf[idx] = d > 0 ? d : 0;
    idx = (idx + 1) % len;
    float s = 0;
    for (int k = 0; k < len; k++) s += buf[k];
    return s;
  }
};

struct Config {
  float fs = 50.0f;           // samples per second handed to update()
  float hpHz = 0.5f;          // pulse band, same corners as the LTspice design
  float lpHz = 5.0f;
  float dcHz = 0.2f;          // DC tracker for the AC/DC ratios
  float saturatedMv = 3000;   // ESP32 ADC at 11 dB tops out around 3.1 V
  float fingerMinMv = 30;     // valid IR DC window (ambient-subtracted); tune
  float fingerMaxMv = 2900;   //   by watching ir_mV with and without a finger
  float minBpm = 30, maxBpm = 200;
  float beatThreshold = 0.5f; // slope-sum must exceed this fraction of its recent peak
  float detectHz = 3.0f;      // extra smoothing for the beat detector only (<= 200 bpm)
  float spo2A = 110.0f;       // SpO2 = A - B*R  (textbook default, calibrate!)
  float spo2B = 25.0f;
  bool invert = true;         // detector output rises with light -> flip so systole is up
};

struct Reading {
  bool valid = false;
  float bpm = NAN, spo2 = NAN, pi = NAN, ratio = NAN;
  float rrCv = 0;     // beat-to-beat interval spread (0.05 = 5 %)
  float quality = 0;  // red/IR pulse correlation, ~1 for a clean signal
  Status status = Status::NoFinger;
};

class PulseOx {
 public:
  explicit PulseOx(const Config& c = Config()) : cfg(c) {
    red.setup(cfg.fs, cfg.hpHz, cfg.lpHz, cfg.dcHz);
    ir.setup(cfg.fs, cfg.hpHz, cfg.lpHz, cfg.dcHz);
    beats.setup(cfg.fs, cfg.detectHz, cfg.minBpm, cfg.maxBpm, cfg.beatThreshold);
  }

  // Feed one sample of each LED phase (all in mV straight from the ADC).
  // Returns true if a heartbeat was detected on this sample.
  bool update(float redMv, float irMv, float ambientMv) {
    n++;
    saturated = redMv > cfg.saturatedMv || irMv > cfg.saturatedMv;
    float r = redMv - ambientMv, i = irMv - ambientMv;
    lastRed = r;
    lastIr = i;

    bool fingerNow = !saturated && i > cfg.fingerMinMv && i < cfg.fingerMaxMv;
    // debounce: finger state must hold for 0.5 s before it changes
    fingerCount = (fingerNow == finger) ? 0 : fingerCount + 1;
    if (fingerCount > cfg.fs * 0.5f) {
      finger = fingerNow;
      fingerCount = 0;
      if (finger) restart(r, i);
    }
    if (!finger) {
      redPulse = irPulse = 0;
      return false;
    }

    float sign = cfg.invert ? -1.0f : 1.0f;
    redPulse = sign * red.step(r);
    irPulse = sign * ir.step(i);
    if (n - startN < (uint32_t)(cfg.fs * 2)) {  // let the filters settle first
      beats.prime(irPulse);
      return false;
    }
    accumulate();
    if (!beats.update(irPulse)) return false;

    if (beats.intervalValid) {
      // R = (ACred/DCred) / (ACir/DCir). AC ratio from the least-squares slope
      // of red vs IR over the beat: uncorrelated noise averages out, unlike
      // peak-to-trough which noise inflates (more so on the weaker red).
      float covRI = sRI / sN - (sR / sN) * (sI / sN);
      float varI = sII / sN - (sI / sN) * (sI / sN);
      float varR = sRR / sN - (sR / sN) * (sR / sN);
      if (varI > 0 && varR > 0 && covRI > 0 && ir.dc > 0 && red.dc > 0) {
        ratios.push((covRI / varI) * (ir.dc / red.dc));
        perfusion.push(100.0f * (irMax - irMin) / ir.dc);
      }
      // Both wavelengths see the same heartbeat, so their pulses should be
      // almost perfectly correlated; noise and motion break that.
      correlation.push(varI > 0 && varR > 0 ? covRI / sqrtf(varI * varR) : 0);
    }
    resetBeatStats();
    return true;
  }

  Reading reading() const {
    Reading out;
    out.rrCv = beats.spread();
    if (saturated) { out.status = Status::Saturated; return out; }
    if (!finger) { out.status = Status::NoFinger; return out; }
    if (!beats.recent(3.0f) || beats.count() < 2 || ratios.size() < 2) {
      out.status = Status::Searching;
      return out;
    }
    out.valid = true;
    out.bpm = beats.bpm();
    out.ratio = ratios.median();
    out.spo2 = fminf(100.0f, fmaxf(0.0f, cfg.spo2A - cfg.spo2B * out.ratio));
    out.pi = perfusion.median();
    out.quality = correlation.median();
    out.status = classify(out);
    return out;
  }

  bool fingerPresent() const { return finger && !saturated; }

  // Latest values, for logging / plotting
  float redPulse = 0, irPulse = 0;   // band-passed pulse waveforms (mV)
  float lastRed = 0, lastIr = 0;     // ambient-subtracted levels (mV)
  float redDc() const { return red.dc; }
  float irDc() const { return ir.dc; }
  const Config& config() const { return cfg; }

 private:
  Config cfg;
  Channel red, ir;
  BeatDetector beats;
  History<5> ratios, perfusion, correlation;
  uint32_t n = 0, startN = 0;
  int fingerCount = 0;
  bool finger = false, saturated = false;

  // per-beat accumulators: IR peak/trough (perfusion index) and the sums for
  // a least-squares fit of red AC against IR AC (ratio of ratios)
  float irMax = 0, irMin = 0, sR = 0, sI = 0, sRI = 0, sII = 0, sRR = 0;
  int sN = 0;

  void restart(float r, float i) {
    red.reset(r);
    ir.reset(i);
    beats.reset();
    ratios.clear();
    perfusion.clear();
    correlation.clear();
    startN = n;
    resetBeatStats();
  }

  void resetBeatStats() {
    irMax = -1e9f;
    irMin = 1e9f;
    sR = sI = sRI = sII = sRR = 0;
    sN = 0;
  }

  void accumulate() {
    irMax = fmaxf(irMax, irPulse);
    irMin = fminf(irMin, irPulse);
    sR += redPulse;
    sI += irPulse;
    sRI += redPulse * irPulse;
    sII += irPulse * irPulse;
    sRR += redPulse * redPulse;
    sN++;
  }

  static Status classify(const Reading& r) {
    // signal-quality checks first: no point judging SpO2 on a bad signal
    if (r.quality < 0.7f || r.pi < 0.05f || r.pi > 20.0f) return Status::PoorSignal;
    if (r.rrCv > 0.15f) return Status::Irregular;
    if (r.spo2 < 90.0f) return Status::SpO2Critical;
    if (r.spo2 < 95.0f) return Status::SpO2Low;
    if (r.bpm > 100.0f) return Status::Tachycardia;
    if (r.bpm < 50.0f) return Status::Bradycardia;  // textbook 60; fit students sit lower
    return Status::Normal;
  }
};

}  // namespace pox
