// PC test for PulseOxDSP.h using synthetic red/IR signals.
// Pulse shape is the same PWL used in the LTspice source (Draft5.asc).
//   c++ -std=c++17 -O2 -I ../PulseOximeter_ESP32 sim_test.cpp -o sim_test && ./sim_test
#include <cstdio>
#include <random>
#include "PulseOxDSP.h"

// LTspice PWL: 0 1u, 0.08 1.02u, 0.15 1.005u, 0.22 1.01u, 0.4 1u, 0.833 1u
static float pulseShape(float phase) {  // phase 0..1 of a 0.833 s beat
  const float t[] = {0, 0.08f, 0.15f, 0.22f, 0.40f, 0.833f};
  const float v[] = {0, 1.0f, 0.25f, 0.5f, 0, 0};
  float x = phase * 0.833f;
  for (int k = 0; k < 5; k++)
    if (x <= t[k + 1]) return v[k] + (v[k + 1] - v[k]) * (x - t[k]) / (t[k + 1] - t[k]);
  return 0;
}

struct Case { const char* name; float bpm, ratio, piIr, noiseMv; bool mayBePoor; };

int run(const Case& c) {
  pox::Config cfg;
  pox::PulseOx ox(cfg);
  std::mt19937 rng(42);
  std::normal_distribution<float> noise(0, c.noiseMv);
  const float dcIr = 1500, dcRed = 1100, fs = cfg.fs;
  float acIr = dcIr * c.piIr / 100, acRed = dcRed * c.ratio * c.piIr / 100;
  float phase = 0;
  pox::Reading last;
  int beats = 0;
  for (int n = 0; n < (int)(fs * 30); n++) {
    float t = n / fs;
    float bpm = c.bpm * (1 + 0.03f * sinf(2 * M_PI * 0.1f * t));  // mild HRV
    phase += bpm / 60 / fs;
    if (phase >= 1) phase -= 1;
    float wander = 1 + 0.02f * sinf(2 * M_PI * 0.25f * t);       // breathing
    auto amb = [&](float dt) { return 150 + 40 * sinf(2 * M_PI * 100 * (t + dt)); };  // 50 Hz mains lights
    float red, ir, a;
    if (t < 1.0f) {  // no finger: detector saturated
      red = ir = 3100; a = amb(0.0018f);
    } else {
      red = amb(0) + wander * (dcRed - acRed * pulseShape(phase)) + noise(rng);
      ir = amb(0.0009f) + wander * (dcIr - acIr * pulseShape(phase)) + noise(rng);
      a = amb(0.0018f) + noise(rng);
    }
    beats += ox.update(red, ir, a);
    last = ox.reading();
  }
  float expSpo2 = cfg.spo2A - cfg.spo2B * c.ratio;
  bool accurate = last.valid && fabsf(last.bpm - c.bpm) < 0.06f * c.bpm &&
                  fabsf(last.spo2 - fminf(expSpo2, 100)) < 2.5f &&
                  fabsf(last.pi - c.piIr) < 0.5f * c.piIr;
  // An unusable signal is acceptable only if it is flagged, never shown as a
  // confident (wrong) number.
  bool flagged = last.status == pox::Status::PoorSignal || last.status == pox::Status::Irregular;
  bool ok = c.mayBePoor ? (accurate || flagged) : accurate;
  printf("%-26s expect HR %5.1f SpO2 %5.1f PI %.2f | got HR %5.1f SpO2 %5.1f PI %.2f R %.3f %-13s beats %2d  %s\n",
         c.name, c.bpm, expSpo2, c.piIr, last.bpm, last.spo2, last.pi, last.ratio,
         pox::statusText(last.status), beats, ok ? "PASS" : "FAIL");
  printf("%26s interval spread %.3f, red/IR correlation %.3f\n", "", last.rrCv, last.quality);
  return ok ? 0 : 1;
}

int main() {
  const Case cases[] = {
      {"resting, healthy", 72, 0.52f, 1.5f, 1.0f, false},
      {"tachycardia, low SpO2", 120, 0.80f, 1.0f, 1.0f, false},
      {"bradycardia", 45, 0.55f, 2.0f, 1.0f, false},
      {"weak perfusion + noise", 80, 0.60f, 0.4f, 2.0f, true},
      {"critical SpO2", 95, 1.00f, 1.2f, 1.0f, false},
  };
  int fails = 0;
  for (const Case& c : cases) fails += run(c);

  // finger removed -> must report NO_FINGER / SATURATED, not a stale number
  pox::PulseOx ox;
  for (int n = 0; n < 200; n++) ox.update(10, 10, 5);
  pox::Reading r = ox.reading();
  printf("%-26s status %s valid %d  %s\n", "dark (no finger)", pox::statusText(r.status), r.valid,
         (!r.valid && r.status == pox::Status::NoFinger) ? "PASS" : "FAIL");
  fails += (r.valid || r.status != pox::Status::NoFinger);
  return fails;
}
