/*
  ESP32 Pulse Oximeter - 49261 Biomedical Instrumentation, Major Project (2.6)

  What it does:
    * Cycles the LEDs RED -> IR -> both off (ambient), about 1000 times a
      second, and reads the phototransistor (VSENSE, GPIO34) in each step.
      Switching this fast lets the op-amp filter chain (5 Hz low-pass) average
      the switching away, so its output is a clean pulse waveform.
    * Reads the op-amp chain output (AMPLIFIED_OUT, GPIO35).
    * Every 20 LED cycles, averages the readings into one sample (~48 per s):
        - red/IR/ambient from GPIO34 -> digital 0.5-5 Hz filters per
          wavelength -> heart rate, SpO2 (ratio of ratios), perfusion index,
          status flag (small decision tree)
        - GPIO35 -> heart rate measured from the analog (op-amp) pulse, to
          compare against the digital result
    * Sends results over the serial port and shows them on an I2C OLED
      (DFRobot DFR0647, SSD1306). The OLED is drawn from the ESP32's other
      core, so its slow I2C transfers never disturb the LED timing.

  Wiring (see README.md):
    GPIO25 -> 100R -> RED LED (660 nm) -> GND   (~14 mA)
    GPIO26 -> 150R -> IR  LED (940 nm) -> GND   (~14 mA)
    3V3 -> phototransistor collector; emitter = VSENSE -> GPIO34, 10k to GND
    VSENSE -> C1 470n -> op-amp filter chain (TL074 on +/-12 V, reference = GND)
           -> AMPLIFIED_OUT -> level shifter -> GPIO35
              (39k from pin 14 to GPIO35, 10k GPIO35 to 3V3, 10k GPIO35 to GND:
               maps the +/-10.5 V op-amp swing into 0.3-2.7 V, 0 V -> 1.5 V)
    OLED: VCC -> 3V3, GND -> GND, SCL -> GPIO22, SDA -> GPIO21
    NEVER feed more than 3.3 V into GPIO34/35.

  Libraries (Arduino IDE > Library Manager): "Adafruit SSD1306" (also installs
  "Adafruit GFX Library"). Set USE_OLED to 0 to build without a display.

  Board: "ESP32 Dev Module" in the Arduino IDE (arduino-esp32 core 2.x or 3.x).
  Not a medical device: SpO2 must be calibrated against a commercial oximeter.
*/

#include "PulseOxDSP.h"

// ----------------------------- Pins -----------------------------------------
const int PIN_LED_RED = 25;
const int PIN_LED_IR = 26;
const int PIN_SENSOR = 34;  // VSENSE (phototransistor emitter), ADC1
const int PIN_ANALOG = 35;  // AMPLIFIED_OUT (op-amp chain output), ADC1
const int PIN_SDA = 21;     // OLED
const int PIN_SCL = 22;

// ----------------------------- Timing ---------------------------------------
// One LED cycle = red phase + IR phase + dark phase, each PHASE_US long
// (3 x 333 us = 1 ms, i.e. 1 kHz). BLOCK_CYCLES cycles are averaged into one
// sample, then the LEDs stay off for GAP_US while the sample is processed.
const uint32_t PHASE_US = 333;
const uint32_t SETTLE_US = 120;   // phototransistor rise/fall time before reading
const int READS_PER_PHASE = 1;    // GPIO34 reads per phase (plus one GPIO35 read)
const int BLOCK_CYCLES = 20;
const uint32_t GAP_US = 1000;
const uint32_t BLOCK_US = 3 * PHASE_US * BLOCK_CYCLES + GAP_US;  // 20.98 ms -> 47.7 samples/s
const float SAMPLE_HZ = 1e6f / BLOCK_US;

// op-amp gain stage 1 + R5/R4 = 48, times the level shifter 5k/(39k + 5k)
const float ANALOG_GAIN = 48.0f * 5.0f / 44.0f;
const bool ANALOG_INVERT = true;   // more blood = less light = output dips -> flip

// ----------------------------- Output ---------------------------------------
// 0 = readable summary once a second
// 1 = Arduino Serial Plotter (digital IR pulse vs analog op-amp pulse)
// 2 = CSV every sample (log to a file, then plot / calibrate in MATLAB or Python)
#define OUTPUT_MODE 0

// ----------------------------- OLED -----------------------------------------
#define USE_OLED 1
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;      // 0.91" module = 32; use 64 for a 0.96" module
const uint8_t OLED_ADDR = 0x3C;

#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

pox::Config makeConfig() {
  pox::Config c;
  c.fs = SAMPLE_HZ;
  // Calibration: log R (CSV mode) next to a commercial oximeter, fit SpO2 = A - B*R
  c.spo2A = 110.0f;
  c.spo2B = 25.0f;
  return c;
}

pox::PulseOx oximeter(makeConfig());
pox::BeatDetector analogBeats;  // heart rate from the op-amp chain output

#if USE_OLED
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Latest results, written by loop() (core 1) and drawn by displayTask (core 0)
struct DisplayData {
  pox::Reading r;
  float analogBpm;
};
DisplayData shown;
portMUX_TYPE shownLock = portMUX_INITIALIZER_UNLOCKED;

const char* friendlyStatus(pox::Status s) {
  switch (s) {
    case pox::Status::NoFinger:  return "Place finger";
    case pox::Status::Searching: return "Measuring...";
    case pox::Status::Saturated: return "Too bright";
    default:                     return pox::statusText(s);
  }
}

// One frame takes ~15 ms over I2C, far longer than the 1 ms processing gap,
// so the display runs as its own task on core 0, twice a second.
void displayTask(void*) {
  for (;;) {
    DisplayData d;
    portENTER_CRITICAL(&shownLock);
    d = shown;
    portEXIT_CRITICAL(&shownLock);

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    if (d.r.valid) {
      oled.setTextSize(2);  // big numbers: heart rate and SpO2
      oled.setCursor(0, 0);
      oled.printf("%3.0f %3.0f%%", d.r.bpm, d.r.spo2);
      oled.setTextSize(1);
      oled.setCursor(0, 16);
      oled.printf("bpm  SpO2   PI %.1f%%", d.r.pi);
      oled.setCursor(0, 24);
      oled.print(pox::statusText(d.r.status));
      if (!isnan(d.analogBpm)) oled.printf(" amp %.0f", d.analogBpm);
    } else {
      oled.setTextSize(1);
      oled.setCursor(0, 0);
      oled.print("Pulse oximeter");
      oled.setTextSize(2);
      oled.setCursor(0, 14);
      oled.print(friendlyStatus(d.r.status));
    }
    oled.display();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
#endif

struct Sample {
  float red, ir, amb, analog;  // mV
};

uint32_t overruns = 0;  // phases or blocks that ran late (ADC too slow)

void waitUntil(uint32_t t) {
  while ((int32_t)(micros() - t) < 0) {
  }
}

// One LED phase: LED on (ledPin < 0 = dark), let the detector settle, read
// both ADCs, LED off exactly at the end of the phase.
void runPhase(uint32_t start, int ledPin, float& sensorSum, float& analogSum) {
  waitUntil(start);
  if (ledPin >= 0) digitalWrite(ledPin, HIGH);
  waitUntil(start + SETTLE_US);
  for (int k = 0; k < READS_PER_PHASE; k++) sensorSum += analogReadMilliVolts(PIN_SENSOR);
  analogSum += analogReadMilliVolts(PIN_ANALOG);
  if ((int32_t)(micros() - (start + PHASE_US)) > 0) overruns++;
  waitUntil(start + PHASE_US);
  if (ledPin >= 0) digitalWrite(ledPin, LOW);
}

Sample acquireBlock(uint32_t t0) {
  float red = 0, ir = 0, amb = 0, analog = 0;
  for (int c = 0; c < BLOCK_CYCLES; c++) {
    uint32_t t = t0 + c * 3 * PHASE_US;
    runPhase(t, PIN_LED_RED, red, analog);         // brief: one on,
    runPhase(t + PHASE_US, PIN_LED_IR, ir, analog);  // then the other,
    runPhase(t + 2 * PHASE_US, -1, amb, analog);     // then both off
  }
  const float n = BLOCK_CYCLES * READS_PER_PHASE;
  return {red / n, ir / n, amb / n, analog / (3 * BLOCK_CYCLES)};
}

void printSummary(const pox::Reading& r, float analogBpm) {
  if (!r.valid) {
    Serial.printf("[%s]  ir=%.0f mV  red=%.0f mV", pox::statusText(r.status),
                  oximeter.lastIr, oximeter.lastRed);
  } else {
    Serial.printf("HR %3.0f bpm | SpO2 %5.1f %% | PI %4.2f %% | R %.3f | Q %.2f | %s",
                  r.bpm, r.spo2, r.pi, r.ratio, r.quality, pox::statusText(r.status));
  }
  if (!isnan(analogBpm)) Serial.printf(" | op-amp HR %3.0f bpm", analogBpm);
  if (overruns) Serial.printf(" | WARNING %lu timing overruns: lower READS_PER_PHASE",
                              (unsigned long)overruns);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_IR, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_IR, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR, ADC_11db);  // ~0.15-3.1 V input range
  analogSetPinAttenuation(PIN_ANALOG, ADC_11db);

  analogBeats.setup(SAMPLE_HZ, 3.0f, 30.0f, 200.0f, 0.5f);

#if USE_OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    xTaskCreatePinnedToCore(displayTask, "oled", 4096, nullptr, 1, nullptr, 0);
  } else {
    Serial.println("OLED not found - check wiring / address (0x3C)");
  }
#endif

  delay(500);
#if OUTPUT_MODE == 0
  Serial.println("ESP32 pulse oximeter - place finger in the clip");
#elif OUTPUT_MODE == 2
  Serial.println("t_ms,red_mV,ir_mV,amb_mV,analog_mV,red_ac,ir_ac,beat,bpm,spo2,pi,ratio,"
                 "quality,status,analog_bpm");
#endif
}

void loop() {
  static uint32_t t0 = micros() + 1000;
  static uint32_t lastPrintMs = 0;
  static bool hadFinger = false;
  static float analogDc = NAN;

  Sample s = acquireBlock(t0);
  t0 += BLOCK_US;  // LEDs now off for GAP_US: process and print in this gap

  bool beat = oximeter.update(s.red, s.ir, s.amb);
  pox::Reading r = oximeter.reading();

  // Analog channel: AMPLIFIED_OUT is already band-passed by the op-amps; only
  // the slow mean (~1.65 V) is removed here for plotting.
  if (isnan(analogDc)) analogDc = s.analog;
  analogDc += 0.02f * (s.analog - analogDc);
  float analogPulse = (ANALOG_INVERT ? -1.0f : 1.0f) * (s.analog - analogDc);

  bool finger = oximeter.fingerPresent();
  if (finger && !hadFinger) analogBeats.reset(analogPulse);
  hadFinger = finger;
  if (finger) analogBeats.update(analogPulse);
  float analogBpm = (finger && analogBeats.count() >= 2 && analogBeats.recent(3.0f))
                        ? analogBeats.bpm() : NAN;

#if USE_OLED
  portENTER_CRITICAL(&shownLock);
  shown = {r, analogBpm};
  portEXIT_CRITICAL(&shownLock);
#endif

#if OUTPUT_MODE == 0
  (void)beat;
  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    printSummary(r, analogBpm);
  }
#elif OUTPUT_MODE == 1
  (void)lastPrintMs;
  (void)r;
  (void)analogBpm;
  // analog pulse divided by the op-amp gain so both traces are in sensor mV
  Serial.printf("IR_digital:%.2f,OpAmp_analog:%.2f,Beat:%d\n", oximeter.irPulse,
                analogPulse / ANALOG_GAIN, beat ? 10 : 0);
#else
  (void)lastPrintMs;
  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%.1f,%.1f,%.2f,%.4f,%.3f,%s,%.1f\n",
                (unsigned long)millis(), s.red, s.ir, s.amb, s.analog, oximeter.redPulse,
                oximeter.irPulse, beat ? 1 : 0, r.bpm, r.spo2, r.pi, r.ratio, r.quality,
                pox::statusText(r.status), analogBpm);
#endif

  if ((int32_t)(micros() - t0) > 0) {  // processing ran past the gap: resync
    overruns++;
    t0 = micros() + 100;
  }
}
