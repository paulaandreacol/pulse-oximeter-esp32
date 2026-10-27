# ESP32 Pulse Oximeter

Two-wavelength pulse oximeter built for **49261 Biomedical Instrumentation** (UTS), Major Project 2.6, by Team 5.

A red (660 nm) and an infrared (940 nm) LED shine into a fingertip, and a phototransistor picks up the light that comes back. A TL074 op-amp filter chain (0.5–5 Hz, gain ×48) cleans up the pulse. An ESP32 switches the LEDs, reads the signals and calculates:

- heart rate
- SpO₂ (ratio of ratios)
- perfusion index
- a status flag (no finger, poor signal, low SpO₂, fast or slow heart rate, …)

Results are shown on an OLED and sent over the serial port.

> **Not a medical device.** SpO₂ readings must be calibrated against a commercial oximeter before they mean anything.

## Repository contents

| Path | What it is |
|---|---|
| `PulseOximeter_ESP32/` | Arduino sketch: `PulseOximeter_ESP32.ino` (hardware, timing, output) and `PulseOxDSP.h` (signal processing) |
| `host_test/sim_test.cpp` | PC test of the signal processing on synthetic red/IR signals |
| `ltspice/` | LTspice design files (see [LTspice files](#ltspice-files)) |

## How it works

```
GPIO25/26 ─► red + IR LEDs ─► finger ─► phototransistor (VSENSE)
                                              │
                         ┌────────────────────┴───────────────────┐
                         ▼                                        ▼
                 GPIO34 (raw, per LED phase)       TL074 chain: high-pass 0.5 Hz
                         │                         → low-pass 5 Hz → gain ×48
                         │                         → level shifter → GPIO35
                         ▼                                        ▼
            digital filters, beat detection,           heart rate from the
            SpO₂, perfusion index, status flag         analog (op-amp) pulse
                         └──────────────┬─────────────────────────┘
                                        ▼
                              OLED display + serial port
```

Each step in order:

1. **LED sequencing.** Red on → IR on → both off, 333 µs each, about 1000 cycles per second. Switching this fast lets the 5 Hz analog filters smooth the switching away.
2. **Averaging and ambient subtraction.** Every 20 cycles are averaged into one sample (≈ 47.7 samples/s). The "both off" reading is subtracted to remove room light.
3. **Digital filtering.** Each wavelength goes through 2nd-order Butterworth filters with the same corners as the analog chain (0.5 Hz high-pass, 5 Hz low-pass).
4. **Beat detection.** A slope-sum function (Zong et al., 2003) responds to the steep rise of each pulse and ignores the dicrotic notch. Heart rate is the median of the last 8 beat intervals.
5. **SpO₂.** Calculated from R = (AC_red/DC_red) / (AC_IR/DC_IR) as SpO₂ = A − B·R. The defaults are A = 110 and B = 25; calibrate them.
6. **Status flag.** A small decision tree checks the signal quality first, then SpO₂ and heart-rate limits.
7. **OLED.** Drawn on the ESP32's second core, so the slow display updates never disturb the LED timing.

## Hardware

### Parts

| Part | Notes |
|---|---|
| ESP32 DevKit (ESP32-WROOM-32) | Powered over USB |
| TL074 quad op-amp | ±12 V bench supply |
| Phototransistor | Must see both visible and IR light (clear package) |
| Red LED 660 nm, IR LED 940 nm | 5 mm |
| OLED, DFRobot DFR0647 | 0.96", 128×64, I²C (SSD1306) |
| Resistors | 100 Ω, 150 Ω, 10 kΩ ×4, 39 kΩ, 680 kΩ, 470 kΩ, 3.3 MΩ |
| Capacitors | 470 nF, 10 nF, 68 nF ×3 (or 10 nF for supply decoupling) |

### Wiring

The TL074 runs from **±12 V**, referenced to ground. The sensor, LEDs and OLED run from the **ESP32's 3.3 V**. Both supplies share **one common ground**.

**Nothing from the ±12 V side may touch an ESP32 pin.**

**ESP32**

| ESP32 pin | Connected to |
|---|---|
| GPIO25 | 100 Ω → red LED anode (cathode → GND) |
| GPIO26 | 150 Ω → IR LED anode (cathode → GND) |
| GPIO34 | VSENSE: phototransistor emitter, 10 kΩ to GND, C1 |
| GPIO35 | Level shifter output (39 kΩ from TL074 pin 14, 10 kΩ to 3V3, 10 kΩ to GND) |
| GPIO21 / GPIO22 | OLED SDA / SCL |
| 3V3 | Phototransistor collector, OLED VCC, level-shifter 10 kΩ |
| GND | Common ground (also bench supply COM) |

**TL074** (notch at the top, pin 1 top-left)

| Pin | Connected to |
|---|---|
| 1, 2, 3 | Op-amp A unused: pins 1 and 2 joined, pin 3 to GND |
| 4 | +12 V, 68 nF to GND |
| 5 | C1 470 nF from VSENSE; R2 680 kΩ to GND (high-pass, 0.50 Hz) |
| 6, 7 | Joined (buffer); pin 7 → R3 3.3 MΩ |
| 10 | R3; C2 10 nF to GND (low-pass, 4.8 Hz) |
| 8, 9 | Joined (buffer); pin 8 → pin 12 |
| 11 | −12 V, 68 nF to GND |
| 13 | R4 10 kΩ to GND; R5 470 kΩ ∥ 68 nF to pin 14 (gain 1 + R5/R4 = 48) |
| 14 | AMPLIFIED_OUT → 39 kΩ → GPIO35 |

The level shifter maps the op-amp's ±10.5 V swing to 0.27–2.66 V, which is safe for the ESP32 ADC.

**Power-up order:** plug in the ESP32's USB first, then switch on ±12 V. Switch off in reverse.

## Software setup

1. **Board package.** In the Arduino IDE, open Boards Manager and install **esp32 by Espressif** (core 2.x or 3.x). Select the board **ESP32 Dev Module**.
2. **Libraries.** Open Library Manager and install **Adafruit SSD1306**; accept installing **Adafruit GFX** too.
3. **Open the sketch.** Open `PulseOximeter_ESP32/PulseOximeter_ESP32.ino`. Keep `PulseOxDSP.h` in the same folder.
4. **Run it.** Upload, then open Serial Monitor at **115200** baud.

### Settings (top of the `.ino`)

| Setting | Default | Meaning |
|---|---|---|
| `OUTPUT_MODE` | 0 | 0 = summary once a second, 1 = Serial Plotter waveforms, 2 = CSV every sample |
| `USE_OLED` | 1 | 0 builds without the display |
| `OLED_HEIGHT` | 64 | 64 for a 0.96" module, 32 for 0.91" |
| `READS_PER_PHASE` | 1 | ADC reads per LED phase; raise only if no "timing overruns" warning appears |
| `ANALOG_GAIN` | 48 × 5/44 | Op-amp gain times level-shifter scale; only affects Serial Plotter scaling |
| `c.spo2A`, `c.spo2B` | 110, 25 | SpO₂ calibration line, in `makeConfig()` |

### Calibrating SpO₂

1. Set `OUTPUT_MODE 2` and log the CSV output next to a commercial finger oximeter.
2. Fit a straight line, SpO₂ = A − B·R, using the `ratio` column.
3. Put A and B into `makeConfig()` and upload again.

## Testing the signal processing on a PC

```bash
cd host_test
c++ -std=c++17 -O2 -I ../PulseOximeter_ESP32 sim_test.cpp -o sim_test
./sim_test
```

The test runs six cases: resting, fast heart rate with low SpO₂, slow heart rate, critical SpO₂, a weak noisy signal, and no finger. For each, it checks heart rate, SpO₂, perfusion index and the status flag.

## LTspice files

| File | Notes |
|---|---|
| `ltspice/Draft5_ESP32.asc` | First draft with fixes: photodiode stage, 1.65 V reference (3.3 V op-amp version) |
| `ltspice/Draft5_ESP32transient.asc` | As above, plus a 68 nF capacitor and a peak-to-peak measurement of the output |
| `ltspice/Draft5_Phototransistor.asc` | Phototransistor + 10 kΩ input instead of the photodiode stage |

The built circuit runs at ±12 V with ground as the reference. None of these files matches that build exactly yet.

## References

- Zong, W., Heldt, T., Moody, G. B., & Mark, R. G. (2003). An open-source algorithm to detect onset of arterial blood pressure pulses. *Computers in Cardiology, 30*, 259–262.
- Webster, J. G. (Ed.). (1997). *Design of pulse oximeters*. IOP Publishing.
