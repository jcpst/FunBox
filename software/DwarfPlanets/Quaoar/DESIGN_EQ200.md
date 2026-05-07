# Quaoar EQ-200 Style Expansion — Design Notes

## Concept

A Boss EQ-200 inspired 11-band graphic EQ built on the Daisy Seed, housed in a 1590XX enclosure. Dual EQ channels configurable as series or parallel, with OLED displays showing the active EQ curve (useful for presets where fader positions don't match stored values).

## Hardware

### Controls
- 12 faders: 11 frequency bands + 1 master level (2× 6-fader breakout PCBs)
- 2 footswitches (bypass, preset/channel toggle)
- 2 LEDs

### Displays
- 2× SSD1306 128×64 OLED (I²C, addresses 0x3C and 0x3D on shared bus)
- Each display shows the EQ curve for one channel

### I/O
- Stereo audio in/out (Daisy Seed codec)
- MIDI in/out (UART)

### Pin Budget (Daisy Seed)

| Function | Pins | Type |
|----------|------|------|
| 12 faders | 12 | ADC (channels 0–11) |
| 2 footswitches | 2 | Digital GPIO |
| 2 LEDs | 2 | Digital GPIO |
| MIDI TX/RX | 2 | UART |
| 2× OLED (shared I²C) | 2 | I²C SDA/SCL |
| **Total** | **20** | 12 ADC + 8 digital |

Remaining: 0 spare ADC, ~12 spare digital GPIO.

## Mechanical Approach

### Stripboard Breakout (Prototype Phase)

The Daisy Seed mounts on a stripboard with stacking headers. The stripboard plugs into the Funbox PCB socket (providing audio codec, power, and potentially reusing some existing controls) while also breaking out spare pins via ribbon cable to the expansion panel with sliders and OLEDs.

For the 1590XX build, the Funbox PCB may not be used at all — instead the stripboard breakout wires directly to:
- 2× 6-fader breakout PCBs (12 slide potentiometers)
- 2 footswitches
- 2 OLED modules
- Audio jacks (via Daisy Seed codec)
- MIDI jacks (via UART pins)

### Custom PCB (Production Phase)

Once validated, a simple 2-layer PCB with:
- Daisy Seed header footprint
- 12 slider footprints (vertical or horizontal)
- 2× OLED header connectors
- Footswitch and LED connections
- MIDI and audio jack connections
- Proper ground planes for low-noise analog reads

## Software Architecture

### Audio Processing
- 11-band graphic EQ: fixed center frequencies (30, 60, 120, 200, 400, 800, 1.2k, 1.6k, 3.2k, 6.4k, 12.8k Hz)
- Each band is a biquad peaking filter (RBJ Audio EQ Cookbook)
- Dual channel: 22 biquads total (stereo) — trivial CPU load on 480 MHz Cortex-M7
- Series mode: input → EQ A → EQ B → output
- Parallel mode: input → EQ A + EQ B (summed) → output
- Master level stage after EQ processing

### Preset System
- Store fader positions to flash memory
- On preset recall, stored values drive the filters (physical faders disconnected)
- "Pickup" mode: fader regains control once it crosses the stored value
- MIDI program change for preset recall

### Display
- Compute magnitude response at ~64 frequency points from biquad transfer functions
- Draw EQ curve on each OLED (~30 Hz refresh, alternating between displays)
- Update in main() loop, never in audio callback

### Initialization
- Use raw `DaisySeed` class (not `DaisyPetal`) since this exceeds the Funbox hardware mapping
- Manual `AdcChannelConfig` for 12 ADC channels
- Manual I²C init for OLEDs
- UART init for MIDI

## EQ Band Frequencies

| Band | Frequency |
|------|-----------|
| 1 | 30 Hz |
| 2 | 60 Hz |
| 3 | 120 Hz |
| 4 | 200 Hz |
| 5 | 400 Hz |
| 6 | 800 Hz |
| 7 | 1.2 kHz |
| 8 | 1.6 kHz |
| 9 | 3.2 kHz |
| 10 | 6.4 kHz |
| 11 | 12.8 kHz |

Each band: ±15 dB boost/cut, fixed Q (narrow enough for graphic EQ, typically Q ≈ 2–4).

## Open Questions

- Slider travel length vs. enclosure depth in 1590XX — verify fit
- Whether to reuse Funbox PCB for audio/power or go fully standalone
- OLED placement: top panel or side-mounted behind a window
- Whether to add a mode switch (series/parallel) as a physical toggle or MIDI-only

