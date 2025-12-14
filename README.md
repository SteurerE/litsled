# lidsled 🛷💡

**A brake light for people who don't brake.**

*Wer bremst, verliert.* — Ancient Alpine Wisdom

Congratulations on being a *responsible* sled pilot. You've decided that visibility matters. That the people behind you deserve a warning before you inevitably don't slow down anyway. Truly, you are the model citizen of the slopes.

This project gives you a state-of-the-art brake detection system featuring multi-stage signal filtering, hysteresis-based state machines, and IMU sensor fusion. It is engineered to detect the precise moment your foot touches the snow with millisecond accuracy.

We sincerely hope you never use it.

But when Oma and her walking poles appear around that blind corner, you'll be grateful it exists. And more importantly, you'll be *legally covered*.

## Features

- 🛑 **Brake Light** — Meticulously engineered. Ideally decorative.
- 🌈 **Speed Rainbow** — Side LEDs animate based on pitch. Steeper slope = faster rainbow. Reward yourself for commitment.
- 🧠 **Smart Filtering** — Ignores bumps and vibrations. Only triggers on *actual* braking. Which, statistically, should be rare.

## Hardware

- Arduino Nano
- MPU6050 IMU
- 2m WS2812B LED strip (66 LEDs)
- Plausible deniability

## Quick Start

```bash
brew install platformio
cd lidsled
pio run --target upload
```

Attach to sled. Descend mountain. *Try* to need the brake light.

---

*In case of accident: "The system worked perfectly, I simply chose not to engage it."*
