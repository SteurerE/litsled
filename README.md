# litsled 🛷🌈

**An adaptive sled light system for people who don't brake.**

*Wer bremst, verliert.* — Hermann Maier

## Background

This project evolved out of my shared passion for night sledging, often together with my brother and friends. We both enjoy the thrill of speeding downhill—sometimes at questionable speeds—on occasionally icy slopes, preferably in the dark. To improve visibility (and style), it became clear to us that upgrading our sleds with an adaptive light system, including a brake detection mechanism, was absolutely necessary. That’s why we created this project during the winter holidays of 2025/26. The system has been proven in real-world conditions, with the first test run taking place on January 3rd, 2026, near the Klagenfurter Hütte in Carinthia, Austria.

## Features

- 🛑 **Angle-aware adaptive brake light** — Detects braking actions based on gravity compensated and filtered accelerometer readings of the MPU6050 IMU chip.
- 🌈 **Speed Rainbow underbody light** — Underbody LEDs are animated based on pitch angle of the sled, the faster you go the higher the refresh rate.

## Software

This firmware was mainly vibe-coded and uses `platformio` for easy compilation and deployment. It includes IMU readout functionality, fusion of accelerometer/gyroscope using [Madgwick filter](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf), brake detection and LED control logic. Furthermore, the accelerometer and gyroscope biases can be calibrated and stored inside the microcontroller's flash memory. PR's are welcome!

## Hardware

Some custom housings were designed and 3D printed to incorporate the microntroller/IMU and the battery. If there is time this section will be updated so that a similiar system can easily be build by yourself or your grandparents.

### List of hardware components

- Low-cost microcontroller [Arduino Nano](https://www.amazon.de/dp/B0DBLWF5W6?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)
- Consumer grade IMU [MPU6050](https://www.amazon.de/dp/B07XRK5FHP?ref=ppx_yo2ov_dt_b_fed_asin_title)
- WS2812B LED strips(IP68 2m variant, can be found on various sited like AliExpress, Amazon, ...)
- [Small power bank](https://www.amazon.de/dp/B0BHZ6RY6C?ref=ppx_yo2ov_dt_b_fed_asin_title)
- [IP68 rated USB-C connector](https://www.amazon.de/dp/B0DWSJCYSJ?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)
- [USB-C cables](amazon.de/dp/B0BF4ZSFT1?ref=ppx_yo2ov_dt_b_fed_asin_title)

## Quick Start

```bash
brew install platformio
cd litsled
pio run --target upload
```
