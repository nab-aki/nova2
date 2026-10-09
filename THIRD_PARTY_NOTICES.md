# Third-Party Notices

This project's own code (everything outside `lib/`) is licensed under the
MIT License — see [LICENSE](LICENSE). The libraries below are bundled or
pulled in as dependencies and keep their own, separate licenses.

| Library | Location | Author | License | Modified by Nova? |
|---|---|---|---|---|
| PCA9685 | `lib/PCA9685/` | Peter Polidoro (Janelia Research Campus) | 3-clause BSD | **Yes** — see below |
| PCF8574 | `lib/PCF8574/` | Rob Tillaart | MIT | No |
| Freenove_VK16K33_Lib_For_ESP32 | `lib/Freenove_VK16K33_Lib_For_ESP32/` | Freenove | Unlicense (public domain) | **Yes** — see below |
| Freenove_WS2812_Lib_for_ESP32 | `lib/Freenove_WS2812_Lib_for_ESP32/` | Freenove | LGPL-3.0 | **Yes** — see below |
| IRremoteESP8266 | fetched by PlatformIO (`lib_deps`, not vendored in `lib/`) | David Conran and contributors (crankyoldgit) | LGPL-2.1 | No |

Each library's own `LICENSE`/`license.txt` file (kept in its folder) is the
authoritative copy; the table above is a summary, not a substitute for it.

## Freenove_WS2812_Lib_for_ESP32 — modifications

The vendored copy under `lib/Freenove_WS2812_Lib_for_ESP32/` is Freenove's
v2.0.1 release, with a small patch so it builds against Arduino-ESP32 2.x
(the upstream v2.0.1 targets the 3.x RMT API). The change is documented in
[lib/Freenove_WS2812_Lib_for_ESP32/NOVA_PATCH.md](lib/Freenove_WS2812_Lib_for_ESP32/NOVA_PATCH.md),
and the modified lines are marked `[Nova]` in the source. It remains under
the original LGPL-3.0 license.

## PCA9685 and Freenove_VK16K33_Lib_For_ESP32 — modifications

Both vendored copies (`lib/PCA9685/`, v3.0.3, and
`lib/Freenove_VK16K33_Lib_For_ESP32/`, v1.0.0) carry the same minimal patch:
the return values of `Wire.endTransmission()` / `Wire.requestFrom()`, which
upstream discards, are counted in three public counters (`nova_i2c_total`,
`nova_i2c_fail`, `nova_i2c_timeout`) so that I2C failures can be reported per
device. What is sent on the bus is unchanged. The change is documented in
[lib/PCA9685/NOVA_PATCH.md](lib/PCA9685/NOVA_PATCH.md) and
[lib/Freenove_VK16K33_Lib_For_ESP32/NOVA_PATCH.md](lib/Freenove_VK16K33_Lib_For_ESP32/NOVA_PATCH.md)
(one per library), and the modified lines are marked `[Nova]` in the
source. Each library remains under its original license.

## Pin numbers, I2C addresses and similar hardware facts

Pin numbers, I2C addresses, and a handful of physical/calibration constants
(e.g. the speed of sound used for the ultrasonic sensor, the battery voltage
divider ratio, the servo pulse-width range) follow the values published by
Freenove for the 4WD Car Kit for ESP32
(https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32, itself released
under CC BY-NC-SA 3.0). These are treated as hardware facts rather than
copied source code — see the "Acknowledgements" section of
[README.md](README.md). No file from that repository is vendored here.
