# Nova

Nova is a homemade "pet robot" project, aiming for something like a smaller,
less-capable version of [LOVOT](https://lovot.life/en) — a companion robot with
a sense of presence and reactions to the person nearby, rather than a simple
line-tracing or obstacle-avoiding car.

The current top priority is the most basic behavior: **autonomously wandering
around a room while avoiding obstacles.** Reactive and playful behaviors are
being layered on top of that base, one small behavior at a time (see
[docs/catalog.md](docs/catalog.md) for the running list and
[docs/handoff.md](docs/handoff.md) for the current status).

Project log / build diary: https://hackaday.io/project/206842-nova-a-homemade-pet-robot-starting-with-a-robot

## Hardware

Base kit: **Freenove 4WD Car Kit for ESP32** (SKU: FNK0053)
https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32

The kit is used largely as-is (ESP32-WROVER module, PCA9685-driven motors and
pan/tilt servos, VK16K33 LED matrix, PCF8574 line-tracking sensor, WS2812
LEDs, passive buzzer, photoresistor, IR receiver). Pin numbers and I2C
addresses follow the kit's official definitions unless noted otherwise.

### Changes from the stock kit

#### Power: 6x NiMH batteries + XT30 connector

The kit calls for two 18650 lithium-ion cells. Nova runs on six NiMH
rechargeable AA batteries instead.

- **Batteries:** 6 × NiMH rechargeable AA in a 6-cell battery box. That gives
  about 7.2 V nominal (1.2 V × 6), close to the about 7.4 V nominal of two
  18650 cells.
- **Connector:** an XT30 pair with AWG16 leads, so the battery box can be
  unplugged.
  - Female side: soldered to the battery box leads.
  - Male side: soldered to the battery holder's solder terminals on the 4WD
    car board.
- **Power switch:** the power switch on the board still works as before.

Notes:

- Use NiMH rechargeable batteries only. Six alkaline AA batteries give a
  higher voltage.
- Check the polarity with a multimeter before plugging the battery in for
  the first time.
- AWG16 wire is thick. A 15 W soldering iron struggled with it; 30 W or more
  makes the job much easier.
- The leads that came with my battery box are thin (probably AWG24 or
  thinner). I haven't replaced them yet, but I recommend rewiring them with
  AWG18 or thicker.

Tested for wandering around on wooden floors at home. Long continuous runs
are not tested.

#### Custom cable: LED matrix + ultrasonic sensor sharing the neck

The front-facing ultrasonic sensor has been removed; the only distance sensor
is on the pan/tilt "neck", alongside the LED matrix eyes. Running both on the
neck at once needed a custom cable.

The kit's manual doesn't cover using the ultrasonic sensor and the LED
matrix at the same time, and the stock cable only lets you connect one or
the other.

- **LED matrix:** connected with the kit's stock cable, unchanged.
- **Ultrasonic sensor:** connected with a homemade 4-pin cable.
  - Length: about 150 mm
  - Connectors: AMP MODU-compatible 4-pin connectors on both ends
  - Wiring: straight. The four lines (VCC, Trig, Echo, GND) are in the same
    order on both ends.
  - Board side: plugs into the header marked for the ultrasonic sensor on
    the 4WD car board.

Before powering on, check the pin labels on both the sensor and the board.
Swapping VCC and GND can damage the sensor.

This works in my setup. It is not an officially documented configuration.

## Status

This is an active, in-progress hobby project — not a finished product. The
"wandering while avoiding obstacles" behaviors, a stuck-detection/recovery
behavior, and the supporting safety layer are implemented and have been
tested on the physical robot; most of the more playful/expressive behaviors
in the catalog are still candidates, not yet built.

All of the firmware code was written with [Claude Code](https://claude.com/claude-code).

## Building and uploading

This project is developed with **PlatformIO**, and that's the tested,
supported way to build it.

### PlatformIO (tested)

1. Install [PlatformIO](https://platformio.org/) (e.g. the VS Code extension,
   or `pip install platformio`).
2. Open this folder as a PlatformIO project (`platformio.ini` is at the repo
   root; its `[env:fnk0053]` section pins the ESP32 Arduino core version and
   build flags).
3. Build: `pio run`
4. Upload (board connected over USB, close any serial monitor first):
   `pio run -t upload --upload-port <COMx or /dev/ttyUSBx>`
5. Serial monitor: `pio device monitor -b 115200` (or any terminal at
   115200 bps).

### Arduino IDE (untested — not how this project is actually built)

The notes below translate the PlatformIO setup into Arduino IDE terms, but
have **not** been verified end-to-end; please treat them as a starting point.

1. Install the ESP32 board package via Boards Manager
   (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`).
   **Important:** install core version **2.0.17**, not the newer 3.x line
   that the kit's own tutorial suggests — this firmware uses 2.x-era APIs
   (e.g. `ledcAttachPin`/`ledcSetup`, and the patched WS2812 library below
   targets the 2.x RMT driver) and will not build against 3.x as-is.
2. Board: **ESP32 Wrover Module**.
3. Libraries — add each folder under [lib/](lib/) to your Arduino
   `libraries/` folder (or use "Add .ZIP Library"):
   - `PCA9685` (motor/servo driver)
   - `PCF8574` (line-tracking sensor)
   - `Freenove_VK16K33_Lib_For_ESP32` (LED matrix "eyes")
   - `Freenove_WS2812_Lib_for_ESP32` — **use the copy in this repo**, not the
     stock Freenove one; it's been patched for the 2.x core (see
     [lib/Freenove_WS2812_Lib_for_ESP32/NOVA_PATCH.md](lib/Freenove_WS2812_Lib_for_ESP32/NOVA_PATCH.md)).
   - `IRremoteESP8266` by crankyoldgit — install via Library Manager, version
     `2.8.6`. PlatformIO builds it with only the NEC protocol enabled via
     build flags (`-D_IR_ENABLE_DEFAULT_=false -DDECODE_NEC=true`) to save
     flash; the Arduino IDE has no per-project equivalent, so building with
     all protocols enabled is the simplest fallback (larger binary, should
     still work).
4. Open [src/main.cpp](src/main.cpp) as the sketch and upload as usual.

## Acknowledgements

Hardware details that aren't creative expression — pin numbers, I2C
addresses, and a few physical/calibration constants (speed of sound, the
battery voltage divider ratio, the servo pulse-width range) — follow the
values published by Freenove for the
[4WD Car Kit for ESP32](https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32).
No file from that repository is vendored in this one.

## License

This project's own code is licensed under the **MIT License** — see
[LICENSE](LICENSE).

Everything under [lib/](lib/) is bundled third-party code and keeps its
original license (BSD-3, MIT, Unlicense, or LGPL-3.0 depending on the
library) — see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the
full list and each library's own `LICENSE` file for the authoritative text.
