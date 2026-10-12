// VL53L8CX（STM32duino のライブラリ）：SPI の HelloWorld 相当。ビルドが通るかだけの確認用（実機では未確認）
// 公式の examples/VL53L8CX_HelloWorld_SPI から、ESP32 向けに変えた所：
//   - SPIClass の作り方。公式の SPIClass(MOSI, MISO, CLK) は STM32 だけの書き方なので、
//     ESP32 の SPIClass(バス) と begin(SCK, MISO, MOSI, SS) にした
//   - ピンは、カメラが使っている GPIO から仮に選んだ番号
//   - PWREN はつながない前提。シリアルは 115200bps
#include <Arduino.h>
#include <SPI.h>
#include <vl53l8cx.h>

#define PIN_TOF_SCK   18   // 仮
#define PIN_TOF_MISO  19   // 仮
#define PIN_TOF_MOSI  23   // 仮
#define PIN_TOF_CS    5    // 仮

static SPIClass tofSpi(VSPI);
static VL53L8CX sensor(&tofSpi, PIN_TOF_CS);
static uint8_t resolution = VL53L8CX_RESOLUTION_4X4;

void setup() {
  Serial.begin(115200);

  tofSpi.begin(PIN_TOF_SCK, PIN_TOF_MISO, PIN_TOF_MOSI, PIN_TOF_CS);

  sensor.begin();
  uint8_t status = sensor.init();
  Serial.printf("init: %u\n", status);
  status = sensor.start_ranging();
  Serial.printf("start_ranging: %u\n", status);
}

void loop() {
  VL53L8CX_ResultsData results;
  uint8_t ready = 0;
  uint8_t status = sensor.check_data_ready(&ready);
  if (status == 0 && ready != 0) {
    status = sensor.get_ranging_data(&results);
    if (status == 0) {
      int perLine = (resolution == VL53L8CX_RESOLUTION_4X4) ? 4 : 8;
      for (int i = 0; i < resolution; i++) {
        if (results.nb_target_detected[i] > 0) {
          Serial.printf("%5d", (int)results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
        } else {
          Serial.print("    X");
        }
        if (i % perLine == perLine - 1) {
          Serial.println();
        }
      }
      Serial.println();
    }
  }
  delay(50);
}
