// VL53L8CX（STM32duino のライブラリ）：I2C の HelloWorld 相当。ビルドの確認用（実機では未確認）
// 公式の examples/VL53L8CX_HelloWorld_I2C から、ESP32 向けに変えた所：
//   - I2C は Wire1（第2バス）。ピンはカメラの SIOD／SIOC と同じ番号を仮に使う
//   - LPn・PWREN はつながない前提（-1）。公式の A3・D11 は Nucleo のピンで、ESP32 では別の意味になる
//   - シリアルは 115200bps
#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

#define PIN_TOF_SDA   26        // 仮：カメラの SIOD
#define PIN_TOF_SCL   27        // 仮：カメラの SIOC
#define TOF_I2C_HZ    400000    // 400kHz（センサーは 1MHz まで）
#define PIN_TOF_LPN   -1        // つながない

static VL53L8CX sensor(&Wire1, PIN_TOF_LPN);
static uint8_t resolution = VL53L8CX_RESOLUTION_4X4;

void setup() {
  Serial.begin(115200);

  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);

  sensor.begin();                       // アドレスは既定（8ビットの書き方で 0x52＝7ビットで 0x29）
  uint8_t status = sensor.init();       // ここでファームウェア（約84KB）を送る
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
  delay(100);
}
