// 比較用：VL53L5CX（SparkFun のライブラリ）：I2C の距離の読み出し。ビルドの確認用（実機では未確認）
// 公式の Example1_DistanceArray 相当。I2C は Wire1、ピンは l8cx_i2c と同じ仮の番号
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define PIN_TOF_SDA   26        // 仮：カメラの SIOD
#define PIN_TOF_SCL   27        // 仮：カメラの SIOC
#define TOF_I2C_HZ    400000

static SparkFun_VL53L5CX sensor;
static VL53L5CX_ResultsData results;
static int imageWidth = 4;

void setup() {
  Serial.begin(115200);

  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);

  // アドレスは7ビットの書き方（0x29）で渡す
  if (!sensor.begin(0x29, Wire1)) {
    Serial.println("センサーが見つかりません");
    while (true) {
      delay(1000);
    }
  }
  sensor.setResolution(4 * 4);
  imageWidth = 4;
  sensor.startRanging();
}

void loop() {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&results)) {
      for (int i = 0; i < imageWidth * imageWidth; i++) {
        Serial.printf("%5d", (int)results.distance_mm[i]);
        if (i % imageWidth == imageWidth - 1) {
          Serial.println();
        }
      }
      Serial.println();
    }
  }
  delay(100);
}
