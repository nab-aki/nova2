// VL53L8CX：起動時の begin()・is_alive()・init() の戻り値とかかった時間を測る（スプリント3.6-1。単体試験）
// ライブラリは Nova の lib/STM32duino_VL53L8CX（IO_Read に上限を付けた版。NOVA_PATCH.md）
// 見ること：
//   - begin・is_alive・init の戻り値（0＝成功）と時間（ms）
//   - そのあと「loop に到達」が1秒ごとに出るか（センサーが応答しなくても止まらないこと）
// 試験用なので millis() で待つ。ライブラリの中には delay() がある（init の中の待ち）
#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

#define PIN_TOF_SDA   26        // ESP32 基板のヘッダー P3-10（カメラの SIOD だったピン）
#define PIN_TOF_SCL   27        // ESP32 基板のヘッダー P3-11（カメラの SIOC だったピン）
#define TOF_I2C_HZ    400000    // 400kHz。線が長い（13cm）ので、だめなら 100000 に下げて比べる
#define PIN_TOF_LPN   -1        // つながない

static VL53L8CX sensor(&Wire1, PIN_TOF_LPN);
static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    // シリアルモニタがつながるのを少し待つ
  }
  Serial.println("[tof] l8cx_init_time 開始");
  Serial.printf("[tof] Wire1 SDA=%d SCL=%d %luHz\n",
                PIN_TOF_SDA, PIN_TOF_SCL, (unsigned long)TOF_I2C_HZ);

  bool wireOk = Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);
  Serial.printf("[tof] Wire1.begin=%d タイムアウト=%ums\n", wireOk ? 1 : 0, Wire1.getTimeOut());

  uint32_t tb = millis();
  int beginStatus = sensor.begin();     // アドレスは既定（8ビットの書き方で 0x52＝7ビットで 0x29）
  uint32_t beginMs = millis() - tb;
  Serial.printf("[tof] begin=%d 時間=%lums\n", beginStatus, (unsigned long)beginMs);

  // センサーがいるかを確かめる（デバイスID 0xF0・リビジョン 0x0C なら alive=1）。
  // alive=0 でも、試験のため init() は呼ぶ（失敗で戻るまでの時間を見る）
  uint32_t ta = millis();
  uint8_t alive = 0;
  uint8_t aliveStatus = sensor.is_alive(&alive);
  uint32_t aliveMs = millis() - ta;
  Serial.printf("[tof] is_alive=%u alive=%u 時間=%lums\n", aliveStatus, alive, (unsigned long)aliveMs);

  uint32_t ti = millis();
  uint8_t initStatus = sensor.init();   // ここでファームウェア（約84KB）を送る
  uint32_t initMs = millis() - ti;
  Serial.printf("[tof] init=%u 時間=%lums\n", initStatus, (unsigned long)initMs);

  lastPrintMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.printf("[tof] loop に到達 t=%lums\n", (unsigned long)now);
  }
}
