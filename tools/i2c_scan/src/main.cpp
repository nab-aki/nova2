// I2C スキャン・ツール（Nova 本体とは独立）
//
// 目的：共有バス（Wire、GPIO13/14、100kHz）に、既存の3台とジャイロ（LSM6DSV16X）が見えるかを確かめる。
//   1. 0x08〜0x77 を2秒ごとにスキャンし、応答したアドレスを表示する
//   2. ジャイロ（0x6A または 0x6B）が見えたら WHO_AM_I（0x0F）を読み、0x70 かを表示する
//   3. WHO_AM_I を100回続けて読み、失敗の回数を表示する（ケーブルを軽く揺らして接触を見る）
//
// 安全：読むだけ。PCA9685 へは何も書かないので、モーター・首は動かない。
//       ジャイロのレジスタにも書き込まない（設定は変えない）。
// I2C の失敗は捨てずに、種類ごとに数えて表示する。
// 参考：秋月電子 AE-LSM6DSV16X のサンプルプログラム（アドレスの探し方・WHO_AM_I の読み方）
//
// シリアル：115200bps。キー操作はない（表示を見るだけ）。

#include <Arduino.h>
#include <Wire.h>

// ---- ピン・バス（本体の src/hal/hal_pins.h と同じ。Freenove 公式サンプルの定義） ----
#define PIN_I2C_SDA          13
#define PIN_I2C_SCL          14
#define I2C_FREQ_HZ          100000   // PCF8574 の上限に合わせて 100kHz

// ---- スキャンの設定 ----
#define SCAN_ADDR_FIRST      0x08     // 0x00〜0x07 は予約アドレス（0x00 は PCA9685 のソフトリセットにも使われる）
#define SCAN_ADDR_LAST       0x77
#define SCAN_INTERVAL_MS     2000
#define WHO_AM_I_REPEAT      100

// ---- ジャイロ（LSM6DSV16X） ----
#define LSM6_ADDR_SA0_GND    0x6A     // J3（「6A」刻印側）を短絡
#define LSM6_ADDR_SA0_VDD    0x6B     // J2（「6B」刻印側）を短絡
#define LSM6_REG_WHO_AM_I    0x0F
#define LSM6_WHO_AM_I_VALUE  0x70

// Wire.endTransmission() の戻り値（Arduino-ESP32 2.x）
//   0：成功（ACK）  2：アドレスに NACK（その番地には誰もいない）
//   1：データが長すぎる  3：データに NACK  4：その他のエラー  5：タイムアウト
#define I2C_TX_OK            0
#define I2C_TX_NACK_ADDR     2

// ---- 既知の機器 ----
struct KnownDevice {
  uint8_t addr;
  const char *name;
  bool required;        // 必ず見えるはずの機器か
  uint32_t missCount;   // スキャンで見えなかった回数（累計）
};

static KnownDevice sKnown[] = {
    {0x20, "PCF8574（ライン）", true, 0},
    {0x5F, "PCA9685（モーター・首）", true, 0},
    {0x71, "LEDマトリクス", true, 0},
    // PCA9685 は電源投入直後、All Call アドレス 0x70 にも応答する（本体は起動時に無効化する）。異常ではない
    {0x70, "PCA9685 の All Call", false, 0},
};
static const int KNOWN_COUNT = sizeof(sKnown) / sizeof(sKnown[0]);

// ---- 累計 ----
static uint32_t sScanCount = 0;
static uint32_t sGyroMissCount = 0;     // ジャイロが見えなかった回数
static uint32_t sBusErrorTotal = 0;     // NACK 以外の失敗（スキャン中）
static uint32_t sWhoReadTotal = 0;      // WHO_AM_I を読んだ回数
static uint32_t sWhoTxFailTotal = 0;    // レジスタ番号の送信に失敗
static uint32_t sWhoRxFailTotal = 0;    // 1バイト受け取れなかった
static uint32_t sWhoMismatchTotal = 0;  // 読めたが 0x70 ではなかった

static unsigned long sNextScanMs = 0;

static const char *TxErrorName(uint8_t code) {
  switch (code) {
    case 1: return "データが長すぎる";
    case 3: return "データに NACK";
    case 4: return "その他のエラー";
    case 5: return "タイムアウト";
    default: return "不明";
  }
}

static const char *DeviceName(uint8_t addr) {
  if (addr == LSM6_ADDR_SA0_GND || addr == LSM6_ADDR_SA0_VDD) return "LSM6DSV16X";
  for (int i = 0; i < KNOWN_COUNT; i++) {
    if (sKnown[i].addr == addr) return sKnown[i].name;
  }
  return "不明";
}

// 1バイト読む結果
enum ReadResult { READ_OK, READ_TX_FAIL, READ_RX_FAIL };

// レジスタを1バイト読む。失敗の種類を返す（txCode には endTransmission の戻り値を入れる）
static ReadResult ReadReg(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t *txCode) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  *txCode = Wire.endTransmission(false);   // リスタート条件で読みに移る
  if (*txCode != I2C_TX_OK) return READ_TX_FAIL;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return READ_RX_FAIL;
  *value = Wire.read();
  return READ_OK;
}

// WHO_AM_I を1回読んで表示し、続けて WHO_AM_I_REPEAT 回読んで失敗を数える
static void CheckWhoAmI(uint8_t addr) {
  uint8_t value = 0;
  uint8_t txCode = 0;
  ReadResult r = ReadReg(addr, LSM6_REG_WHO_AM_I, &value, &txCode);
  if (r == READ_TX_FAIL) {
    Serial.printf("[WHO_AM_I] 0x%02X：読めません（送信の失敗：%s、コード%u）\n", addr, TxErrorName(txCode), txCode);
  } else if (r == READ_RX_FAIL) {
    Serial.printf("[WHO_AM_I] 0x%02X：読めません（1バイト受け取れませんでした）\n", addr);
  } else if (value == LSM6_WHO_AM_I_VALUE) {
    Serial.printf("[WHO_AM_I] 0x%02X：0x%02X（正しい。LSM6DSV16X です）\n", addr, value);
  } else {
    Serial.printf("[WHO_AM_I] 0x%02X：0x%02X（0x%02X ではありません。別の部品か、読み取りの誤りです）\n",
                  addr, value, LSM6_WHO_AM_I_VALUE);
  }

  uint32_t txFail = 0, rxFail = 0, mismatch = 0;
  for (int i = 0; i < WHO_AM_I_REPEAT; i++) {
    r = ReadReg(addr, LSM6_REG_WHO_AM_I, &value, &txCode);
    if (r == READ_TX_FAIL) txFail++;
    else if (r == READ_RX_FAIL) rxFail++;
    else if (value != LSM6_WHO_AM_I_VALUE) mismatch++;
  }
  sWhoReadTotal += WHO_AM_I_REPEAT;
  sWhoTxFailTotal += txFail;
  sWhoRxFailTotal += rxFail;
  sWhoMismatchTotal += mismatch;
  Serial.printf("[WHO_AM_I] %d回読み：失敗 %lu（送信 %lu・受信 %lu・値違い %lu）\n", WHO_AM_I_REPEAT,
                (unsigned long)(txFail + rxFail + mismatch), (unsigned long)txFail, (unsigned long)rxFail,
                (unsigned long)mismatch);
}

static void ScanOnce(void) {
  sScanCount++;
  bool found[SCAN_ADDR_LAST + 1] = {false};
  uint32_t busError = 0;

  Serial.printf("\n[スキャン %lu回目] 応答：", (unsigned long)sScanCount);
  int foundCount = 0;
  for (uint8_t addr = SCAN_ADDR_FIRST; addr <= SCAN_ADDR_LAST; addr++) {
    Wire.beginTransmission(addr);
    uint8_t code = Wire.endTransmission();
    if (code == I2C_TX_OK) {
      found[addr] = true;
      foundCount++;
      Serial.printf("0x%02X（%s） ", addr, DeviceName(addr));
    } else if (code != I2C_TX_NACK_ADDR) {
      // NACK 以外はバスの異常。捨てずに表示する
      busError++;
      Serial.printf("\n  【バスの異常】0x%02X：%s（コード%u） ", addr, TxErrorName(code), code);
    }
  }
  if (foundCount == 0) Serial.print("なし");
  Serial.println();
  sBusErrorTotal += busError;

  // 判定：既存の3台
  bool allRequired = true;
  for (int i = 0; i < KNOWN_COUNT; i++) {
    if (!sKnown[i].required) continue;
    if (!found[sKnown[i].addr]) {
      sKnown[i].missCount++;
      allRequired = false;
      Serial.printf("  【見えません】0x%02X %s\n", sKnown[i].addr, sKnown[i].name);
    }
  }

  // 判定：ジャイロ
  bool gyroGnd = found[LSM6_ADDR_SA0_GND];
  bool gyroVdd = found[LSM6_ADDR_SA0_VDD];
  if (!gyroGnd && !gyroVdd) {
    sGyroMissCount++;
    Serial.println("  【見えません】LSM6DSV16X（0x6A にも 0x6B にも応答なし）");
  } else if (gyroGnd && gyroVdd) {
    Serial.println("  【注意】0x6A と 0x6B の両方が応答しました。J2・J3 の状態を確認してください");
  }

  if (allRequired && (gyroGnd != gyroVdd) && busError == 0) {
    Serial.println("  判定：4台そろっています");
  }

  if (gyroGnd) CheckWhoAmI(LSM6_ADDR_SA0_GND);
  if (gyroVdd) CheckWhoAmI(LSM6_ADDR_SA0_VDD);

  // 累計（ケーブルを揺らしたときに、一瞬の失敗を見逃さないため）
  Serial.printf("[累計] スキャン %lu回 ／ 見えなかった回数：", (unsigned long)sScanCount);
  for (int i = 0; i < KNOWN_COUNT; i++) {
    if (!sKnown[i].required) continue;
    Serial.printf("0x%02X=%lu ", sKnown[i].addr, (unsigned long)sKnown[i].missCount);
  }
  Serial.printf("ジャイロ=%lu ／ バスの異常 %lu件 ／ WHO_AM_I 失敗 %lu/%lu（送信 %lu・受信 %lu・値違い %lu）\n",
                (unsigned long)sGyroMissCount, (unsigned long)sBusErrorTotal,
                (unsigned long)(sWhoTxFailTotal + sWhoRxFailTotal + sWhoMismatchTotal),
                (unsigned long)sWhoReadTotal, (unsigned long)sWhoTxFailTotal, (unsigned long)sWhoRxFailTotal,
                (unsigned long)sWhoMismatchTotal);
}

void setup() {
  Serial.begin(115200);
  bool ok = Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Serial.println("\n\n=== I2C スキャン・ツール ===");
  Serial.printf("Wire：SDA=GPIO%d、SCL=GPIO%d、%dHz（開始：%s）\n", PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ,
                ok ? "成功" : "失敗");
  Serial.println("期待：0x20・0x5F・0x71 と、ジャイロ（0x6A または 0x6B）の4台。");
  Serial.println("      0x70 は PCA9685 の All Call で、電源投入直後は応答します（異常ではありません）。");
  Serial.printf("%dms ごとにスキャンします。読むだけで、モーター・首は動かしません。\n", SCAN_INTERVAL_MS);
  sNextScanMs = millis();
}

void loop() {
  unsigned long now = millis();
  if ((long)(now - sNextScanMs) >= 0) {
    sNextScanMs = now + SCAN_INTERVAL_MS;
    ScanOnce();
  }
}
