// Freenove 4WD Car Kit for ESP32（FNK0053）
// モーター最低PWM値・首サーボ角度の測定用ツール（本体プロジェクトとは独立）
//
// モーター駆動・電池電圧読み取り・サーボ駆動の定義は、Freenove公式リポジトリの
// サンプルスケッチに従っている。
//   https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32
//   Sketches/01.1_Car_Move_and_Turn   （モーター制御）
//   Sketches/01.2_Servo               （サーボ制御）
//   Sketches/01.4_Battery_level       （電池電圧読み取り）
//   Sketches/02.1_Ultrasonic_Ranging  （超音波測距）
//   Sketches/04.1_Tracking_Sensor     （ライントラッキング）
//
// シリアル（115200bps）からの1文字コマンドで、
// ・モーター測定モード：前進・後退・回転の各モードでのPWM値を変えながら、
//   モーターが実際に動き出す最低PWM値を確認する
// ・サーボ測定モード：首サーボ（左右・上下）を1°単位で動かし、
//   可動範囲や基準角度を確認する
// ・超音波測定モード：一定間隔で測距し、値のばらつきや無反応の頻度を確認する
// ・ライントラッキング測定モード：左・中央・右の値と変化・最小/最大を確認する
// ・電池電圧測定モード：200msごとに読み、1秒ごとに 現在値/最小値/平均 を表示する
//   （モーター・サーボの表示にも現在の電池電圧を併記する）
// の5モードを切り替えて使う。

#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
#include <PCF8574.h>
#include "esp_adc_cal.h"

// ------------------------ モーター（PCA9685経由）------------------------ //
#define PCA9685_SDA       13     // ESP32側 I2C SDA
#define PCA9685_SCL       14     // ESP32側 I2C SCL
#define PCA9685_ADDRESS   0x5F
#define MOTOR_FREQUENCY   1000   // モーター駆動のPWM周波数
#define PIN_MOTOR_M1_IN1  15
#define PIN_MOTOR_M1_IN2  14
#define PIN_MOTOR_M2_IN1  9
#define PIN_MOTOR_M2_IN2  8
#define PIN_MOTOR_M3_IN1  12
#define PIN_MOTOR_M3_IN2  13
#define PIN_MOTOR_M4_IN1  10
#define PIN_MOTOR_M4_IN2  11
#define MOTOR_SPEED_MIN   -4095  // PCA9685のパルス幅（公式サンプル準拠）の下限
#define MOTOR_SPEED_MAX   4095   // 同上限

PCA9685 pca9685;

// PCA9685の全体アドレス（MODE1レジスタ）を初期化する
// （公式サンプル Sketches/01.4_Battery_level の PCA9685_Close_Com_Address と同じ処理）
void PCA9685_Close_Com_Address(void) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

void Motor_Setup(void) {
  Wire.begin(PCA9685_SDA, PCA9685_SCL);
  pca9685.setupSingleDevice(Wire, PCA9685_ADDRESS);
  PCA9685_Close_Com_Address();
  pca9685.setToFrequency(MOTOR_FREQUENCY);
}

// 4輪のPWM（パルス幅）を指定して駆動する
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
  m1_speed = constrain(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m2_speed = constrain(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m3_speed = constrain(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m4_speed = constrain(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  if (m1_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, m1_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
  } else {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, -m1_speed);
  }
  if (m2_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, m2_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
  } else {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, -m2_speed);
  }
  if (m3_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, m3_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
  } else {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, -m3_speed);
  }
  if (m4_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, m4_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
  } else {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, -m4_speed);
  }
}

// ------------------------ 電池電圧（公式サンプル準拠）------------------------ //
#define PIN_BATTERY     32
#define DEFAULT_VREF    1100

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
static const float batteryCoefficient = 4;  // 分圧比（公式サンプルの初期値）

void Battery_Setup(void) {
  analogSetWidth(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

int Get_Battery_Voltage_ADC(void) {
  long batteryADC = 0;
  for (int i = 0; i < 5; i++) {
    batteryADC += analogRead(PIN_BATTERY);
  }
  return batteryADC / 5;
}

float Get_Battery_Voltage(void) {
  int batteryADC = Get_Battery_Voltage_ADC();
  uint32_t voltage_at_pin_mv = esp_adc_cal_raw_to_voltage(batteryADC, adc_chars);
  return (voltage_at_pin_mv / 1000.0f) * batteryCoefficient;
}

// ------------------------ 電池電圧測定モード ------------------------ //
#define BATTERY_READ_INTERVAL_MS   200
#define BATTERY_PRINT_INTERVAL_MS  1000UL

static float batteryLast = 0;
static float batteryMin = 0;
static double batterySum = 0;
static unsigned long batteryCount = 0;
static unsigned long lastBatteryReadMillis = 0;
static unsigned long lastBatteryPrintMillis = 0;

// 電圧を1回読み、現在値・最小値・平均用の合計を更新する
void BatteryReadSample(void) {
  batteryLast = Get_Battery_Voltage();
  if (batteryCount == 0 || batteryLast < batteryMin) batteryMin = batteryLast;
  batterySum += batteryLast;
  batteryCount++;
}

// 最小値と平均をリセットする
void ResetBatteryStats(void) {
  batteryCount = 0;
  batterySum = 0;
  batteryMin = 0;
}

void PrintBatterySummary(void) {
  if (batteryCount == 0) {
    Serial.println("電池電圧 (まだ計測データがありません)");
    return;
  }
  Serial.print("電池電圧 現在:"); Serial.print(batteryLast, 2);
  Serial.print("V 最小:"); Serial.print(batteryMin, 2);
  Serial.print("V 平均:"); Serial.print(batterySum / batteryCount, 2);
  Serial.print("V (n="); Serial.print(batteryCount); Serial.println(")");
}

void PrintBatteryHelp(void) {
  Serial.println("=== Nova 電池電圧測定ツール ===");
  Serial.println("200msごとに読み取り、1秒ごとに 現在値/最小値/平均 を表示します");
  Serial.println("r:最小値と平均をリセット");
  Serial.println("m:モーター測定モードへ  v:サーボ測定モードへ  u:超音波測定モードへ  l:ライントラッキング測定モードへ");
}

void HandleBatteryCommand(char c) {
  switch (c) {
    case 'r':
      ResetBatteryStats();
      Serial.println("電池電圧の最小値・平均をリセットしました");
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.print("不明なコマンド: '");
      Serial.print(c);
      Serial.println("'");
      break;
  }
}

// 一定間隔での読み取りと、1秒ごとの表示を行う（非ブロッキング）
void UpdateBatteryMeasurement(void) {
  unsigned long now = millis();

  if (now - lastBatteryReadMillis >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMillis = now;
    BatteryReadSample();
  }

  if (now - lastBatteryPrintMillis >= BATTERY_PRINT_INTERVAL_MS) {
    lastBatteryPrintMillis = now;
    PrintBatterySummary();
  }
}

// ------------------------ 首サーボ（PCA9685経由）------------------------ //
// チャンネル番号・角度→パルス幅の変換式は公式サンプル
// Sketches/01.2_Servo の Servo_1_Angle/Servo_2_Angle に準拠（50Hz動作）。
#define PCA9685_CHANNEL_0   0   // servo1（左右）
#define PCA9685_CHANNEL_1   1   // servo2（上下）
#define SERVO_FREQUENCY     50

#define SERVO1_MIN          0
#define SERVO1_MAX          180
#define SERVO1_CENTER       90
#define SERVO2_MIN          90
#define SERVO2_MAX          150
#define SERVO2_CENTER       120

#define SERVO_STEP_SMALL      1
#define SERVO_STEP_LARGE      5
#define SERVO_STEP_INTERVAL_MS 20 // 1°動かす間隔（急激に飛ばさないため）

enum ServoId { SERVO_1, SERVO_2 };

static ServoId selectedServo = SERVO_1;
static int servo1TargetAngle = SERVO1_CENTER;
static int servo1CurrentAngle = SERVO1_CENTER;
static int servo2TargetAngle = SERVO2_CENTER;
static int servo2CurrentAngle = SERVO2_CENTER;
static unsigned long lastServoStepMillis = 0;

// 角度（0〜180°）をPCA9685のパルス幅に変換して出力する
// （公式サンプルと同じ map(0-180 -> 102-512)）
void Servo_Write(PCA9685::Channel channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulseWidth = map(angle, 0, 180, 102, 512);
  pca9685.setChannelPulseWidth(channel, pulseWidth);
}

const char* ServoName(ServoId servo) {
  return (servo == SERVO_1) ? "servo1(左右)" : "servo2(上下)";
}

int ServoTargetAngle(ServoId servo) {
  return (servo == SERVO_1) ? servo1TargetAngle : servo2TargetAngle;
}

// 選択中のサーボの目標角度を、安全範囲内に収めて設定する
void SetServoTargetAngle(ServoId servo, int angle) {
  if (servo == SERVO_1) {
    servo1TargetAngle = constrain(angle, SERVO1_MIN, SERVO1_MAX);
  } else {
    servo2TargetAngle = constrain(angle, SERVO2_MIN, SERVO2_MAX);
  }
}

void PrintServoStatus(void) {
  Serial.print("選択中:");
  Serial.print(ServoName(selectedServo));
  Serial.print(" servo1:");
  Serial.print(servo1TargetAngle);
  Serial.print("度 servo2:");
  Serial.print(servo2TargetAngle);
  Serial.print("度 電池電圧:");
  Serial.print(Get_Battery_Voltage(), 2);
  Serial.println("V");
}

void PrintServoHelp(void) {
  Serial.println("=== Nova 首サーボ角度測定ツール ===");
  Serial.println("1:servo1(左右)選択  2:servo2(上下)選択");
  Serial.println("]:+1度  [:-1度  +:+5度  -:-5度");
  Serial.println("c:中央に戻す(servo1=90 servo2=120)  p:現在角度を表示");
  Serial.print("servo1可動範囲:"); Serial.print(SERVO1_MIN);
  Serial.print("〜"); Serial.println(SERVO1_MAX);
  Serial.print("servo2可動範囲:"); Serial.print(SERVO2_MIN);
  Serial.print("〜"); Serial.println(SERVO2_MAX);
  Serial.println("m:モーター測定モードに戻る  u:超音波測定モードへ  l:ライントラッキング測定モードへ  e:電池電圧測定モードへ");
}

void HandleServoCommand(char c) {
  bool changed = true;

  switch (c) {
    case '1': selectedServo = SERVO_1; break;
    case '2': selectedServo = SERVO_2; break;
    case ']': SetServoTargetAngle(selectedServo, ServoTargetAngle(selectedServo) + SERVO_STEP_SMALL); break;
    case '[': SetServoTargetAngle(selectedServo, ServoTargetAngle(selectedServo) - SERVO_STEP_SMALL); break;
    case '+': SetServoTargetAngle(selectedServo, ServoTargetAngle(selectedServo) + SERVO_STEP_LARGE); break;
    case '-': SetServoTargetAngle(selectedServo, ServoTargetAngle(selectedServo) - SERVO_STEP_LARGE); break;
    case 'c':
      servo1TargetAngle = SERVO1_CENTER;
      servo2TargetAngle = SERVO2_CENTER;
      break;
    case 'p':
      changed = false;
      PrintServoStatus();
      break;
    case '\r':
    case '\n':
      changed = false;
      break;
    default:
      changed = false;
      Serial.print("不明なコマンド: '");
      Serial.print(c);
      Serial.println("'");
      break;
  }

  if (changed) {
    PrintServoStatus();
  }
}

// 目標角度に向けて、1°ずつゆっくりサーボを動かす（非ブロッキング）
void UpdateServoMotion(void) {
  unsigned long now = millis();
  if (now - lastServoStepMillis < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStepMillis = now;

  if (servo1CurrentAngle != servo1TargetAngle) {
    servo1CurrentAngle += (servo1TargetAngle > servo1CurrentAngle) ? 1 : -1;
    Servo_Write(PCA9685_CHANNEL_0, servo1CurrentAngle);
  }
  if (servo2CurrentAngle != servo2TargetAngle) {
    servo2CurrentAngle += (servo2TargetAngle > servo2CurrentAngle) ? 1 : -1;
    Servo_Write(PCA9685_CHANNEL_1, servo2CurrentAngle);
  }
}

// ------------------------ 超音波センサー（公式サンプル準拠）------------------------ //
// ピン番号・測距計算式は公式サンプル Sketches/02.1_Ultrasonic_Ranging に準拠。
// センサーはPCA9685を経由せず、ESP32のGPIOに直結されている。
#define PIN_SONIC_TRIG      12
#define PIN_SONIC_ECHO      15
#define ULTRASONIC_MAX_DISTANCE_CM 300
#define SONIC_TIMEOUT_US    (ULTRASONIC_MAX_DISTANCE_CM * 60) // 公式サンプルと同じ計算
#define SOUND_VELOCITY_M_S  340

#define ULTRASONIC_HISTORY_SIZE          20
#define ULTRASONIC_INTERVAL_MIN_MS       50
#define ULTRASONIC_INTERVAL_STEP_MS      50
#define ULTRASONIC_INTERVAL_MAX_MS       450
#define ULTRASONIC_DEFAULT_INTERVAL_MS   100
#define ULTRASONIC_SUMMARY_INTERVAL_MS   1000UL

static float ultrasonicHistoryValue[ULTRASONIC_HISTORY_SIZE];
static bool ultrasonicHistoryValid[ULTRASONIC_HISTORY_SIZE];
static uint8_t ultrasonicHistoryCount = 0; // 蓄積件数（最大20で頭打ち）
static uint8_t ultrasonicHistoryIndex = 0; // 次に書き込む位置（循環）
static unsigned long ultrasonicIntervalMs = ULTRASONIC_DEFAULT_INTERVAL_MS;
static unsigned long ultrasonicNoResponseTotal = 0; // リセットからの累積回数
static unsigned long lastUltrasonicMeasureMillis = 0;
static unsigned long lastUltrasonicSummaryMillis = 0;

void Ultrasonic_Setup(void) {
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
}

// トリガーパルスを送り、エコーが返るまでの時間（μs）を返す。0なら反応なし。
// digitalWrite間の10μs待ちはHC-SR04のトリガー生成に必要な最小待ち時間であり、
// pulseIn自体もタイムアウト付きの待ちのため、いずれもdelay()には該当しない。
unsigned long Get_Sonar_PingTime(void) {
  digitalWrite(PIN_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  return pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT_US);
}

// 循環バッファに1件記録する（古い順で並ぶよう管理）
void UltrasonicRecord(bool valid, float distanceCm) {
  ultrasonicHistoryValid[ultrasonicHistoryIndex] = valid;
  ultrasonicHistoryValue[ultrasonicHistoryIndex] = distanceCm;
  ultrasonicHistoryIndex = (ultrasonicHistoryIndex + 1) % ULTRASONIC_HISTORY_SIZE;
  if (ultrasonicHistoryCount < ULTRASONIC_HISTORY_SIZE) {
    ultrasonicHistoryCount++;
  }
}

// 循環バッファ上で最も古いデータのインデックスを返す
uint8_t UltrasonicHistoryStart(void) {
  return (ultrasonicHistoryCount < ULTRASONIC_HISTORY_SIZE) ? 0 : ultrasonicHistoryIndex;
}

void ResetUltrasonicStats(void) {
  ultrasonicHistoryCount = 0;
  ultrasonicHistoryIndex = 0;
  ultrasonicNoResponseTotal = 0;
  lastUltrasonicSummaryMillis = millis();
}

// 直近20回のうち有効な値だけで 最小/最大/平均/ばらつき幅 を計算して1行表示する
void PrintUltrasonicSummary(void) {
  float minDistance = 0, maxDistance = 0, sum = 0;
  uint8_t validCount = 0;
  uint8_t start = UltrasonicHistoryStart();

  for (uint8_t i = 0; i < ultrasonicHistoryCount; i++) {
    uint8_t idx = (start + i) % ULTRASONIC_HISTORY_SIZE;
    if (!ultrasonicHistoryValid[idx]) continue;
    float d = ultrasonicHistoryValue[idx];
    if (validCount == 0 || d < minDistance) minDistance = d;
    if (validCount == 0 || d > maxDistance) maxDistance = d;
    sum += d;
    validCount++;
  }

  Serial.print("超音波 ");
  if (validCount == 0) {
    Serial.print("有効なデータなし ");
  } else {
    float average = sum / validCount;
    Serial.print("最小:"); Serial.print(minDistance, 1); Serial.print("cm ");
    Serial.print("最大:"); Serial.print(maxDistance, 1); Serial.print("cm ");
    Serial.print("平均:"); Serial.print(average, 1); Serial.print("cm ");
    Serial.print("ばらつき幅:"); Serial.print(maxDistance - minDistance, 1); Serial.print("cm ");
  }
  Serial.print("反応なし:"); Serial.print(ultrasonicNoResponseTotal); Serial.print("回 ");
  Serial.print("間隔:"); Serial.print(ultrasonicIntervalMs); Serial.println("ms");
}

// 直近の生データ（最大20件）を測定順にそのまま一覧表示する
void PrintUltrasonicRaw(void) {
  Serial.println("=== 直近の生データ ===");
  if (ultrasonicHistoryCount == 0) {
    Serial.println("(データがまだありません)");
    return;
  }
  uint8_t start = UltrasonicHistoryStart();
  for (uint8_t i = 0; i < ultrasonicHistoryCount; i++) {
    uint8_t idx = (start + i) % ULTRASONIC_HISTORY_SIZE;
    Serial.print(i + 1);
    Serial.print(":");
    if (ultrasonicHistoryValid[idx]) {
      Serial.print(ultrasonicHistoryValue[idx], 1);
      Serial.println("cm");
    } else {
      Serial.println("反応なし");
    }
  }
}

void PrintUltrasonicHelp(void) {
  Serial.println("=== Nova 超音波測定ツール ===");
  Serial.println("r:統計リセット  g:直近の生データを一覧表示");
  Serial.println("1〜9:測定間隔を50ms刻みで変更(50〜450ms)");
  Serial.print("現在の間隔:"); Serial.print(ultrasonicIntervalMs); Serial.println("ms");
  Serial.println("1秒ごとに直近20回の統計を表示します");
  Serial.println("m:モーター測定モードへ  v:サーボ測定モードへ  l:ライントラッキング測定モードへ  e:電池電圧測定モードへ");
}

void HandleUltrasonicCommand(char c) {
  if (c >= '1' && c <= '9') {
    ultrasonicIntervalMs = (unsigned long)(c - '0') * ULTRASONIC_INTERVAL_STEP_MS;
    Serial.print("測定間隔を"); Serial.print(ultrasonicIntervalMs); Serial.println("msに変更しました");
    return;
  }

  switch (c) {
    case 'r':
      ResetUltrasonicStats();
      Serial.println("超音波の統計をリセットしました");
      break;
    case 'g':
      PrintUltrasonicRaw();
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.print("不明なコマンド: '");
      Serial.print(c);
      Serial.println("'");
      break;
  }
}

// 一定間隔での測距と、1秒ごとの統計表示を行う（非ブロッキング）
void UpdateUltrasonicMeasurement(void) {
  unsigned long now = millis();

  if (now - lastUltrasonicMeasureMillis >= ultrasonicIntervalMs) {
    lastUltrasonicMeasureMillis = now;
    unsigned long pingTime = Get_Sonar_PingTime();
    if (pingTime == 0) {
      ultrasonicNoResponseTotal++;
      UltrasonicRecord(false, 0.0f);
    } else {
      float distanceCm = (float)pingTime * SOUND_VELOCITY_M_S / 2 / 10000.0f;
      UltrasonicRecord(true, distanceCm);
    }
  }

  if (now - lastUltrasonicSummaryMillis >= ULTRASONIC_SUMMARY_INTERVAL_MS) {
    lastUltrasonicSummaryMillis = now;
    PrintUltrasonicSummary();
  }
}

// ------------------------ ライントラッキングセンサー（公式サンプル準拠）------------------------ //
// PCF8574経由のI2C入力で、公式サンプル Sketches/04.1_Tracking_Sensor に準拠。
// このモジュールは比較器で白黒を判定したデジタル値(0/1)のみを返すため、
// アナログ値は取得できない（ハードウェア上の制約）。
#define PCF8574_ADDRESS       0x20
#define TRACK_LEFT            0
#define TRACK_MIDDLE          1
#define TRACK_RIGHT           2
#define TRACK_SENSOR_COUNT    3

#define TRACK_READ_INTERVAL_MS   100
#define TRACK_PRINT_INTERVAL_MS  200

PCF8574 trackSensor(PCF8574_ADDRESS);

static uint8_t trackValue[TRACK_SENSOR_COUNT] = {0, 0, 0};
static uint8_t trackPrevPrinted[TRACK_SENSOR_COUNT] = {0, 0, 0};
static uint8_t trackMin[TRACK_SENSOR_COUNT] = {0, 0, 0};
static uint8_t trackMax[TRACK_SENSOR_COUNT] = {0, 0, 0};
static bool trackStatsStarted = false;
static unsigned long lastTrackReadMillis = 0;
static unsigned long lastTrackPrintMillis = 0;

void Track_Setup(void) {
  // I2CバスはMotor_Setup()内のWire.begin()で初期化済み（同じSDA/SCLを共有）
  trackSensor.begin();
}

// 左・中央・右の生の値（0/1）を読み、最小/最大を更新する
void TrackReadSensors(void) {
  uint8_t raw = trackSensor.read8() & 0x07;
  trackValue[TRACK_LEFT]   = (raw & 0x01) >> 0;
  trackValue[TRACK_MIDDLE] = (raw & 0x02) >> 1;
  trackValue[TRACK_RIGHT]  = (raw & 0x04) >> 2;

  for (int i = 0; i < TRACK_SENSOR_COUNT; i++) {
    if (!trackStatsStarted || trackValue[i] < trackMin[i]) trackMin[i] = trackValue[i];
    if (!trackStatsStarted || trackValue[i] > trackMax[i]) trackMax[i] = trackValue[i];
  }
  trackStatsStarted = true;
}

void ResetTrackStats(void) {
  trackStatsStarted = false;
}

void PrintTrackStats(void) {
  if (!trackStatsStarted) {
    Serial.println("(まだ計測データがありません)");
    return;
  }
  Serial.print("直近の統計 左min:"); Serial.print(trackMin[TRACK_LEFT]);
  Serial.print(" max:"); Serial.print(trackMax[TRACK_LEFT]);
  Serial.print(" 中央min:"); Serial.print(trackMin[TRACK_MIDDLE]);
  Serial.print(" max:"); Serial.print(trackMax[TRACK_MIDDLE]);
  Serial.print(" 右min:"); Serial.print(trackMin[TRACK_RIGHT]);
  Serial.print(" max:"); Serial.println(trackMax[TRACK_RIGHT]);
}

// 左・中央・右を1行で表示する。前回表示時から変化していれば行末に*を付ける
void PrintTrackLine(void) {
  bool changed = (trackValue[TRACK_LEFT] != trackPrevPrinted[TRACK_LEFT]) ||
                 (trackValue[TRACK_MIDDLE] != trackPrevPrinted[TRACK_MIDDLE]) ||
                 (trackValue[TRACK_RIGHT] != trackPrevPrinted[TRACK_RIGHT]);

  Serial.print("左:"); Serial.print(trackValue[TRACK_LEFT]);
  Serial.print(" 中央:"); Serial.print(trackValue[TRACK_MIDDLE]);
  Serial.print(" 右:"); Serial.print(trackValue[TRACK_RIGHT]);
  Serial.println(changed ? " *" : "");

  trackPrevPrinted[TRACK_LEFT]   = trackValue[TRACK_LEFT];
  trackPrevPrinted[TRACK_MIDDLE] = trackValue[TRACK_MIDDLE];
  trackPrevPrinted[TRACK_RIGHT]  = trackValue[TRACK_RIGHT];
}

void PrintTrackHelp(void) {
  Serial.println("=== Nova ライントラッキング測定ツール ===");
  Serial.println("このセンサーはデジタル値(0/1)のみ取得可能です（アナログ値は非対応）");
  Serial.println("100msごとに読み取り、200msごとに1行表示します（変化した行は末尾に*）");
  Serial.println("r:直近の最小/最大を表示してリセット");
  Serial.println("m:モーター測定モードへ  v:サーボ測定モードへ  u:超音波測定モードへ  e:電池電圧測定モードへ");
}

void HandleTrackCommand(char c) {
  switch (c) {
    case 'r':
      PrintTrackStats();
      ResetTrackStats();
      Serial.println("ライントラッキングの統計をリセットしました");
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.print("不明なコマンド: '");
      Serial.print(c);
      Serial.println("'");
      break;
  }
}

// 一定間隔での読み取りと表示を行う（非ブロッキング）
void UpdateTrackMeasurement(void) {
  unsigned long now = millis();

  if (now - lastTrackReadMillis >= TRACK_READ_INTERVAL_MS) {
    lastTrackReadMillis = now;
    TrackReadSensors();
  }

  if (now - lastTrackPrintMillis >= TRACK_PRINT_INTERVAL_MS) {
    lastTrackPrintMillis = now;
    PrintTrackLine();
  }
}

// ------------------------ PWM測定ロジック ------------------------ //
#define PWM_LIMIT        2000    // 安全のためのPWM上限（依頼仕様）
#define PWM_STEP_LARGE   50
#define PWM_STEP_SMALL   10
#define AUTO_STOP_MS     30000UL // 30秒入力がなければ自動停止

enum Mode { MODE_STOP, MODE_FORWARD, MODE_BACKWARD, MODE_ROTATE };

static Mode currentMode = MODE_STOP;
static int pwmValue = 0;               // 0〜PWM_LIMIT
static unsigned long lastInputMillis = 0;

const char* ModeName(Mode mode) {
  switch (mode) {
    case MODE_FORWARD:  return "前進";
    case MODE_BACKWARD: return "後退";
    case MODE_ROTATE:   return "回転";
    default:            return "停止";
  }
}

// 現在のモード・PWM値をモーターに反映する
void ApplyMotor(void) {
  switch (currentMode) {
    case MODE_FORWARD:
      Motor_Move(pwmValue, pwmValue, pwmValue, pwmValue);
      break;
    case MODE_BACKWARD:
      Motor_Move(-pwmValue, -pwmValue, -pwmValue, -pwmValue);
      break;
    case MODE_ROTATE:
      // 左側（M1, M2）と右側（M3, M4）を逆回転させ、その場で回転させる
      Motor_Move(-pwmValue, -pwmValue, pwmValue, pwmValue);
      break;
    default:
      Motor_Move(0, 0, 0, 0);
      break;
  }
}

void PrintStatus(void) {
  Serial.print("モード:");
  Serial.print(ModeName(currentMode));
  Serial.print(" PWM:");
  Serial.print(pwmValue);
  Serial.print(" 電池電圧:");
  Serial.print(Get_Battery_Voltage(), 2);
  Serial.println("V");
}

void PrintHelp(void) {
  Serial.println("=== Nova モーター最低PWM測定ツール ===");
  Serial.println("f:前進モード  b:後退モード  r:回転モード  s:即停止");
  Serial.println("+:PWM+50  -:PWM-50  ]:PWM+10  [:PWM-10");
  Serial.print("PWM上限:");
  Serial.print(PWM_LIMIT);
  Serial.println("  30秒入力がないと自動停止します");
  Serial.println("v:首サーボ測定モードへ  u:超音波測定モードへ  l:ライントラッキング測定モードへ  e:電池電圧測定モードへ");
}

void HandleCommand(char c) {
  bool changed = true;

  switch (c) {
    case 'f': currentMode = MODE_FORWARD;  break;
    case 'b': currentMode = MODE_BACKWARD; break;
    case 'r': currentMode = MODE_ROTATE;   break;
    case 's':
      currentMode = MODE_STOP;
      pwmValue = 0;
      break;
    case '+': pwmValue = min(pwmValue + PWM_STEP_LARGE, PWM_LIMIT); break;
    case '-': pwmValue = max(pwmValue - PWM_STEP_LARGE, 0);         break;
    case ']': pwmValue = min(pwmValue + PWM_STEP_SMALL, PWM_LIMIT); break;
    case '[': pwmValue = max(pwmValue - PWM_STEP_SMALL, 0);         break;
    case '\r':
    case '\n':
      changed = false;
      break;
    default:
      changed = false;
      Serial.print("不明なコマンド: '");
      Serial.print(c);
      Serial.println("'");
      break;
  }

  if (changed) {
    lastInputMillis = millis();
    ApplyMotor();
    PrintStatus();
  }
}

// ------------------------ モード切り替え（モーター／サーボ）------------------------ //
// PCA9685はチップ全体で1つのPWM周波数しか持てないため、
// モーター測定（1000Hz）とサーボ測定（50Hz）は同時に使えない。
// モードを切り替えるたびに、周波数と信号を安全な状態にしてから切り替える。
enum TestMode { TEST_MODE_MOTOR, TEST_MODE_SERVO, TEST_MODE_ULTRASONIC, TEST_MODE_TRACK, TEST_MODE_BATTERY };
static TestMode testMode = TEST_MODE_MOTOR;

void EnterServoMode(void) {
  currentMode = MODE_STOP;
  pwmValue = 0;
  ApplyMotor();
  pca9685.setToFrequency(SERVO_FREQUENCY);
  // 現在の目標角度をすぐにサーボへ反映する（起動直後は中央）
  Servo_Write(PCA9685_CHANNEL_0, servo1CurrentAngle);
  Servo_Write(PCA9685_CHANNEL_1, servo2CurrentAngle);
  testMode = TEST_MODE_SERVO;
  lastServoStepMillis = millis();
  Serial.println("=== 首サーボ測定モードに切り替えました ===");
  PrintServoHelp();
  PrintServoStatus();
}

void EnterUltrasonicMode(void) {
  // 超音波センサーはPCA9685を使わないため、モーターの安全停止のみ行う
  currentMode = MODE_STOP;
  pwmValue = 0;
  ApplyMotor();
  ResetUltrasonicStats();
  testMode = TEST_MODE_ULTRASONIC;
  lastUltrasonicMeasureMillis = millis();
  Serial.println("=== 超音波測定モードに切り替えました ===");
  PrintUltrasonicHelp();
}

void EnterTrackMode(void) {
  // ライントラッキングセンサーはPCA9685を使わないため、モーターの安全停止のみ行う
  currentMode = MODE_STOP;
  pwmValue = 0;
  ApplyMotor();
  ResetTrackStats();
  trackPrevPrinted[TRACK_LEFT] = trackPrevPrinted[TRACK_MIDDLE] = trackPrevPrinted[TRACK_RIGHT] = 0;
  testMode = TEST_MODE_TRACK;
  lastTrackReadMillis = millis();
  lastTrackPrintMillis = millis();
  TrackReadSensors(); // 初回値をすぐ取得しておく
  Serial.println("=== ライントラッキング測定モードに切り替えました ===");
  PrintTrackHelp();
}

void EnterBatteryMode(void) {
  // 電池電圧はPCA9685を使わないため、モーターの安全停止のみ行う
  currentMode = MODE_STOP;
  pwmValue = 0;
  ApplyMotor();
  ResetBatteryStats();
  testMode = TEST_MODE_BATTERY;
  lastBatteryReadMillis = millis();
  lastBatteryPrintMillis = millis();
  BatteryReadSample(); // 初回値をすぐ取得しておく
  Serial.println("=== 電池電圧測定モードに切り替えました ===");
  PrintBatteryHelp();
}

void EnterMotorMode(void) {
  // サーボへの信号を止めてからモーター用周波数に戻す
  pca9685.setChannelPulseWidth(PCA9685_CHANNEL_0, 0);
  pca9685.setChannelPulseWidth(PCA9685_CHANNEL_1, 0);
  pca9685.setToFrequency(MOTOR_FREQUENCY);
  testMode = TEST_MODE_MOTOR;
  lastInputMillis = millis();
  Serial.println("=== モーター測定モードに戻りました ===");
  PrintHelp();
  PrintStatus();
}

void setup() {
  Serial.begin(115200);
  Motor_Setup();
  Battery_Setup();
  Ultrasonic_Setup();
  Track_Setup();
  Motor_Move(0, 0, 0, 0);
  lastInputMillis = millis();
  PrintHelp();
  PrintStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'v' && testMode != TEST_MODE_SERVO) {
      EnterServoMode();
    } else if (c == 'm' && testMode != TEST_MODE_MOTOR) {
      EnterMotorMode();
    } else if (c == 'u' && testMode != TEST_MODE_ULTRASONIC) {
      EnterUltrasonicMode();
    } else if (c == 'l' && testMode != TEST_MODE_TRACK) {
      EnterTrackMode();
    } else if (c == 'e' && testMode != TEST_MODE_BATTERY) {
      EnterBatteryMode();
    } else if (testMode == TEST_MODE_BATTERY) {
      HandleBatteryCommand(c);
    } else if (testMode == TEST_MODE_SERVO) {
      HandleServoCommand(c);
    } else if (testMode == TEST_MODE_ULTRASONIC) {
      HandleUltrasonicCommand(c);
    } else if (testMode == TEST_MODE_TRACK) {
      HandleTrackCommand(c);
    } else {
      HandleCommand(c);
    }
  }

  switch (testMode) {
    case TEST_MODE_MOTOR:
      // 安全のため、30秒間入力がなければ自動停止する
      if (currentMode != MODE_STOP && (millis() - lastInputMillis >= AUTO_STOP_MS)) {
        currentMode = MODE_STOP;
        pwmValue = 0;
        ApplyMotor();
        Serial.println("30秒間入力がなかったため自動停止しました");
        PrintStatus();
        lastInputMillis = millis();
      }
      break;
    case TEST_MODE_SERVO:
      UpdateServoMotion();
      break;
    case TEST_MODE_ULTRASONIC:
      UpdateUltrasonicMeasurement();
      break;
    case TEST_MODE_TRACK:
      UpdateTrackMeasurement();
      break;
    case TEST_MODE_BATTERY:
      UpdateBatteryMeasurement();
      break;
  }
}
