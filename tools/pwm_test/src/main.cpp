// Freenove 4WD Car Kit for ESP32（FNK0053）
// モーター最低PWM値の測定用ツール（本体プロジェクトとは独立）
//
// モーター駆動・電池電圧読み取りの定義は、Freenove公式リポジトリの
// サンプルスケッチに従っている。
//   https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32
//   Sketches/01.1_Car_Move_and_Turn （モーター制御）
//   Sketches/01.4_Battery_level     （電池電圧読み取り）
//
// シリアル（115200bps）からの1文字コマンドで、前進・後退・回転の
// 各モードでのPWM値を変えながら、モーターが実際に動き出す
// 最低PWM値を確認するためのツール。

#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
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

void setup() {
  Serial.begin(115200);
  Motor_Setup();
  Battery_Setup();
  Motor_Move(0, 0, 0, 0);
  lastInputMillis = millis();
  PrintHelp();
  PrintStatus();
}

void loop() {
  if (Serial.available() > 0) {
    HandleCommand((char)Serial.read());
  }

  // 安全のため、30秒間入力がなければ自動停止する
  if (currentMode != MODE_STOP && (millis() - lastInputMillis >= AUTO_STOP_MS)) {
    currentMode = MODE_STOP;
    pwmValue = 0;
    ApplyMotor();
    Serial.println("30秒間入力がなかったため自動停止しました");
    PrintStatus();
    lastInputMillis = millis();
  }
}
