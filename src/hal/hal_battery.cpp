#include "hal_battery.h"

#include "esp_adc_cal.h"

#include "hal_pins.h"

#define BATTERY_DEFAULT_VREF   1100   // 公式サンプルと同じ
#define BATTERY_DIVIDER_RATIO  4.0f   // 分圧比（公式サンプルの batteryCoefficient）
#define BATTERY_SAMPLE_COUNT   5

static esp_adc_cal_characteristics_t *adcChars = NULL;

void Battery_Setup(void) {
  analogSetWidth(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  adcChars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, BATTERY_DEFAULT_VREF, adcChars);
}

int Battery_ReadAdc(void) {
  long sum = 0;
  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    sum += analogRead(PIN_BATTERY);
  }
  return sum / BATTERY_SAMPLE_COUNT;
}

float Battery_AdcToVoltage(int adc) {
  uint32_t voltageAtPinMv = esp_adc_cal_raw_to_voltage(adc, adcChars);
  return (voltageAtPinMv / 1000.0f) * BATTERY_DIVIDER_RATIO;
}

float Battery_ReadVoltage(void) {
  return Battery_AdcToVoltage(Battery_ReadAdc());
}
