#include "hal_reset.h"

#include <esp_system.h>

void Reset_Setup(void) {
  // 何もしない（esp_reset_reason() はいつでも読める）
}

uint8_t Reset_ReasonCode(void) {
  return (uint8_t)esp_reset_reason();
}

const char *Reset_ReasonName(uint8_t code) {
  switch ((esp_reset_reason_t)code) {
    case ESP_RST_POWERON:   return "電源投入";
    case ESP_RST_EXT:       return "外部リセット";
    case ESP_RST_SW:        return "ソフトウェアリセット";
    case ESP_RST_PANIC:     return "パニック（例外）";
    case ESP_RST_INT_WDT:   return "割り込みウォッチドッグ";
    case ESP_RST_TASK_WDT:  return "タスクウォッチドッグ";
    case ESP_RST_WDT:       return "その他のウォッチドッグ";
    case ESP_RST_DEEPSLEEP: return "ディープスリープからの復帰";
    case ESP_RST_BROWNOUT:  return "ブラウンアウト（電圧低下）";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "不明";
  }
}

bool Reset_IsBrownout(uint8_t code) {
  return (esp_reset_reason_t)code == ESP_RST_BROWNOUT;
}
