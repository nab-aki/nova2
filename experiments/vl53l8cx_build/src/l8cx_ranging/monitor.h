// ToF のモニタリング（見るだけ。スプリント3.6-2）：Wi-Fi・HTTP・WebSocket で 8×8 の表と状態をブラウザへ送る
// 測距の loop（コア1）は、ここの関数で最新の値を置くだけで、Wi-Fi を待たない。Wi-Fi と送信はコア0のタスクで動く
#ifndef L8CX_MONITOR_H
#define L8CX_MONITOR_H

#include <Arduino.h>

// loop のタスクから呼ぶ（モニタリングのタスクを作るだけで、すぐ戻る）。Wi-Fi は MONITOR_BASELINE_MS のあとで始める
void MonitorSetup(void);

// 成功したフレームを置く。dist・status は Nova の向き（y＝上から、x＝左から）で n×n 個。dataUs は get_ranging_data にかかった時間
void MonitorOnFrame(const int16_t *dist, const uint8_t *status, uint8_t n, uint32_t dataUs);

// 起動からの累計（loop のタスクだけが増やす。キー s で数え直しても消えない）
void MonitorCountReadyFail(void);
void MonitorCountDataFail(void);
void MonitorCountBusError(void);
void MonitorCountStall(void);

// 別のタスクから出たログの行（どのタスクからでも呼べる）
void MonitorCountOtherLog(void);

// 電池の電圧と、いま測距中か
void MonitorSetStatus(uint16_t batteryMv, bool tofOk);

// いまの状態（IP など）をシリアルに出す（キー w）
void MonitorPrintStatus(void);

#endif  // L8CX_MONITOR_H
