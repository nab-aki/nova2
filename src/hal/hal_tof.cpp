#include "hal_tof.h"

#include "../config.h"

#if TOF_ENABLE

#include <Wire.h>
#include <vl53l8cx.h>

#include "hal_pins.h"

static VL53L8CX sensor(&Wire1, -1);
static VL53L8CX_ResultsData results;   // 約1.4KB。スタックに置かない

// タスクと loop で共有する（どちらもコア1だが、タスクが割り込むので守る）
static portMUX_TYPE shareMux = portMUX_INITIALIZER_UNLOCKED;
static TofFrame shared;      // 最新の1枚
static TofStats stats;
static TofFrame work;        // タスクの作業用

// 失敗した段階を残す。戻り値が 0 なら true
static bool Step(const char *name, uint8_t status) {
  if (status == 0) {
    return true;
  }
  portENTER_CRITICAL(&shareMux);
  stats.failName = name;
  stats.failStep = status;
  stats.state = TOF_STATE_FAILED;
  portEXIT_CRITICAL(&shareMux);
  return false;
}

// 起動。書き込み直後に応答しないことがあるので is_alive は間をあけて試す。失敗したら init を呼ばない
// （センサーがないときの起動の遅れを避けるため。docs/specs/common_tof_checklist.md）
static bool StartSensor(void) {
  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);
  sensor.begin();

  uint8_t alive = 0;
  uint8_t status = 0;
  for (int i = 1; i <= TOF_ALIVE_TRIES; i++) {
    alive = 0;
    status = sensor.is_alive(&alive);   // 戻り値を先に見る（失敗すると alive は保証がない）
    portENTER_CRITICAL(&shareMux);
    stats.aliveTries = (uint8_t)i;
    stats.aliveStatus = status;
    stats.alive = alive;
    portEXIT_CRITICAL(&shareMux);
    if (status == 0 && alive != 0) {
      break;
    }
    if (i < TOF_ALIVE_TRIES) {
      vTaskDelay(pdMS_TO_TICKS(TOF_ALIVE_RETRY_MS));
    }
  }
  if (status != 0 || alive == 0) {
    // どちらの線が握られているかを残す（固まった件の手がかり。common_tof_checklist.md「起動時の注意」）
    int sda = digitalRead(PIN_TOF_SDA);
    int scl = digitalRead(PIN_TOF_SCL);
    portENTER_CRITICAL(&shareMux);
    stats.sdaLevel = sda;
    stats.sclLevel = scl;
    stats.failName = "is_alive";
    stats.failStep = status;
    stats.state = TOF_STATE_FAILED;
    portEXIT_CRITICAL(&shareMux);
    return false;
  }

  uint32_t t0 = millis();
  uint8_t initStatus = sensor.init();
  uint32_t initMs = millis() - t0;
  portENTER_CRITICAL(&shareMux);
  stats.initMs = initMs;
  portEXIT_CRITICAL(&shareMux);
  if (!Step("init", initStatus)) return false;
  if (!Step("set_resolution", sensor.set_resolution(VL53L8CX_RESOLUTION_8X8))) return false;
  if (!Step("set_ranging_frequency_hz", sensor.set_ranging_frequency_hz(TOF_FREQ_HZ))) return false;
  if (!Step("start_ranging", sensor.start_ranging())) return false;

  portENTER_CRITICAL(&shareMux);
  stats.state = TOF_STATE_RUNNING;
  portEXIT_CRITICAL(&shareMux);
  return true;
}

// 1枚読んで置く
static void ReadFrame(void) {
  uint32_t t0 = micros();
  uint8_t status = sensor.get_ranging_data(&results);
  uint32_t us = micros() - t0;
  if (status != 0) {
    portENTER_CRITICAL(&shareMux);
    stats.dataFail++;
    portEXIT_CRITICAL(&shareMux);
    return;
  }
  uint32_t now = millis();
  for (int z = 0; z < TOF_ZONES; z++) {
    work.distanceMm[z] = results.distance_mm[z * VL53L8CX_NB_TARGET_PER_ZONE];
    work.status[z] = results.target_status[z * VL53L8CX_NB_TARGET_PER_ZONE];
  }
  work.readyMs = now;
  work.seq++;

  uint32_t stackFree = uxTaskGetStackHighWaterMark(NULL);
  portENTER_CRITICAL(&shareMux);
  if (stats.frames > 0) {
    uint32_t period = now - shared.readyMs;
    stats.periodSumMs += period;
    stats.periodCount++;
    if (period > stats.periodMaxMs) stats.periodMaxMs = period;
  }
  shared = work;
  stats.frames++;
  stats.readSumUs += us;
  if (us > stats.readMaxUs) stats.readMaxUs = us;
  stats.stackFree = stackFree;
  portEXIT_CRITICAL(&shareMux);
}

static void TofTask(void *arg) {
  (void)arg;
  if (!StartSensor()) {
    vTaskDelete(NULL);   // この起動中は使わない。やり直さない
    return;
  }
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(TOF_POLL_MS));
    uint8_t ready = 0;
    uint8_t status = sensor.check_data_ready(&ready);
    if (status != 0) {
      portENTER_CRITICAL(&shareMux);
      stats.readyFail++;
      portEXIT_CRITICAL(&shareMux);
      continue;
    }
    if (ready) {
      ReadFrame();
    }
  }
}

void Tof_Setup(void) {
  stats = TofStats{};
  stats.state = TOF_STATE_STARTING;
  stats.sdaLevel = -1;
  stats.sclLevel = -1;
  xTaskCreatePinnedToCore(TofTask, "tof", TOF_TASK_STACK, NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE);
}

bool Tof_GetLatest(uint32_t lastSeq, TofFrame *out) {
  bool isNew = false;
  portENTER_CRITICAL(&shareMux);
  if (shared.seq != lastSeq) {
    *out = shared;
    isNew = true;
  }
  portEXIT_CRITICAL(&shareMux);
  return isNew;
}

void Tof_GetStats(TofStats *out) {
  portENTER_CRITICAL(&shareMux);
  *out = stats;
  portEXIT_CRITICAL(&shareMux);
}

#else  // TOF_ENABLE が 0：比べるための環境 fnk0053_notof

void Tof_Setup(void) {}

bool Tof_GetLatest(uint32_t lastSeq, TofFrame *out) {
  (void)lastSeq;
  (void)out;
  return false;
}

void Tof_GetStats(TofStats *out) {
  *out = TofStats{};
  out->state = TOF_STATE_DISABLED;
  out->sdaLevel = -1;
  out->sclLevel = -1;
}

#endif
