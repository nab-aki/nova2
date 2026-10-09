// Nova（Freenove 4WD Car Kit for ESP32 FNK0053 ベース）
// スプリント2：ID25「うろうろ」v1 と ID15「障害物で困る」最小版
//   ・目：2〜6秒のランダム間隔でまばたき（ときどき2回連続）
//   ・車体：止まって首で見回し、空いていそうな向きへ歩き、また止まる（ID25）
//   ・正面の障害物に気づいたら即停止し、目を見開く（ID9）
//   ・塞がったままなら、後退して左右を確かめ、その場回転で向きを変える（ID15）
//   ・進めていない（詰まった）と気づいたら、「？」の目で考えてから後退とその場回転で抜け出す（ID26）
//   ・持ち上げたら（ライントラッキング111）すぐモーターを止める（安全層）
//   ・シリアル（115200bps）に、測距値・判定・首の状態・モーター状態を出力する
//   ・シリアルの 3〜6 で、回転角の測定用に1ステップだけ回せる。7・8 は連続回転 90°（一時停止中だけ）。p でうろうろを一時停止／再開
//     一時停止中は q a w s e d r f で回転の調整値（時間・キック・PWM）を変えられる（? でキー一覧）
//
// 構成：hal/（ハードウェア操作）→ core/（センサー集約・首・障害物・安全・動き・表情・調停）
//       → behaviors/（振る舞い）
// delay() は使わず、loop() を回し続けて millis() で時間を管理する。

#include <Arduino.h>

#include "behaviors/blink.h"
#include "behaviors/debug_turn.h"
#include "behaviors/notice.h"
#include "behaviors/pause_cue.h"
#include "behaviors/stuck.h"
#include "behaviors/stuck_eyes.h"
#include "behaviors/trouble.h"
#include "behaviors/wander.h"
#include "config.h"
#include "core/arbiter.h"
#include "core/debug_pause.h"
#include "core/eyes.h"
#include "core/motion.h"
#include "core/neck.h"
#include "core/obstacle.h"
#include "core/safety.h"
#include "core/sensors.h"
#include "core/test_stats.h"
#include "core/trace.h"
#include "core/turn_tuning.h"
#include "hal/hal.h"
#include "hal/hal_i2c.h"
#include "hal/hal_log.h"

static BlinkBehavior blinkBehavior;
static NoticeBehavior noticeBehavior;
static NoticeEyesBehavior noticeEyesBehavior(&noticeBehavior);
static WanderBehavior wanderBehavior;
static TroubleBehavior troubleBehavior(&noticeBehavior);
static DebugTurnBehavior debugTurnBehavior;
static PauseCueBehavior pauseCueBehavior;
static StuckBehavior stuckBehavior;
static StuckEyesBehavior stuckEyesBehavior(&stuckBehavior);

static unsigned long lastStatusMs = 0;

// センサー値と状態をまとめて1行表示する
static void PrintStatus(unsigned long nowMs) {
  const SensorData &s = Sensors_Get();

  char distance[24];
  if (s.distanceValid) {
    snprintf(distance, sizeof(distance), "%.1fcm", s.distanceCm);
  } else if (s.distanceRawCm < 0) {
    snprintf(distance, sizeof(distance), "反応なし");
  } else {
    snprintf(distance, sizeof(distance), "範囲外(%.1fcm)", s.distanceRawCm);
  }

  // 接近速度（正＝近づいている）
  char approach[24];
  float cmPerSec = 0.0f;
  if (Obstacle_ApproachSpeed(&cmPerSec)) {
    snprintf(approach, sizeof(approach), "%+.1fcm/s", cmPerSec);
  } else {
    snprintf(approach, sizeof(approach), "-");
  }

  // 車体の出力（回転中は生のPWM、直進中は正規化速度とPWM）
  char drive[40];
  if (Motion_IsTurning()) {
    snprintf(drive, sizeof(drive), "%s", Motion_TurnName(Motion_GetTurnKind()));
  } else {
    snprintf(drive, sizeof(drive), "速度:%.2f(PWM %d)",
             Motion_GetSpeed(), Motor_SpeedToPwm(Motion_GetSpeed()));
  }

  Log_Printf("状態",
             "距離:%s 障害物:%s(近%d/%d) 接近:%s 光:%d ライン:%d%d%d(左中右) 電池:%.2fV(ADC %d) "
             "%s 首:%d/%d(%s,%s)%s%s 目:%s 車体:%s 目の振る舞い:%s 気づき停止:%d回",
             distance,
             Obstacle_IsBlocked() ? "あり" : "なし", Obstacle_NearCount(), Obstacle_SampleCount(),
             approach, s.lightAdc,
             s.track & 0x01, (s.track >> 1) & 0x01, (s.track >> 2) & 0x01,
             s.batteryV, s.batteryAdc,
             drive,
             Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT),
             Neck_OwnerName(Neck_Owner()), Neck_IsSteady(nowMs) ? "安定" : "動作中",
             Safety_IsStopping() ? " 安全:停止中" : "",
             Safety_IsTrackLost() ? " ライン読めず" :
             (Safety_IsLifted() ? " 持ち上げ中" : (Pause_IsPaused() ? " 一時停止中" : "")),
             Eyes_Name(Eyes_Get()),
             Arbiter_ActiveName(LAYER_BODY), Arbiter_ActiveName(LAYER_EYES),
             noticeBehavior.stopCount());
}

// ------------------------ デバッグキー（回転角の測定用）------------------------ //

// 回転の調整キーの対象。最後に押した回転キー（3・4 ならその場回転、5・6 なら片側旋回）か、g で選んだほう
static bool tunePivot = false;

// 連続したその場回転で DEBUG_CONT_TURN_DEG 回る時間（ID26 と同じ係数）
static unsigned long ContTurnMs(void) {
  return (unsigned long)(DEBUG_CONT_TURN_DEG * ROTATE_CONT_MS_PER_DEG + 0.5f);
}

static void PrintKeyHelp(void) {
  Log_Printf("キー", "3:その場回転 左  4:その場回転 右（各 %dms）", TurnTuning_StepMs(TURN_ROTATE_LEFT));
  Log_Printf("キー", "5:片側旋回 左  6:片側旋回 右（各 %dms）", TurnTuning_StepMs(TURN_PIVOT_LEFT));
  Log_Printf("キー", "いずれも1ステップだけ回します。止まっているときだけ受け付けます。終わるとステップ中の電池の最低値も出します");
  Log_Printf("キー", "7:連続その場回転 左  8:連続その場回転 右（%d°のつもりで %lums。一時停止中だけ。回った角度を測って ROTATE_CONT_MS_PER_DEG を直す）",
             DEBUG_CONT_TURN_DEG, ContTurnMs());
  Log_Printf("キー", "p か リモコンの ▶:うろうろの一時停止／再開（一時停止中は うろうろ・困る が止まり、3〜6 で落ち着いて測れます）");
  Log_Printf("キー", "  切り替わると目で合図します（一時停止＝目を細める、再開＝ゆっくり閉じて開く）");
  Log_Printf("キー", "t:試験の集計と、落ちる直前の流れ（足あと）を表示  h:試験の集計を今すぐ保存（止まっているときだけ）  x:集計と足あとを消去（5秒以内に2回）");
  Log_Printf("キー", "回転の調整（一時停止中だけ。押すたびに値と config.h 用の #define を出します。書き込み直すと元に戻ります）：");
  Log_Printf("キー", "  q/a:1ステップの時間 ±%dms（%d〜%d）  w/s:キックの時間 ±%dms（%d〜%d、0でキックなし）",
             DEBUG_TUNE_STEP_MS_STEP, DEBUG_TUNE_STEP_MS_MIN, DEBUG_TUNE_STEP_MS_MAX,
             DEBUG_TUNE_KICK_MS_STEP, DEBUG_TUNE_KICK_MS_MIN, DEBUG_TUNE_KICK_MS_MAX);
  Log_Printf("キー", "  e/d:キックのPWM ±%d  r/f:保持のPWM ±%d（どちらも%d〜%d）",
             DEBUG_TUNE_PWM_STEP, DEBUG_TUNE_PWM_STEP, DEBUG_TUNE_PWM_MIN, DEBUG_TUNE_PWM_MAX);
  Log_Printf("キー", "  g:調整の対象を切り替え（その場回転⇔片側旋回。3〜6 を押してもそのグループになる）  v:両方の現在の値を表示");
  Log_Printf("キー", "現在：%s／調整の対象：%s", Pause_IsPaused() ? "一時停止中" : "うろうろ中",
             tunePivot ? "片側旋回" : "その場回転");
  TurnTuning_Print(false, false);
  TurnTuning_Print(true, false);
}

// 止まっていて、立て直しの最中でも持ち上げ中でもないときだけ受け付ける
static void RequestDebugTurn(TurnKind kind, unsigned long durationMs) {
  if (debugTurnBehavior.isBusy()) {
    Log_Printf("キー", "回転の最中（または予約済み）なので無視します");
    return;
  }
  if (Safety_IsLifted()) {
    Log_Printf("キー", "%s", Safety_IsTrackLost() ? "ライントラッキングが読めず止めているので無視します"
                                                  : "持ち上げられているので無視します");
    return;
  }
  if (troubleBehavior.isBusy()) {
    Log_Printf("キー", "立て直し（困る）の最中なので無視します");
    return;
  }
  if (stuckBehavior.isBusy()) {
    Log_Printf("キー", "詰まり脱出の最中なので無視します");
    return;
  }
  if (Motion_IsTurning() ||
      fabsf(Motion_GetSpeed()) >= MOTOR_SPEED_EPSILON ||
      fabsf(Motion_GetTarget()) >= MOTOR_SPEED_EPSILON) {
    Log_Printf("キー", "車体が動いているので無視します（止まってから押してください）");
    return;
  }
  debugTurnBehavior.request(kind, durationMs);
}

// 連続回転（キー 7・8）。一時停止中だけ受け付ける
static void RequestContTurn(TurnKind kind) {
  if (!Pause_IsPaused()) {
    Log_Printf("キー", "連続回転は一時停止中（p）にしてから押してください");
    return;
  }
  RequestDebugTurn(kind, ContTurnMs());
}

// 回転の調整キー。一時停止中だけ効き、回転の最中は受け付けない
static void TuneKey(TurnTuneItem item, int direction) {
  if (!Pause_IsPaused()) {
    Log_Printf("調整", "一時停止中（p）にしてから押してください");
    return;
  }
  if (debugTurnBehavior.isBusy() || Motion_IsTurning()) {
    Log_Printf("調整", "回転の最中なので無視します");
    return;
  }
  if (!TurnTuning_Adjust(tunePivot, item, direction)) {
    Log_Printf("調整", "範囲の端です");
  }
  TurnTuning_Print(tunePivot, true);
}

static bool IsTuneKey(char c) {
  return c == 'q' || c == 'a' || c == 'w' || c == 's' || c == 'e' || c == 'd' || c == 'r' || c == 'f';
}

// 一時停止と再開を切り替え、目で合図する（キー p とリモコンの ▶ の共通の入口）
static void TogglePause(const char *source, unsigned long nowMs) {
  bool paused = Pause_Toggle(source);
  pauseCueBehavior.trigger(paused);
  TestStats_OnPauseToggle(paused, nowMs);
}

// I2C の失敗（機器ごと）と、ライントラッキングが読めずに止めた回数。起動からの累計で、保存はしない
static void PrintBusStats(void) {
  I2c_PrintStats();
  Log_Printf("I2C", "ライントラッキングが読めずに止めた回数：%lu回（%d回続けて読めないと止める。持ち上げとは別に数える）%s",
             (unsigned long)Safety_TrackLostCount(), SAFETY_TRACK_FAIL_COUNT,
             Safety_IsTrackLost() ? "【いま止めています】" : "");
}

static void HandleSerialKeys(void) {
  static char lastKey = 0;
  static unsigned long lastKeyMs = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      continue;   // 改行は無視（端末が自動で付ける場合がある）。連打の判定にも使わない
    }
    // キーを押しっぱなしにしたときの自動リピート（数十ms間隔）と、指のバウンスを無視する。
    // 同じキーが DEBUG_KEY_REPEAT_MS 以内に続いたら、リピートとみなして捨てる
    // （1ステップだけ回す・一時停止を切り替える、が押しっぱなしで繰り返されないように）。
    unsigned long now = millis();
    // 調整キー（q a w s e d r f）は、連続で押して値を動かしたいので、リピート判定の間隔を短くする
    unsigned long repeatMs = IsTuneKey(c) ? DEBUG_TUNE_REPEAT_MS : DEBUG_KEY_REPEAT_MS;
    bool repeat = (c == lastKey) && (now - lastKeyMs < repeatMs);
    lastKey = c;
    lastKeyMs = now;   // リピートが続く間は延長して、途切れるまで捨て続ける
    if (repeat) {
      continue;
    }
    switch (c) {
      case '3': tunePivot = false; RequestDebugTurn(TURN_ROTATE_LEFT, TurnTuning_StepMs(TURN_ROTATE_LEFT));   break;
      case '4': tunePivot = false; RequestDebugTurn(TURN_ROTATE_RIGHT, TurnTuning_StepMs(TURN_ROTATE_RIGHT)); break;
      case '5': tunePivot = true;  RequestDebugTurn(TURN_PIVOT_LEFT, TurnTuning_StepMs(TURN_PIVOT_LEFT));     break;
      case '6': tunePivot = true;  RequestDebugTurn(TURN_PIVOT_RIGHT, TurnTuning_StepMs(TURN_PIVOT_RIGHT));   break;
      case '7': tunePivot = false; RequestContTurn(TURN_ROTATE_LEFT);  break;
      case '8': tunePivot = false; RequestContTurn(TURN_ROTATE_RIGHT); break;
      case 'p': TogglePause("キー p", now); break;
      case 'q': TuneKey(TUNE_STEP_MS, +1);  break;
      case 'a': TuneKey(TUNE_STEP_MS, -1);  break;
      case 'w': TuneKey(TUNE_KICK_MS, +1);  break;
      case 's': TuneKey(TUNE_KICK_MS, -1);  break;
      case 'e': TuneKey(TUNE_KICK_PWM, +1); break;
      case 'd': TuneKey(TUNE_KICK_PWM, -1); break;
      case 'r': TuneKey(TUNE_HOLD_PWM, +1); break;
      case 'f': TuneKey(TUNE_HOLD_PWM, -1); break;
      case 'g':
        tunePivot = !tunePivot;
        Log_Printf("調整", "調整の対象を「%s」にしました", tunePivot ? "片側旋回" : "その場回転");
        TurnTuning_Print(tunePivot, false);
        break;
      case 'v':
        TurnTuning_Print(false, true);
        TurnTuning_Print(true, true);
        break;
      case 't':
        TestStats_Print(now);
        Trace_Print();
        PrintBusStats();
        break;
      case 'h': TestStats_RequestSave(now); break;
      case 'x': TestStats_HandleClearKey(now); break;
      case '?': PrintKeyHelp(); break;
      case '\r':
      case '\n':
        break;
      default:
        Log_Printf("キー", "不明なキー '%c'（? で一覧）", c);
        break;
    }
  }
}

void setup() {
  Log_Setup();
  Log_Printf("起動", "Nova スプリント3（ID25 うろうろ・ID15 障害物で困る・ID26 詰まり脱出）");

  uint8_t resetCode = Reset_ReasonCode();
  Log_Printf("起動", "リセット理由：%s%s", Reset_ReasonName(resetCode),
             Reset_IsBrownout(resetCode) ? "（電池切れ・電圧低下の疑い。試験の集計にも残ります）" : "");

  // 前回のブートの足あとを取り出す（RTCメモリ。ブラウンアウトでは消えない）。
  // 落ちていたときは、何をしている最中だったかをここで出し、モーターが止まっている今のうちに
  // NVS へも写す（RAM の控えは、次のリセット＝モニタを開いたときの DTR/RTS などで消えるため）
  Trace_Setup(resetCode, millis());
  if (Reset_IsBrownout(resetCode)) {
    Trace_PrintPrevious("落ちる直前の流れ");
  }

  randomSeed(esp_random());

  bool trackOk = Hal_Setup();
  if (!trackOk) {
    Log_Printf("起動", "ライントラッキングセンサーが応答しません（起動は続けます）");
  }

  Pause_Setup(DEBUG_START_PAUSED != 0);
  TestStats_Setup(DEBUG_START_PAUSED != 0, millis());
  Sensors_Setup();
  Neck_Setup();
  Obstacle_Setup();
  Safety_Setup();
  Eyes_Setup();
  Motion_Setup();

  // 登録順は同順位のときの優先順。優先度は config.h の PRIORITY_* で決まる
  Arbiter_Register(&debugTurnBehavior);
  Arbiter_Register(&troubleBehavior);
  Arbiter_Register(&stuckBehavior);
  Arbiter_Register(&noticeBehavior);
  Arbiter_Register(&wanderBehavior);
  Arbiter_Register(&pauseCueBehavior);
  Arbiter_Register(&stuckEyesBehavior);
  Arbiter_Register(&noticeEyesBehavior);
  Arbiter_Register(&blinkBehavior);

  Log_Printf("起動", "初期化完了。停止閾値%.0fcm・巡航PWM%d・見回し±%d°で、%s",
             OBSTACLE_STOP_CM, Motor_SpeedToPwm(CRUISE_SPEED), WANDER_SCAN_PAN_DEG,
             Pause_IsPaused() ? "一時停止から始めます（p で再開）" : "うろうろを始めます");
  PrintKeyHelp();
}

void loop() {
  unsigned long now = millis();

  HandleSerialKeys();
  uint32_t irCode;
  if (Ir_Poll(&irCode) && irCode == IR_BUTTON_PAUSE) {   // 受信したボタンは hal_ir が1行出す
    TogglePause("リモコン ▶", now);
  }
  Buzzer_Update(now);
  Sensors_Update(now);
  Neck_Update(now);
  Obstacle_Update(Sensors_Get(), now);

  Arbiter_Update(Sensors_Get(), now);   // 振る舞いが車体・目・首を動かす
  Safety_Update(Sensors_Get(), now);    // 安全層が最後に上書きする

  Eyes_Update(now);
  Motion_Update(now);
  Trace_Update(now);       // 足あとに「ここまで生きていた」と、いまのモーターの出力を刻む
  TestStats_Update(now);   // 止まっているときだけ、試験の集計をフラッシュへ保存する

  if (now - lastStatusMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusMs = now;
    PrintStatus(now);
  }
}
