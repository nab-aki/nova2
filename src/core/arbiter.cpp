#include "arbiter.h"

#include "../hal/hal_log.h"

static Behavior *behaviors[ARBITER_MAX_BEHAVIORS];
static uint8_t behaviorCount = 0;
static Behavior *active[LAYER_COUNT] = {NULL, NULL};

static const char *const LAYER_NAMES[LAYER_COUNT] = {"車体", "目"};

bool Arbiter_Register(Behavior *behavior) {
  if (behavior == NULL || behaviorCount >= ARBITER_MAX_BEHAVIORS) {
    return false;
  }
  behaviors[behaviorCount++] = behavior;
  return true;
}

// そのレイヤーで発動を希望している振る舞いのうち、最優先のものを選ぶ
static Behavior *SelectBest(BehaviorLayer layer, const SensorData &sensors, unsigned long nowMs) {
  Behavior *best = NULL;
  int bestPriority = 0;
  for (uint8_t i = 0; i < behaviorCount; i++) {
    Behavior *b = behaviors[i];
    if (b->layer() != layer) {
      continue;
    }
    int p = b->priority(sensors, nowMs);
    // 優先度が高い、または同順位で実行中のもの（続行を優先）
    if (p > bestPriority || (p > 0 && p == bestPriority && b == active[layer])) {
      best = b;
      bestPriority = p;
    }
  }
  return best;
}

void Arbiter_Update(const SensorData &sensors, unsigned long nowMs) {
  for (uint8_t l = 0; l < LAYER_COUNT; l++) {
    BehaviorLayer layer = (BehaviorLayer)l;
    Behavior *best = SelectBest(layer, sensors, nowMs);

    if (best != active[layer]) {
      Log_Printf("調停", "%s: %s→%s", LAYER_NAMES[layer],
                 active[layer] ? active[layer]->name() : "なし",
                 best ? best->name() : "なし");
      if (active[layer]) {
        active[layer]->onStop(nowMs);
      }
      active[layer] = best;
      if (best) {
        best->onStart(nowMs);
      }
    }

    if (active[layer]) {
      active[layer]->onUpdate(sensors, nowMs);
    }
  }
}

const char *Arbiter_ActiveName(BehaviorLayer layer) {
  return active[layer] ? active[layer]->name() : "なし";
}
