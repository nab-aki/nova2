# 共通部品：首の調停（v1）

ID には紐づかない共通部品（#0）。ID9・ID15・ID16・ID25 が共通で使う。

## 目的

首（servo1 左右／servo2 上下）は、目と超音波センサーの両方を載せているため、
「安全のために正面へ固定したい」「測るために向けたい」「表情として動かしたい」が
同時に起きる。これを優先度で1つに決める。

設計原則2（CLAUDE.md）：
首の使用権は「安全＞測距＞表情」の順。走行中は首を正面・水平に固定し、
見回しは停止中（または減速中の短時間）のみ。首が動いている間と、
止まってから安定するまでの測距値は無効扱い。

## 使用権の優先度

| 持ち主 | 優先度 | 使う場面 |
|---|---|---|
| 安全（SAFETY） | 30 | 走行中に正面・水平へ固定する |
| 測距（RANGE） | 20 | 停止中の見回し（ID15・ID16） |
| 表情（EXPRESSION） | 10 | 首かしげ・見上げるなど（ID3・ID10） |

- 申し出（`Neck_Request`）は、自分より優先度の高い持ち主がいれば断られる（角度は変わらない）。
- 同じ持ち主からの申し出は常に通る。使い終わったら `Neck_Release` で返す。
- v1 で実際に使うのは安全だけ。測距・表情の枠はスプリント2以降のために用意する。

## 「首が安定した」の判定

測距値を信用してよいのは、首を動かす命令を出してから一定時間が経った後とする。

待ち時間 ＝ `NECK_SETTLE_BASE_MS` ＋ 動かした角度 × `NECK_SETTLE_PER_DEG_MS`

（動かした角度 ＝ 左右・上下の変化量の大きいほう）

### 実測との対応（docs/measurements.md：首の停止後の安定時間）

測定の 0ms は「正面へ戻す命令を出した時刻」なので、実測値は首が動く時間も含んでいる。

| 振り幅 | 実測 平均 | 実測 最長 | この式の値 |
|---|---|---|---|
| ±20° | 12ms | 60ms | 100ms |
| ±40° | 44ms | 60ms | 140ms |
| ±55° | 104ms | 160ms | 170ms |

- `NECK_SETTLE_BASE_MS = 60`：±20°・±40° の実測の最長がどちらも 60ms。
- `NECK_SETTLE_PER_DEG_MS = 2`：±55° で 60＋55×2＝170ms となり、実測の最長 160ms を上回る。
- ±20° は 5回中4回が「0ms」だったが、これは首が一瞬で止まった証拠ではない。
  超音波の指向角が広く、首を振っても同じ壁の最近点を見ていたため、値が変わらなかった
  可能性が高い。そのため小さい動きでも `BASE` の 60ms は必ず待つ。
- servo2（上下）の安定時間は未測定。当面は servo1 と同じ式を使う（スプリント2以降で確認）。

### 測距値の扱い

- 首が安定していない間に完了した測距は、障害物の判定に使わない。
- 首が動き始めたら、それまでの測距の履歴を捨てる（`Obstacle_Reset`）。

## API（src/core/neck.h）

```
void Neck_Setup(void);
bool Neck_Request(NeckOwner owner, int panDeg, int tiltDeg, unsigned long nowMs);
void Neck_Release(NeckOwner owner);
NeckOwner Neck_Owner(void);
const char *Neck_OwnerName(NeckOwner owner);
bool Neck_IsSteady(unsigned long nowMs);   // 首が止まっていて、測距値を信用してよいか
bool Neck_IsFront(void);                   // 正面・水平か（走り出す条件）
void Neck_Update(unsigned long nowMs);
```

- 角度の丸め（可動範囲）は今までどおり `hal/hal_servo.*` が行う。
- 現在と同じ角度を申し出たときは、首を動かさず、安定待ちもやり直さない
  （走行中は安全が毎ループ正面を申し出るため）。

## 調整パラメータ（config.h）

| 名前 | 初期値 | 意味 |
|---|---|---|
| SERVO2_LEVEL_DEG | 98 | 実測：超音波が水平になる servo2 の角度（90 から変更） |
| NECK_SETTLE_BASE_MS | 60 | 首を動かしてから必ず待つ時間 |
| NECK_SETTLE_PER_DEG_MS | 2 | 動かした角度1°あたりの追加の待ち時間 |
| NECK_PRIORITY_SAFETY | 30 | 使用権：安全 |
| NECK_PRIORITY_RANGE | 20 | 使用権：測距 |
| NECK_PRIORITY_EXPRESSION | 10 | 使用権：表情 |

## 完了条件

- 動作：走行中は首が正面・水平から動かない。首を動かした直後の測距値が判定に使われない。
- シリアルに、首の角度・使用権の持ち主・安定／動作中の別が出る。
