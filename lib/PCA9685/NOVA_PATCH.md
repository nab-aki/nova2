# Nova による改変（I2C の失敗を数える）

公式 PCA9685 v3.0.3 は、I2C の通信結果（`endTransmission()`・`requestFrom()` の戻り値）を捨てている。
捨てずに数えるようにした。動作（送る内容・順番・タイミング）は変えていない。
改変箇所には `[Nova]` のコメントが付いている。

理由：同じ I2C バスにジャイロを加えて4台になったので、通信の失敗を機器ごとに数えて、シリアルの `t` に出すため
（docs/specs/common_gyro_checklist.md「共有バスの対策」3）。集計は `src/hal/hal_i2c.*` に集める。

- `src/PCA9685.h`
  - public メンバを追加：`nova_i2c_total`・`nova_i2c_fail`・`nova_i2c_timeout`（`uint32_t`）
  - private 関数を追加：`novaCountTx()`（送信の結果を数える）・`novaCountRx()`（受信の結果を数える）
- `src/PCA9685/PCA9685Definitions.h`
  - `write()`：`endTransmission()` の戻り値を数える（送信1回）
  - `read()`：`endTransmission()` と `requestFrom()` の戻り値を数える（送信1回＋受信1回）
- `src/PCA9685/PCA9685.cpp`
  - `resetAllDevices()`：`endTransmission()` の戻り値を数える（送信1回）

数え方：

- `nova_i2c_total`：通信の回数（送信1回・受信1回をそれぞれ1と数える）。
- `nova_i2c_fail`：失敗の回数。送信は `endTransmission()` の戻り値が 0 以外、受信は頼んだバイト数を受け取れなかったとき。
- `nova_i2c_timeout`：そのうち、送信がタイムアウトした回数（`endTransmission()` の戻り値 5）。
  受信は失敗の理由が分からないので、ここには入らない。

読む側は `src/hal/hal_pca9685.cpp`。
