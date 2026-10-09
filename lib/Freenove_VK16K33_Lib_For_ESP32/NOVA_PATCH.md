# Nova による改変（I2C の失敗を数える）

公式 Freenove_VK16K33_Lib_For_ESP32 v1.0.0 は、I2C の送信結果（`Wire.endTransmission()` の戻り値）を捨てている。
捨てずに数えるようにした。動作（送る内容・順番・タイミング）は変えていない。
改変箇所には `[Nova]` のコメントが付いている。

理由：同じ I2C バスにジャイロを加えて4台になったので、通信の失敗を機器ごとに数えて、シリアルの `t` に出すため
（docs/specs/common_gyro_checklist.md「共有バスの対策」3）。集計は `src/hal/hal_i2c.*` に集める。

- `Freenove_VK16K33_Lib_For_ESP32.h`
  - public メンバを追加：`nova_i2c_total`・`nova_i2c_fail`・`nova_i2c_timeout`（`unsigned long`）
  - private 関数の宣言を追加：`novaCountTx()`（送信の結果を数える）
- `Freenove_VK16K33_Lib_For_ESP32.cpp`
  - `novaCountTx()` の定義を追加
  - `init()`（2種）・`setBrightness()`・`setBlink()`・`show()`：`Wire.endTransmission()` の戻り値を数える（各 送信1回）

数え方：

- `nova_i2c_total`：送信の回数。
- `nova_i2c_fail`：失敗の回数（`endTransmission()` の戻り値が 0 以外）。
- `nova_i2c_timeout`：そのうち、タイムアウトした回数（`endTransmission()` の戻り値 5）。

読む側は `src/hal/hal_matrix.cpp`。
