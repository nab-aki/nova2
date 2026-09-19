# Nova による改変

公式 Freenove_WS2812_Lib_for_ESP32 v2.0.1 は Arduino-ESP32 3.x 系の RMT API を使っている。
Nova は Arduino-ESP32 2.0.17（PlatformIO 標準）で動作確認しているため、
`src/` 内の2ファイルに `ESP_ARDUINO_VERSION_MAJOR` による分岐を追加した（3.x 側の元コードは残している）。
改変箇所には `[Nova]` のコメントが付いている。

- コンストラクタ：RMTメモリ量の指定（2.x は `RMT_MEM_64`）
- `begin()`：`rmtInit(pin, true, mem)` ＋ `rmtSetTick(100ns)`
- `show()`：`rmtWriteBlocking()`
- ヘッダ：2.x 用に `rmt_obj_t* rmt_send` メンバを追加
