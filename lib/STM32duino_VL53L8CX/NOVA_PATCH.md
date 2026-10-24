# Nova による改変

公式 STM32duino VL53L8CX 2.1.0（https://github.com/stm32duino/VL53L8CX 、タグ 2.1.0＝コミット a93a9d6）を取り込んだもの。
取り込んだのは `src/`・`LICENSE`・`README.md`・`keywords.txt`・`library.properties`（`examples/` と `.github/` は除く）。

元のライブラリは、センサーが応答しないと `IO_Read` の中で永久に待つ（Nova の loop と安全層が止まる）。
`src/vl53l8cx.h` の `IO_Read`（I2C の分岐）だけを直した。改変箇所には `[Nova]` のコメントが付いている。

- レジスタのアドレスを送るループ：回数の上限 `NOVA_IO_READ_TX_TRIES`（既定 3。ビルドフラグで変えられる）を付けた。
  上限まで送って失敗したら 1 を返す。
  - Arduino-ESP32 2.x では `endTransmission(false)` は送らずに 0 を返す（送るのは次の `requestFrom`）ので、
    ESP32 ではこのループはくり返さない。他の環境向けの保険。
- 読み出し：`requestFrom` が返したバイト数が要求より少なければ、すぐに 1 を返す。
  32バイトずつ読む分岐（元は読めるまで永久にくり返す）と、1回で読む分岐の両方。
- エラーの値は元と同じ 1（元の `return i != size` と合わせた）。
- `IO_Write`・SPI の分岐・API（`vl53l8cx_api.c` など）は変えていない。
