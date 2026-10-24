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
- `IO_Write`・SPI の分岐は変えていない。

## 読み出しに失敗したあとの解析を止める（`src/vl53l8cx_api.c`）

元のライブラリは、読み出しに失敗してもエラーを `status` にためるだけで、バッファに残った前の内容の解析を続ける。
`IO_Read` の永久ループを直したことで、この経路を通るようになった。測距の途中で使う3つの関数だけを直した。

- `vl53l8cx_get_ranging_data`：読み出しに失敗したら、解析しないで戻る。
  元は残った内容をブロックの見出しとして読み、その大きさ（最大 12×4,095 バイト）で `memcpy` するので、結果の構造体の外を書きつぶすおそれがあった。
  `streamcount` も更新しない。
- `vl53l8cx_start_ranging`：最初の `vl53l8cx_get_resolution` に失敗したら、そこで戻る。
  元は残った内容から解像度（最大 255）を求めて読み出しの大きさ（`data_read_size`）を決めるので、
  あとの `get_ranging_data` で一時バッファ（解像度 64 用）の外まで読み込むおそれがあった。
  `data_read_size`・`streamcount` は前の値のまま。
- `vl53l8cx_check_data_ready`：読み出しに失敗したら、`*p_isReady` を 0 にして戻る（元は残った内容で判定し、`streamcount` も書き換えていた）。

ほかの関数（設定の読み書き、`init`、`stop_ranging`、プラグイン）にも同じ形はあるが、読む大きさが固定で、構造体の外は書かないので変えていない。

