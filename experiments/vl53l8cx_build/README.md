# VL53L8CX のビルド確認（確認用。Nova 本体のビルドには含まれない）

## 目的

ToF センサー VL53L8CX を買う前に、STM32duino のライブラリが Nova の環境
（PlatformIO・Arduino・esp-wrover-kit・espressif32@7.1.3）でビルドできるかを確かめる。
実機には書き込んでいない（センサーは未購入）。経緯と読み取った内容は
[docs/specs/common_tof_checklist.md](../../docs/specs/common_tof_checklist.md)。

## 3つの環境

| 環境 | 内容 | ソース |
|---|---|---|
| `l8cx_i2c` | STM32duino VL53L8CX 2.1.0、I2C（Wire1、GPIO26・27、400kHz） | `src/l8cx_i2c/` |
| `l8cx_spi` | 同上、SPI（ピンは仮。ビルドが通るかだけ） | `src/l8cx_spi/` |
| `l5cx_sparkfun` | 比較用：SparkFun VL53L5CX v1.0.3、I2C（Wire1） | `src/l5cx_sparkfun/` |

スケッチは公式の HelloWorld 相当で、ESP32 向けに書き換えてある（変えた所は各 `main.cpp` の先頭）。

## ビルド結果（2026-10-12）

| 環境 | 結果 | Flash | RAM |
|---|---|---|---|
| `l8cx_i2c` | 成功 | 385,301 バイト（29.4%） | 24,616 バイト（7.5%） |
| `l8cx_spi` | 成功 | 381,641 バイト（29.1%） | 24,696 バイト（7.5%） |
| `l5cx_sparkfun` | 成功 | 380,893 バイト（29.1%） | 23,176 バイト（7.1%） |

ライブラリは無修正。警告・エラーなし。

## 使い方

リポジトリのルートで実行する（`pio` は `~/.platformio/penv/Scripts/pio.exe`）。

```
pio run -d experiments/vl53l8cx_build              # 3つとも
pio run -d experiments/vl53l8cx_build -e l8cx_i2c  # 1つだけ
```

- ライブラリは初回に GitHub から `.pio/libdeps` へ取得される（ネットワークが必要）。`.pio/` は .gitignore 済み。
- 書き込み（`-t upload`）をすると Nova 本体のファームが上書きされる。書き込むのは、ユーザーが指示したときだけ。
