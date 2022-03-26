# これは何？
腹筋ローラーのRep数を計測するファームウェアです。

[動作動画](https://twitter.com/UGKGbrothers/status/1505657964736421888)

[参考：腹筋ローラーのトレーニング映像](https://www.youtube.com/watch?v=neKcj1PcnXM)

# 計測のしくみ
- ToFセンサで腹筋ローラーの距離を計測し続けます。
- 一定時間近づいて、一定時間離れたら１回と計測します。
### TODO：状態遷移図

# 画面の見方
1回トレーニングするごとに強度によって、上部のS(trong), M(edium), W(eak)のいずれかがカウントアップします。
下部の円は現在の状態を表現しています。
| 色 | 状態 |
|:---|:------------|
| 黄 | 測定範囲外 |
| 白 | 測定開始可能 |
| 青 | 前進 |
| 赤 | 後退 |

白になるように距離を調整して、そこから腹筋ローラーのトレーニングを開始します。
そして、青→赤→白と変わっていけば１回トレーニングしたことになりSMWのいずれかがカウントアップします。
また、黄 or 白状態の場合、円に加えて現在のToFセンサ値を表示します。
1000-1300mmが白になる距離です。

# 必要なもの
| 名前 | 品名 |
|-----------|------------|
| マイコンボード | NUCLEO-F411RE |
| ToFセンサ     | VL53L0X        |
| LCD    | ST7735        | 
| USBケーブル    |  USBミニBタイプ(PCのペリフェラルとして接続)    |
| 電源    | USBバスパワー       |

# 配線
[twitter link](https://twitter.com/UGKGbrothers/status/1480446401490432000)

# IOマップ
[twitter link](https://twitter.com/UGKGbrothers/status/1482458100133425152)

# 始め方
## requirements
| Item | Version |
| ------------- | ------------- |
| Ubuntu  | 20.04  |
| Rust  | 1.52.1  |

## 手順
1. Clone this repository
2. Add crate
```
sudo apt-get install libssl-dev
cargo install cargo-edit
```
3. Add target
```
cd abdominal_muscle_roller_meter
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
rustup component add llvm-tools-preview
```
4. Build
```
cargo build --release
```
5. Flash binary file
```
sudo apt install openocd
openocd -f nucleo.cfg -c"program target/thumbv7em-none-eabihf/debug/abdominal_muscle_roller_meter verify reset exit"
```

# 問題点
シングルスレッドで実装しているため、REP計測がLCD描画時間に依存しています。REP計測を割り込み処理にさせたいのですが、ペリフェラル共有の実装方法が理解不足でできていません。
