# これは何？
腹筋ローラーのRep数を計測するファームウェアです。

[動作動画](https://twitter.com/UGKGbrothers/status/1505657964736421888)

[参考：腹筋ローラーのトレーニング映像](https://www.youtube.com/watch?v=neKcj1PcnXM)

# 計測のしくみ
- ToFセンサで腹筋ローラーの距離を計測し続けます。
- 一定時間近づいて、一定時間離れたら１回と計測します。
### 状態遷移図
![statediagram](https://user-images.githubusercontent.com/44434953/167241956-fb07a9eb-3d77-4987-a938-d73dba21491b.png)

# 画面の見方
1回トレーニングするごとに強度によって、上部のS(trong), M(edium), W(eak)のいずれかがカウントアップします。
下部の円は現在の状態を表現しています。
| 色 | 状態 |画像|
|:---|:------------|:-----|
| 黄 | 測定範囲外 |![init](https://user-images.githubusercontent.com/44434953/167242081-2e52b20f-e953-4393-b5cc-2ac771e5407c.jpg)|
| 白 | 測定開始可能 |![ready](https://user-images.githubusercontent.com/44434953/167242082-c7afcbcc-36ec-4a52-874e-2e7c118d9d2d.jpg)|
| 青 | 前進 |![push](https://user-images.githubusercontent.com/44434953/167242083-646b8351-87be-446c-9ed7-230ba1350eac.jpg)|
| 赤 | 後退 |![pull](https://user-images.githubusercontent.com/44434953/167242090-9e49e69b-f719-45fc-aa62-c8fb41cdae86.jpg)|

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
