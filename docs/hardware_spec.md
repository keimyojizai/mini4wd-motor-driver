# ハードウェア仕様と主要部品

[← READMEへ戻る](../README.md)

---

## 基本仕様

| 項目 | 内容 |
|---|---|
| 製品名 | ミニ四駆AI モータードライバ基板 v0.3 |
| 対応マイコン | XIAO MG24 Sense専用 |
| 駆動方式 | 正転専用、M-側制御 |
| モーター制御 | Nch MOSFET低側駆動 |
| ブレーキ | Pch MOSFETによるショートブレーキ |
| 制御信号 | PWM入力 1本 |
| ゲート駆動 | UCC27511による5Vゲート駆動 |
| 想定電源 | Ni-MH 2セル / Li-ion 1セル |
| 非対応電源 | Li-ion 2セル、8.4V系電源 |
| 電池 / M+電圧観測 | VBAT_SENSE |
| M-側電圧観測 | MID_SENSE |
| 基板温度観測 | NTC TEMP |
| 基板サイズ | 約 55 mm × 21 mm |
| 基板厚 | 1.2 mm |
| 推奨大電流配線 | 18AWG、短距離なら20AWGも可 |

---

## 設計思想

本基板は、汎用モータードライバではありません。

**ミニ四駆AI制御に必要な機能へ絞り込んだ、専用設計の高電流ドライバ**です。

重点を置いたのは次の4点です。

1. **1セル系電源で強く駆動すること**
2. **短時間で効くショートブレーキを扱えること**
3. **XIAO MG24 Senseと組み合わせて実装しやすいこと**
4. **電圧・温度をソフトウェアから観測できること**

---

## 駆動性能について

本基板は、一般的な小型モータードライバICではなく、  
**低オン抵抗MOSFETと専用ゲートドライバを用いた、ミニ四駆用ブラシモーター向け高電流ドライバ**として設計しています。

手元評価では、  
**DRV8835搭載の小型モータードライバでは電流制限により起動・駆動できなかったミニ四駆用モーターについても、本基板では駆動できることを確認しています。**

ただし、実際の駆動可否や発熱は、

- 使用するモーター
- 電池
- 配線抵抗
- 配線長
- PWM Duty
- ショートブレーキ時間
- 車体負荷

によって変化します。

**最大連続電流・最大ピーク電流を一律に保証するものではありません。**  
必要に応じて、実機評価に基づく電流・温度特性を今後追記します。

---

## 主要部品

### Nch MOSFET: SiSS64DN-T1-GE3

- 用途: **モーター駆動用Nch MOSFET**
- メーカー: Vishay
- パッケージ: PowerPAK 1212-8S
- VDS: 30 V
- RDS(on): 最大 2.86 mΩ @ VGS = 4.5 V
- Qg: 21 nC typ.

低オン抵抗で、1セル系電源でも損失を抑えながらモーター電流を扱う目的で採用しています。

- [Vishay SiSS64DN データシート](https://www.vishay.com/docs/67294/siss64dn.pdf)

---

### Pch MOSFET: SI7137DP-T1-GE3

- 用途: **ショートブレーキ用Pch MOSFET**
- メーカー: Vishay
- パッケージ: PowerPAK SO-8
- VDS: -20 V
- 低オン抵抗のPch MOSFETを採用

ブレーキ時にM-側をM+側へ短絡するための高側Pch MOSFETです。

- [Vishay SI7137DP データシート](https://www.vishay.com/doc/?69063=)

---

### ゲートドライバ: UCC27511DBVR

- 用途: **MOSFETゲート駆動**
- メーカー: Texas Instruments
- 動作電圧範囲: 4.5 V ～ 18 V
- 4 A peak source / 8 A peak sink
- split output構成により、立ち上がり・立ち下がりのゲート抵抗を個別に設定可能

本基板では、XIAO MG24 SenseのPWM信号を直接MOSFETゲートへ入れるのではなく、UCC27511で受けてゲートを駆動します。

これにより、

- MOSFETのゲート充放電を強く行う
- 5Vゲート駆動を実現する
- スイッチング遷移を安定させる

ことを狙っています。

- [TI UCC27511 データシート](https://www.ti.com/lit/gpn/UCC27511)
- [TI UCC27511 製品ページ](https://www.ti.com/product/UCC27511)

---

### 5V昇圧DCDC: TPS61023DRLR

- 用途: **5V系電源生成**
- メーカー: Texas Instruments
- 入力電圧範囲: 0.5 V ～ 5.5 V
- 起動時最低入力電圧: 1.8 V
- 出力設定範囲: 2.2 V ～ 5.5 V
- 3.7 A valley switching current limit typ.

本基板では、Ni-MH 2セルまたはLi-ion 1セルから5Vを生成し、

- UCC27511のゲートドライブ電源
- XIAO MG24 Senseへの5V供給

に使用します。

- [TI TPS61023 データシート](https://www.ti.com/lit/ds/symlink/tps61023.pdf)
- [TI TPS61023 製品ページ](https://www.ti.com/product/TPS61023)

---

### 温度センサ: NTCG104BH103JT1

- 用途: **基板温度観測**
- メーカー: TDK
- 抵抗値: 10 kΩ @ 25°C
- 抵抗許容差: ±5%
- B定数: 4100 K typ. @ 25/85°C
- B定数許容差: ±3%

本基板では、NTCサーミスタと10kΩ固定抵抗で分圧を作り、XIAO MG24 SenseのADCで基板周辺の温度変化を観測します。

- [TDK NTCG104BH103JT1 製品ページ](https://product.tdk.com/en/search/sensor/ntc/chip-ntc-thermistor/info?part_no=NTCG104BH103JT1)

---

## 5V給電と逆流対策

XIAO MG24 Senseの5Vピンは、外部から電源入力として使用できますが、Seeed公式ドキュメントでは、USB給電との競合を避けるために**逆流防止用ダイオード**を入れるよう説明されています。

本基板では、XIAO MG24 Senseへの5V供給ラインに逆流対策を実装しています。

- [Seeed Studio XIAO MG24(Sense) 入門ガイド](https://wiki.seeedstudio.com/ja/xiao_mg24_getting_started/)

---

## センス信号

### VBAT_SENSE

**電池＋およびモーターM+側の共通ノード電圧**を観測するための信号です。

用途例:

- 電池残量の目安
- 負荷時の電圧降下観測
- 走行中の電源状態記録

---

### MID_SENSE

**モーターM-側の電圧**を観測するための信号です。

M-はMOSFETでスイッチングされるノードであり、

- 駆動時
- ショートブレーキ時
- PWM中間Duty時

で波形が変化します。

用途例:

- M-側電圧の観測
- 駆動状態のデバッグ
- 独自の制御・解析実験

---

### NTC TEMP

基板上の温度変化を観測するための信号です。

本基板では、

```text
10kΩ固定抵抗 + 10kΩ NTCサーミスタ
```

による分圧回路を構成し、ADCで読み取ります。

温度換算は、採用しているNTCのB定数を用いてソフトウェア側で行います。

---

## VBAT_SENSE / MID_SENSE の換算式

VBAT_SENSEとMID_SENSEは、いずれも

```text
22kΩ : 10kΩ
```

の抵抗分圧を使っています。

分圧後電圧を `V_ADC`、実際のノード電圧を `V_REAL` とすると、

```text
V_REAL = V_ADC × (22k + 10k) / 10k
       = V_ADC × 3.2
```

です。

基板裏面にも、

```text
Actual voltage = ADC input voltage × 3.2
```

とシルク印刷しています。

---

## NTC温度換算

### 前提値

採用NTC:

- 型番: `NTCG104BH103JT1`
- `R0 = 10,000 Ω` @ 25°C
- `T0 = 25°C = 298.15 K`
- `B = 4100 K`（25/85°C typ.）

### 抵抗値から温度を求める式

```text
T[K] = 1 / { 1/T0 + (1/B) × ln(R_NTC / R0) }
T[°C] = T[K] - 273.15
```

### 参考実装例

```cpp
#include <math.h>

constexpr float R0 = 10000.0f;      // 10kΩ @ 25°C
constexpr float T0 = 298.15f;       // 25°C in Kelvin
constexpr float BETA = 4100.0f;     // NTCG104BH103JT1, B25/85 typ.

float ntcResistanceToTemperatureC(float rNtc) {
  const float tempK = 1.0f /
    (1.0f / T0 + (1.0f / BETA) * logf(rNtc / R0));
  return tempK - 273.15f;
}
```
