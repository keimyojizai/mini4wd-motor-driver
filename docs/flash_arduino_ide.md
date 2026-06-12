# Arduino IDEで手動書き込みする手順

書き込みツールがうまく動かない場合の手動手順です。

## 1. Arduino IDEを入れる

Arduino IDE 2.xをインストールします。

## 2. Silicon Labs board packageを追加する

Arduino IDEを開き、Preferences / 設定 の **Additional Boards Manager URLs** に以下を追加します。

```text
https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json
```

その後、Boards Managerで `Silicon Labs` を検索してインストールします。

## 3. ライブラリを入れる

Library Managerで以下をインストールします。

- `ArduinoBLE`
- `Seeed Arduino LSM6DS3`

注意: `Arduino_LSM6DS3 by Arduino` ではなく、Seeed Studioの `Seeed Arduino LSM6DS3` を使ってください。

## 4. ボード設定

Toolsで以下を選択します。

```text
Board: Seeed Studio XIAO MG24 (Sense)
Protocol stack: BLE (Arduino)
Programmer: OpenOCD
Port: XIAO MG24 Senseが表示されているポート
```

## 5. 書き込み

`firmware/mini4ai_v358/mini4ai_v358.ino` を開き、Uploadします。

書き込み後、WebアプリからBLE接続し、ファームウェアバージョンが `v3.58-r10` と表示されることを確認してください。
