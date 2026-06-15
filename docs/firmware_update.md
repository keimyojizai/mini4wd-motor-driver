# ファームウェア更新手順

推奨ファームウェア: **v3.58-r10**

## 先に確認すること

- USBケーブルがデータ通信対応であること。
- XIAO MG24 SenseがPCに認識されていること。
- 書き込み中は、ミニ四駆側の電源をオフにしてからUSB接続すること。
- Webアプリで接続中の場合は、切断してから書き込むこと。

## 方法A: Windows用 書き込みツール

1. Releaseから `Mini4AI_FirmwareWriter_v3.58-r10_Windows_r11.zip` をダウンロードします。
2. ZIPを右クリックして「すべて展開」します。
3. XIAO MG24 SenseをUSBでPCに接続します。
4. 展開したフォルダ直下の `START_HERE_Flash_Mini4AI.cmd` をダブルクリックします。
5. 初期BLEデバイス名を聞かれたら入力します。空欄なら `Mini4AI` になります。
6. 画面の指示に従います。

PowerShellの実行ポリシーで止まる場合は、ZIPを展開したフォルダで右クリックし、「ターミナルで開く」から以下を実行します。

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\firmware_writer\windows\flash_windows.ps1
```

## macOS / Linuxについて

macOS / Linux用の書き込みツールは、現時点では動作確認対象外のため同梱していません。

## デバイス名について

書き込みツールでは、初回書き込み時のBLEデバイス名を指定できます。

- 空欄: `Mini4AI`
- 使用可能文字: 半角英数字、ハイフン、アンダースコア
- 最大20文字

Webアプリ接続後も、「ファームウェア更新」カードからデバイス名を変更できます。変更後は、ミニ四駆側の電源を入れ直すと接続画面の表示名に反映されます。

## Webアプリでの本体保存

ファームウェアを書き込んだ後、WebアプリからBLE接続して設定を行います。

スタンドアローン起動で使う設定は、Webアプリの **「本体へ保存」** を押したときだけXIAO MG24 Sense本体へ保存されます。

「走行開始」は現在の画面設定を一時適用して走行する操作です。電源を切った後も同じ設定でスタンドアローン起動したい場合は、必ず「本体へ保存」を押してください。

「本体へ保存」は、Webアプリ左側のメニューにあります。走行開始・停止ボタンとは離して配置し、保存前に確認画面を表示します。

詳細は [standalone_mode.md](standalone_mode.md) を参照してください。

## 方法B: Arduino IDEで手動書き込み

Windows書き込みツールでうまくいかない場合、または既にArduino IDE環境がある場合は、[flash_arduino_ide.md](flash_arduino_ide.md) を参照してください。

## 方法C: 復旧

書き込みに失敗した、デバイスが見えない、アップロードできない場合は [recovery.md](recovery.md) を参照してください。

## 書き込みツールが内部で行うこと

書き込みツールはArduino IDEを起動しません。内部ではArduino CLIを使って以下を実行します。

1. Arduino CLIの確認または取得
2. Silicon Labs Arduino Coreの取得
3. `ArduinoBLE` と `Seeed Arduino LSM6DS3` の取得
4. `firmware/mini4ai_v358/mini4ai_v358.ino` のビルド
5. 必要に応じて `firmware_config.h` に初期BLEデバイス名を生成
6. XIAO MG24 Senseへのアップロード

使用するFQBN:

```text
SiliconLabs:silabs:xiao_mg24:protocol_stack=ble_arduino
```

## Windowsでbatを押しても何も起きない場合

ZIP内から直接実行している可能性があります。Windows用書き込みツールは、必ずZIPを右クリックして「すべて展開」してから、展開先の `START_HERE_Flash_Mini4AI.cmd` を実行してください。

実行後は `Mini4AI_flash_log.txt` が作成されます。失敗時はこのログを確認してください。
