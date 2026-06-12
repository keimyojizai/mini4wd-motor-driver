# トラブルシュート

## Webアプリで接続できない

- ChromeまたはEdgeを使ってください。
- WebアプリをHTTPSまたはlocalhostで開いてください。
- OSのBluetoothをONにしてください。
- すでに別のタブや別PCから接続していないか確認してください。
- WebアプリはMini4AIのBLEサービスUUIDで検索します。デバイス名を変更していても接続できます。古いファームウェアの場合は `MiniYonAI_03` または `Mini4AI` で表示されることがあります。


## デバイス名を変えたら見つからない

- ミニ四駆側の電源を入れ直してください。BLEの表示名は再起動後に反映されます。
- WebアプリはBLEサービスUUIDでも検索するため、通常は変更後の名前でも接続できます。
- それでも見えない場合は、OSのBluetooth設定画面で古いペアリング情報を削除し、Chrome/Edgeを再起動してください。

## ファームウェアバージョンが「不明」になる

古いファームウェアにはバージョン取得用BLE characteristicがありません。v3.58-r10以降を書き込んでください。

## 書き込みツールが止まる

- 初回はArduino CLI、Silicon Labs core、ライブラリを取得します。インターネット接続が必要です。
- 会社・学校ネットワークではGitHubやArduino downloadsが遮断される場合があります。
- その場合は別ネットワークで実行するか、Arduino IDEで手動書き込みしてください。

## `LSM6DS3.h` が見つからない

`Seeed Arduino LSM6DS3` が入っていません。Arduino CLIなら以下を実行します。

```bash
arduino-cli lib install "Seeed Arduino LSM6DS3"
```

## `ArduinoBLE.h` が見つからない

以下を実行します。

```bash
arduino-cli lib install ArduinoBLE
```

## BLEで見えない

- 書き込み後、一度USBを抜き挿ししてください。
- XIAO MG24 Senseの電源が入っているか確認してください。
- ファームウェアのBLE初期化に失敗している可能性がある場合は、再書き込みしてください。

## Windowsでbatを開くと一瞬だけ表示されて閉じる

原因の多くは、ZIP内から直接実行している、または古いbatファイルの改行/文字コードがWindows CMDと合っていないことです。

対処:

1. ZIPを右クリックして「すべて展開」します。
2. 展開したフォルダ直下の `RUN_ME_FIRST_Flash_Mini4AI_Windows.bat` を実行します。
3. 失敗した場合は `Mini4AI_flash_log.txt` を確認します。

書き込み時は、ミニ四駆側の電源をオフにしてからUSB接続してください。


## スタンドアローン起動で保存した設定が反映されない

Webアプリの「走行開始」は一時適用です。スタンドアローン起動で使う設定は、「本体へ保存」を押したときだけ本体に保存されます。

確認すること:

- ルール設定後に「本体へ保存」を押したか
- 「本体へ保存しました」と表示されたか
- 保存後にミニ四駆側の電源を入れ直したか
- ルールが0本の状態で保存していないか

詳しくは [standalone_mode.md](standalone_mode.md) を参照してください。
