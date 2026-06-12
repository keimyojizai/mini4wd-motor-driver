# Releaseチェックリスト

Release作成前に以下を確認します。

## ファームウェア

- [ ] `#define FW_VERSION "v3.58-r10"` が正しい
- [ ] `firmwareInfoChar` がWebアプリから読める
- [ ] XIAO MG24 Senseでビルドできる
- [ ] BLE接続できる
- [ ] START/STOPできる
- [ ] 本体へ保存できる
- [ ] スタンドアローン起動が意図通り
- [ ] standalone_mode.md で本体へ保存とスタンドアローン起動を説明済み

## Webアプリ

- [ ] `web/index.html` の表示バージョンが正しい
- [ ] 推奨ファームウェア `v3.58-r10` が正しい
- [ ] ファームウェア更新カードのリンクが正しい
- [ ] Chrome/EdgeでBLE接続できる
- [ ] 停止後ログが取得できる

## Release assets

- [ ] `Mini4AI_FirmwareWriter_v3.58-r10_Windows_r11.zip`
- [ ] `Mini4AI_WebApp_v4.21-r16.zip`
- [ ] `source.zip`

## 公開文

Release本文には以下を入れます。

```text
推奨Webアプリ: v4.21-r16
推奨ファームウェア: v3.58-r10

更新内容:
- ファームウェア更新カードを追加
- 接続中ファームウェアバージョン表示に対応
- Windows用のArduino CLI書き込みスクリプトを追加

注意:
- 書き込み時は、ミニ四駆側の電源をオフにしてからUSB接続してください。
- Webアプリから直接ファームウェアを書き込む機能はありません。
```
