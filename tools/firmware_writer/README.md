# Mini4AI Firmware Writer

Arduino IDEを開かずに、XIAO MG24 SenseへMini4AIファームウェアを書き込むためのWindows用スクリプトです。

## 対象

- Windows 10/11
- Seeed Studio XIAO MG24 Sense / XIAO MG24

## 使い方

1. ZIPを右クリックして「すべて展開」します。
2. 展開したフォルダ直下の `RUN_ME_FIRST_Flash_Mini4AI_Windows.bat` を実行します。
3. 初期BLEデバイス名を入力します。空欄なら `Mini4AI` になります。
4. 画面の指示に従って書き込みます。

## 注意

ファームウェア書き込み時は、ミニ四駆側の電源をオフにしてからUSB接続してください。

macOS / Linux用の書き込みツールは、現時点では動作確認対象外のため同梱していません。
