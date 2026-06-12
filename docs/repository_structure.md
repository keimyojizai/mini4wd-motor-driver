# リポジトリ構成

```text
mini4wd-motor-driver/
├─ README.md
├─ firmware/
│  └─ mini4ai_v358/
│     └─ mini4ai_v358.ino
├─ web/
│  └─ index.html
├─ docs/
│  ├─ firmware_update.md
│  ├─ flash_arduino_ide.md
│  ├─ recovery.md
│  ├─ troubleshooting.md
│  └─ release_checklist.md
├─ tools/
│  ├─ firmware_writer/
│  │  ├─ README.md
│  │  ├─ windows/
│  │  ├─ macos/
│  │  └─ linux/
│  └─ release/
└─ .github/
   └─ workflows/
```

## 公開時の推奨

- GitHub Pagesは `web/index.html` を公開対象にする。
- ReleaseにはOS別の書き込みZIPを置く。
- READMEの先頭に「通常利用」と「ファームウェア更新」を分けて書く。
- 不具合報告テンプレートでは、Webアプリ版、ファーム版、OS、ブラウザ、XIAO MG24 Senseの有無を聞く。
