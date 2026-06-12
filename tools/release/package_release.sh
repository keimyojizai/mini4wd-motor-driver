#!/usr/bin/env bash
set -euo pipefail
VERSION_FW="v3.58"
VERSION_WEB="v4.21-r14"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RELEASE_DIR="$ROOT/releases"
mkdir -p "$RELEASE_DIR"

make_zip() {
  local name="$1"; shift
  local tmp
  tmp="$(mktemp -d)"
  for p in "$@"; do
    if [ -e "$ROOT/$p" ]; then
      mkdir -p "$tmp/$(dirname "$p")"
      cp -R "$ROOT/$p" "$tmp/$p"
    fi
  done
  (cd "$tmp" && zip -qr "$RELEASE_DIR/$name" .)
  rm -rf "$tmp"
  echo "Created $RELEASE_DIR/$name"
}

make_zip "Mini4AI_FirmwareWriter_${VERSION_FW}_Windows_r11.zip" \
  "RUN_ME_FIRST_Flash_Mini4AI_Windows.bat" \
  "Flash_Mini4AI_Windows.bat" \
  "Flash_Mini4AI_Windows.cmd" \
  "README_Windows.md" \
  "firmware/mini4ai_v358" \
  "docs/firmware_update.md" \
  "docs/flash_arduino_ide.md" \
  "docs/recovery.md" \
  "docs/troubleshooting.md" \
  "tools/firmware_writer/windows"

make_zip "Mini4AI_WebApp_${VERSION_WEB}.zip" "web"
make_zip "Mini4AI_Source_${VERSION_WEB}_${VERSION_FW}.zip" "README.md" "firmware" "web" "docs" "tools" ".github"
