param(
  [string]$VersionFw = "v3.58",
  [string]$VersionWeb = "v4.21-r14"
)

$ErrorActionPreference = "Stop"
$Root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$ReleaseDir = Join-Path $Root "releases"
New-Item -ItemType Directory -Force -Path $ReleaseDir | Out-Null

function New-Zip($Name, $Paths) {
  $Temp = Join-Path $env:TEMP ([Guid]::NewGuid().ToString())
  New-Item -ItemType Directory -Force -Path $Temp | Out-Null
  foreach ($p in $Paths) {
    $src = Join-Path $Root $p
    if (Test-Path $src) {
      $dst = Join-Path $Temp $p
      New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
      Copy-Item $src $dst -Recurse -Force
    }
  }
  $zip = Join-Path $ReleaseDir $Name
  if (Test-Path $zip) { Remove-Item $zip -Force }
  Compress-Archive -Path (Join-Path $Temp "*") -DestinationPath $zip -Force
  Remove-Item $Temp -Recurse -Force
  Write-Host "Created $zip"
}

New-Zip "Mini4AI_FirmwareWriter_${VersionFw}_Windows_r11.zip" @(
  "RUN_ME_FIRST_Flash_Mini4AI_Windows.bat",
  "Flash_Mini4AI_Windows.bat",
  "Flash_Mini4AI_Windows.cmd",
  "README_Windows.md",
  "firmware\mini4ai_v358",
  "docs\firmware_update.md",
  "docs\flash_arduino_ide.md",
  "docs\recovery.md",
  "docs\troubleshooting.md",
  "tools\firmware_writer\windows"
)

New-Zip "Mini4AI_WebApp_${VersionWeb}.zip" @("web")
New-Zip "Mini4AI_Source_${VersionWeb}_${VersionFw}.zip" @("README.md", "firmware", "web", "docs", "tools", ".github")
