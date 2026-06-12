param(
  [string]$DeviceName = "",
  [string]$LogPath = ""
)

$ErrorActionPreference = "Stop"
$BoardUrl = "https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json"
$Fqbn = "SiliconLabs:silabs:xiao_mg24:protocol_stack=ble_arduino"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Root = (Resolve-Path (Join-Path $ScriptDir "..\..\..")).Path
$SketchDir = Join-Path $Root "firmware\mini4ai_v358"
$ToolsDir = Join-Path $Root ".tools\arduino-cli"
$ArduinoCli = "arduino-cli"
$StartedTranscript = $false

function Write-Section([string]$Text) {
  Write-Host ""
  Write-Host "=== $Text ===" -ForegroundColor Cyan
}

function Normalize-DeviceName([string]$Name) {
  if ($null -eq $Name) { return "" }
  $n = $Name.Trim()
  $chars = New-Object System.Collections.Generic.List[char]
  foreach ($c in $n.ToCharArray()) {
    if ($c -match '[A-Za-z0-9_-]') { [void]$chars.Add($c) }
  }
  $n = -join $chars
  if ($n.Length -gt 20) { $n = $n.Substring(0,20) }
  return $n
}

function Write-FirmwareConfig([string]$Name) {
  $header = Join-Path $SketchDir "firmware_config.h"
  $escaped = $Name.Replace('"','')
  @("#pragma once", "#define MINI4AI_DEFAULT_DEVICE_NAME `"$escaped`"") | Set-Content -Path $header -Encoding ASCII
  Write-Host "Default BLE device name: $escaped"
}

function Ensure-ArduinoCli {
  $cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
  if ($cmd) { return $cmd.Source }

  Write-Section "Downloading Arduino CLI"
  New-Item -ItemType Directory -Force -Path $ToolsDir | Out-Null
  $zip = Join-Path $ToolsDir "arduino-cli.zip"
  $url = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
  try {
    Invoke-WebRequest -Uri $url -OutFile $zip
    Expand-Archive -Path $zip -DestinationPath $ToolsDir -Force
    $exe = Get-ChildItem -Path $ToolsDir -Recurse -Filter "arduino-cli.exe" | Select-Object -First 1
    if (-not $exe) { throw "arduino-cli.exe was not found after extraction." }
    return $exe.FullName
  } catch {
    throw "Failed to download Arduino CLI. Install arduino-cli manually, then run this tool again. Details: $($_.Exception.Message)"
  }
}

function Run-Checked {
  param(
    [Parameter(Mandatory=$true)][string]$Exe,
    [Parameter(Mandatory=$true)][string[]]$CliArgs,
    [Parameter(Mandatory=$true)][string]$Title
  )
  Write-Section $Title
  Write-Host ("$Exe " + ($CliArgs -join " "))
  & $Exe @CliArgs
  $code = $LASTEXITCODE
  if ($code -ne 0) {
    throw "Command failed with exit code ${code}: $Exe $($CliArgs -join ' ')"
  }
}

function Find-Port([string]$Cli) {
  while ($true) {
    Write-Section "Checking connected boards"
    try {
      $json = & $Cli board list --format json | Out-String
      $data = $json | ConvertFrom-Json
      $ports = @()
      if ($data.detected_ports) { $ports = @($data.detected_ports) }
      elseif ($data) { $ports = @($data) }
      $addresses = @()
      foreach ($p in $ports) {
        if ($p.port.address) { $addresses += $p.port.address }
        elseif ($p.address) { $addresses += $p.address }
      }
      $addresses = $addresses | Where-Object { $_ -match '^COM[0-9]+$' } | Sort-Object -Unique
      if ($addresses.Count -eq 1) {
        Write-Host "Selected port: $($addresses[0])"
        return $addresses[0]
      }
      if ($addresses.Count -gt 1) {
        Write-Host "Detected ports:"
        for ($i=0; $i -lt $addresses.Count; $i++) { Write-Host "[$($i+1)] $($addresses[$i])" }
        Write-Host ""
        Write-Host "Enter the number to select a port."
        Write-Host "Press Enter to rescan. Type Q to cancel."
        $sel = Read-Host "Selection"
        if ($sel.Trim().ToUpperInvariant() -eq "Q") { throw "Flashing cancelled by user." }
        if (-not $sel.Trim()) { continue }
        $idx = 0
        if ([int]::TryParse($sel, [ref]$idx)) {
          $idx = $idx - 1
          if ($idx -ge 0 -and $idx -lt $addresses.Count) { return $addresses[$idx] }
        }
        Write-Host "Invalid selection. Please try again." -ForegroundColor Yellow
        continue
      }
    } catch {
      Write-Host "Automatic port detection failed: $($_.Exception.Message)" -ForegroundColor Yellow
    }

    Write-Host "No XIAO MG24 COM port was selected."
    Write-Host "Connect the XIAO MG24 by USB, then press Enter to rescan."
    Write-Host "Or type a COM port manually, for example COM3. Type Q to cancel."
    $manual = (Read-Host "COM port").Trim()
    if ($manual.ToUpperInvariant() -eq "Q") { throw "Flashing cancelled by user." }
    if (-not $manual) { continue }
    if ($manual -match '^COM[0-9]+$') { return $manual.ToUpperInvariant() }
    Write-Host "Invalid COM port: $manual" -ForegroundColor Yellow
  }
}

try {
  if ($LogPath) {
    Start-Transcript -Path $LogPath -Force | Out-Null
    $StartedTranscript = $true
  }

  Write-Host "Mini4AI Firmware Writer for Windows"
  Write-Host "Before flashing: turn OFF the Mini 4WD side power, then connect USB."

  if (-not (Test-Path (Join-Path $SketchDir "mini4ai_v358.ino"))) {
    throw "Sketch was not found: $SketchDir"
  }

  $DeviceName = Normalize-DeviceName $DeviceName
  if (-not $DeviceName) {
    $DeviceName = Normalize-DeviceName (Read-Host "Default BLE device name. Press Enter for Mini4AI")
  }
  if (-not $DeviceName) { $DeviceName = "Mini4AI" }
  Write-FirmwareConfig $DeviceName

  $ArduinoCli = Ensure-ArduinoCli
  Write-Host "Arduino CLI: $ArduinoCli"

  Write-Section "Initializing Arduino CLI config"
  $configOk = $false
  try {
    & $ArduinoCli config dump *> $null
    if ($LASTEXITCODE -eq 0) { $configOk = $true }
  } catch {
    $configOk = $false
  }
  if ($configOk) {
    Write-Host "Arduino CLI config already exists. Skipping config init."
  } else {
    & $ArduinoCli config init
    $code = $LASTEXITCODE
    if ($code -ne 0) {
      throw "Arduino CLI config init failed with exit code ${code}."
    }
  }

  Run-Checked -Exe $ArduinoCli -CliArgs @("core", "update-index", "--additional-urls", $BoardUrl) -Title "Updating core index"
  Run-Checked -Exe $ArduinoCli -CliArgs @("core", "install", "SiliconLabs:silabs", "--additional-urls", $BoardUrl) -Title "Installing Silicon Labs core"
  Run-Checked -Exe $ArduinoCli -CliArgs @("lib", "update-index") -Title "Updating library index"
  Run-Checked -Exe $ArduinoCli -CliArgs @("lib", "install", "ArduinoBLE") -Title "Installing ArduinoBLE"
  Run-Checked -Exe $ArduinoCli -CliArgs @("lib", "install", "Seeed Arduino LSM6DS3") -Title "Installing Seeed Arduino LSM6DS3"
  Run-Checked -Exe $ArduinoCli -CliArgs @("compile", "--fqbn", $Fqbn, "--warnings", "default", "--export-binaries", $SketchDir) -Title "Building firmware"

  $Port = Find-Port $ArduinoCli
  Run-Checked -Exe $ArduinoCli -CliArgs @("upload", "--fqbn", $Fqbn, "--port", $Port, "--verify", $SketchDir) -Title "Flashing firmware"

  Write-Section "Done"
  Write-Host "Open the Web App and check firmware version v3.58-r10."
  exit 0
} catch {
  Write-Host ""
  Write-Host "[ERROR] $($_.Exception.Message)" -ForegroundColor Red
  exit 1
} finally {
  if ($StartedTranscript) {
    try { Stop-Transcript | Out-Null } catch {}
  }
}
