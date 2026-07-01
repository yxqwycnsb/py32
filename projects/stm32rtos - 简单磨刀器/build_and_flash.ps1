<#
One-step build & optional flash script for this project.
Usage:
  pwsh -ExecutionPolicy Bypass -File .\build_and_flash.ps1             # configure + build
  pwsh -ExecutionPolicy Bypass -File .\build_and_flash.ps1 -Flash     # also flash via OpenOCD
#>
param(
    [switch]$Flash,
    [string]$Toolchain = "arm-none-eabi.toolchain.cmake",
    [string]$LinkerScript = "Start/Project.sct",
    [string]$BuildDir = "build",
    [string]$OpenOcdInterface = "interface/stlink.cfg",
    [string]$OpenOcdTarget = "target/stm32f1x.cfg"
)

function Check-Tool($name) {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if (-not $cmd) { Write-Error "$name not found in PATH."; return $false }
    return $true
}

Write-Host "== Verify tools =="
$tools = @('pwsh','cmake','arm-none-eabi-gcc','arm-none-eabi-objcopy','arm-none-eabi-size')
$ok = $true
foreach ($t in $tools) { if (-not (Check-Tool $t)) { $ok = $false } }
if (-not $ok) { Write-Error "Install missing tool(s) and ensure they are in PATH."; exit 2 }

Write-Host "pwsh:"; pwsh --version 2>$null | Out-Null
Write-Host "arm-none-eabi-gcc:"; arm-none-eabi-gcc --version | Select-Object -First 1
Write-Host "cmake:"; cmake --version | Select-Object -First 1

# Configure
Write-Host "`n== CMake configure =="
$cmakeArgs = "-S . -B $BuildDir -DCMAKE_TOOLCHAIN_FILE=$Toolchain -DLINKER_SCRIPT=$LinkerScript"
Write-Host "cmake $cmakeArgs"
$rc = & cmake $cmakeArgs
if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed."; exit $LASTEXITCODE }

# Build
Write-Host "`n== Build =="
$rc = & cmake --build $BuildDir --config Release -- -j
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed."; exit $LASTEXITCODE }

# Locate firmware ELF
$elf = Join-Path -Path $BuildDir -ChildPath "firmware.elf"
if (-not (Test-Path $elf)) {
    # try typical CMake output
    $candidates = Get-ChildItem -Path $BuildDir -Filter "*.elf" -Recurse | Select-Object -First 1
    if ($candidates) { $elf = $candidates.FullName } else { Write-Error "firmware ELF not found."; exit 3 }
}

Write-Host "Built: $elf"

if ($Flash) {
    if (-not (Check-Tool 'openocd')) { Write-Error "OpenOCD not found in PATH."; exit 4 }
    Write-Host "`n== Flashing via OpenOCD =="
    $cmd = "openocd -f $OpenOcdInterface -f $OpenOcdTarget -c \"program $elf verify reset exit\""
    Write-Host $cmd
    & openocd -f $OpenOcdInterface -f $OpenOcdTarget -c "program $elf verify reset exit"
    if ($LASTEXITCODE -ne 0) { Write-Error "OpenOCD flashing failed."; exit $LASTEXITCODE }
    Write-Host "Flashing completed.";
}

Write-Host "All done."; exit 0
