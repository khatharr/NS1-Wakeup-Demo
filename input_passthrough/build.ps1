$ErrorActionPreference = 'Stop'
$sourceDir = Join-Path $PSScriptRoot 'src'
$outputDir = Join-Path $PSScriptRoot 'bin'
$sources = @(
    (Join-Path $sourceDir 'ns1_input_passthrough.c'),
    (Join-Path $sourceDir 'serial_bridge.c'),
    (Join-Path $sourceDir 'gamepad_input.c')
)
$output = Join-Path $outputDir 'NS Input Passthrough.exe'

[System.IO.Directory]::CreateDirectory($outputDir) | Out-Null
& gcc -std=c11 -O2 -Wall -Wextra -mwindows -s -o $output @sources
if ($LASTEXITCODE -ne 0) {
    throw "Input passthrough build failed: exit=$LASTEXITCODE"
}
