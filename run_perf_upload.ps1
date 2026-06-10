param()
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'
& $pio run -e esp32p4-perf -t upload
exit $LASTEXITCODE
