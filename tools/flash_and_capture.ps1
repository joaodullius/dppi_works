# Flash a sysbuild build dir through the DK's J-Link, then reset + capture RTT.
# usage: flash_and_capture.ps1 -Build <dir> -Out <log> [-Seconds 30] [-Domain <image>] [-Head 40] [-Tail 12]
param(
    [Parameter(Mandatory)][string]$Build,
    [Parameter(Mandatory)][string]$Out,
    [int]$Seconds = 30,
    [string]$Domain = "",
    [int]$Head = 40,
    [int]$Tail = 12,
    [string]$Serial = "1057766689",
    [string]$Device = "NRF5340_XXAA_APP",
    [string]$Elf = ""   # zephyr.elf of the app: its _SEGGER_RTT address is passed to the logger
)
$sp = Split-Path -Parent $MyInvocation.MyCommand.Path
$args = @('sdk-manager', 'toolchain', 'launch', '--ncs-version', 'v3.4.1', '--chdir', 'C:\ncs\v3.4.1', '--',
          'west', 'flash', '-d', $Build, '--dev-id', $Serial)
if ($Domain) { $args += @('--domain', $Domain) }
"== flash $Build $Domain"
& nrfutil @args 2>&1 | Select-String -Pattern "Flashing file|Verifying|Error|error|FAILED|Reset$" |
    ForEach-Object { $_.Line.Trim() -replace '^-- runners.nrfutil: ', '' }
if ($LASTEXITCODE -ne 0) { "FLASH FAILED (exit $LASTEXITCODE)"; exit 1 }
$addr = ""
if ($Elf) {
    # FLPR builds: RISC-V ELF, but the RTT block (shared SRAM) is read through the M33 connection
    $nm = if ($Device -like "*RV32*" -or $Elf -like "*flpr*") { 'C:\ncs\toolchains\4f5b6ad6dd\opt\zephyr-sdk\gnu\riscv64-zephyr-elf\bin\riscv64-zephyr-elf-nm.exe' } else { 'C:\ncs\toolchains\4f5b6ad6dd\opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-nm.exe' }
    $line = & $nm $Elf | Select-String -Pattern " _SEGGER_RTT$" | Select-Object -First 1
    if ($line) { $addr = "0x" + ($line.Line -split ' ')[0]; "RTT control block at $addr" }
}
"== rtt $Seconds s"
& "$sp\rtt_capture.ps1" -Out $Out -Seconds $Seconds -Reset -Head $Head -Tail $Tail -Serial $Serial -Device $Device -RttAddress $addr
