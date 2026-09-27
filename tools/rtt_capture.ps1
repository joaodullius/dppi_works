# Capture SEGGER RTT channel 0 from the nRF5340 app core for N seconds.
# - The reset (if requested) is done BEFORE the logger attaches: nrfutil cannot
#   open the J-Link while JLinkRTTLogger holds it. Nothing is lost: with
#   LOG_BACKEND_RTT_MODE_DROP the boot lines wait in the RTT buffer.
# - JLinkRTTLogger exits on any keypress / stdin EOF, so stdin is kept open and
#   a newline is sent at the end for a clean shutdown (flushes the file).
# usage: rtt_capture.ps1 -Out file.log -Seconds 30 [-Reset] [-Head 60] [-Tail 20]
param(
    [string]$Out = "rtt.log",
    [int]$Seconds = 30,
    [switch]$Reset,
    [int]$Head = 60,
    [int]$Tail = 20,
    [string]$Serial = "1057766689",
    [string]$Device = "NRF5340_XXAA_APP",
    # Address of _SEGGER_RTT in the ELF (nm); avoids attaching to a stale
    # control block left in RAM by a previous firmware.
    [string]$RttAddress = ""
)
$logger = 'C:\Program Files\SEGGER\JLink_V924a\JLinkRTTLogger.exe'
if (Test-Path $Out) { Remove-Item $Out -Force }

if ($Reset) {
    $r = nrfutil device reset --serial-number $Serial --reset-kind RESET_PIN 2>&1 | Out-String
    "reset via J-Link ($Serial): exit=$LASTEXITCODE $($r.Trim())"
}

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $logger
$addrArg = if ($RttAddress) { "-RTTAddress $RttAddress " } else { "" }
$psi.Arguments = "-Device $Device -If SWD -Speed 4000 -USB $Serial $addrArg-RTTChannel 0 `"$Out`""
$psi.UseShellExecute = $false
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$p = [System.Diagnostics.Process]::Start($psi)
$p.BeginOutputReadLine()
$p.BeginErrorReadLine()

Start-Sleep ($Seconds + 3)

try { $p.StandardInput.WriteLine(); $p.StandardInput.Close() } catch {}
if (-not $p.WaitForExit(5000)) { Stop-Process -Id $p.Id -Force }
Start-Sleep 1

if (Test-Path $Out) {
    $lines = Get-Content $Out
    "== $Out : $($lines.Count) linhas"
    $lines | Select-Object -First $Head
    if ($lines.Count -gt ($Head + $Tail)) { "..."; $lines | Select-Object -Last $Tail }
} else {
    "== sem arquivo de log"
}
