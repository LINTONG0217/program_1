param(
    [string]$Port = "",
    [string]$Source = "openart\openart_ball_pushout.py"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourcePath = Join-Path $projectRoot $Source
if (-not (Test-Path $sourcePath)) {
    throw "OpenART source not found: $sourcePath"
}

$pythonExe = "python"
if (Test-Path (Join-Path $projectRoot ".venv\Scripts\python.exe")) {
    $pythonExe = Join-Path $projectRoot ".venv\Scripts\python.exe"
}

function Invoke-Mpremote {
    param([string[]]$Args)
    $out = & $pythonExe -m mpremote @Args 2>&1
    $text = $out -join [Environment]::NewLine
    if ($LASTEXITCODE -ne 0) {
        Write-Host $text
        throw "mpremote failed"
    }
    return $text
}

if (-not $Port) {
    Write-Host "No -Port specified, trying mpremote auto connect..." -ForegroundColor Yellow
    $connectArgs = @("connect", "auto")
} else {
    $connectArgs = @("connect", $Port)
}

Write-Host "Stopping OpenART runtime..." -ForegroundColor Cyan
try {
    Invoke-Mpremote -Args ($connectArgs + @("soft-reset")) | Out-Null
    Start-Sleep -Milliseconds 250
} catch {
    Write-Host "soft-reset failed, continuing with file copy..." -ForegroundColor Yellow
}

Write-Host "Uploading $Source -> :main.py" -ForegroundColor Cyan
Invoke-Mpremote -Args ($connectArgs + @("fs", "cp", $sourcePath, ":main.py")) | Out-Null

Write-Host "Verifying main.py exists..." -ForegroundColor Cyan
$check = Invoke-Mpremote -Args ($connectArgs + @("exec", "import os; print('OPENART_FILES', os.listdir())"))
Write-Host $check

Write-Host "Soft reset to start OpenART vision..." -ForegroundColor Cyan
Invoke-Mpremote -Args ($connectArgs + @("soft-reset")) | Out-Null
Write-Host "OpenART push-out vision deployed." -ForegroundColor Green
