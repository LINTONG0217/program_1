param(
    [string]$Port = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$pyCandidates = @(
    (Join-Path $projectRoot ".venv\Scripts\python.exe"),
    (Join-Path $projectRoot ".venv\bin\python.exe"),
    "python"
)

$pythonExe = $null
foreach ($candidate in $pyCandidates) {
    if ($candidate -eq "python") {
        $cmd = Get-Command python -ErrorAction SilentlyContinue
        if ($cmd) { $pythonExe = "python"; break }
    } elseif (Test-Path $candidate) {
        $pythonExe = $candidate
        break
    }
}

if (-not $pythonExe) {
    throw "未找到 Python，请先创建 .venv 或安装 Python。"
}

function Test-MicroPythonPort {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    try {
        $execScript = 'import sys;print(sys.implementation.name)'
        $output = & $pythonExe -m mpremote connect $Name exec $execScript 2>&1
        $outStr = $output -join [Environment]::NewLine
        if ($LASTEXITCODE -eq 0 -and $outStr -match 'micropython') {
            return $true
        }
    }
    catch {
        return $false
    }
    return $false
}

function Resolve-DeployPort {
    param(
        [string]$GivenPort
    )

    if ($GivenPort -and (Test-MicroPythonPort -Name $GivenPort)) {
        return $GivenPort
    }

    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    foreach ($p in $ports) {
        if (Test-MicroPythonPort -Name $p) {
            return $p
        }
    }

    return $null
}

Write-Host "[1/4] 检查 mpremote..." -ForegroundColor Cyan
& $pythonExe -m mpremote --help | Out-Null

function Invoke-Mpremote {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Args,
        [int]$Retries = 3,
        [int]$SleepMs = 250,
        [string]$PortName = ""
    )

    $currentPort = $PortName
    for ($i = 1; $i -le $Retries; $i++) {
        $useArgs = @()
        foreach ($a in $Args) {
            if ($a -eq "__PORT__") {
                $useArgs += $currentPort
            } else {
                $useArgs += $a
            }
        }

        $out = & $pythonExe -m mpremote @useArgs 2>&1
        $outStr = $out -join [Environment]::NewLine
        if ($LASTEXITCODE -eq 0) {
            return $outStr
        }

        $looksLikeDisconnect = (
            $outStr -match 'could not open port' -or
            $outStr -match 'PermissionError' -or
            $outStr -match '拒绝访问' -or
            $outStr -match 'No such file' -or
            $outStr -match 'timed out' -or
            $outStr -match 'Could not enter raw repl' -or
            $outStr -match 'OSError\(' -or
            $outStr -match 'Device disconnected'
        )

        if ($looksLikeDisconnect) {
            try {
                $newPort = Resolve-DeployPort -GivenPort $currentPort
                if ($newPort -and $newPort -ne $currentPort) {
                    Write-Host "检测到端口变化：$currentPort -> $newPort" -ForegroundColor Yellow
                    $currentPort = $newPort
                }
            } catch {
                # ignore
            }
        }

        if ($i -lt $Retries) {
            Write-Host "mpremote 失败，准备重试 ($i/$Retries)..." -ForegroundColor Yellow
            Start-Sleep -Milliseconds $SleepMs
        } else {
            Write-Host "mpremote 最终失败。常见原因：" -ForegroundColor Red
            Write-Host "- 串口被 Thonny/串口助手占用（PermissionError/拒绝访问）" -ForegroundColor Red
            Write-Host "- USB 线/Hub 供电不稳，板子中途掉线/复位（端口会消失或变号）" -ForegroundColor Red
            Write-Host "- 板端程序在跑高频打印/UART，导致通信超时（先 soft-reset）" -ForegroundColor Red
            Write-Host "输出如下：" -ForegroundColor Red
            Write-Host $outStr
            throw "mpremote command failed"
        }
    }
}

$resolvedPort = Resolve-DeployPort -GivenPort $Port
if (-not $resolvedPort) {
    throw "未找到可用的 MicroPython 串口。请关闭 Thonny/串口助手后重试。"
}

Write-Host "使用串口: $resolvedPort" -ForegroundColor Yellow

Write-Host "[1.5/4] 软复位，停止板端正在运行的程序..." -ForegroundColor Cyan
Invoke-Mpremote -Args @("connect", "__PORT__", "soft-reset") -Retries 3 -SleepMs 400 -PortName $resolvedPort | Out-Null
Start-Sleep -Milliseconds 200

Write-Host "[2/4] 上传根配置与启动入口..." -ForegroundColor Cyan
Invoke-Mpremote -Args @("connect", "__PORT__", "fs", "cp", (Join-Path $projectRoot "Module\config.py"), ":config.py") -Retries 4 -SleepMs 500 -PortName $resolvedPort | Out-Null
Invoke-Mpremote -Args @("connect", "__PORT__", "fs", "cp", (Join-Path $projectRoot "APP\main.py"), ":main.py") -Retries 4 -SleepMs 500 -PortName $resolvedPort | Out-Null

Write-Host "[3/4] 上传 APP/BSP/Module 目录..." -ForegroundColor Cyan
Invoke-Mpremote -Args @("connect", "__PORT__", "fs", "cp", "-r", (Join-Path $projectRoot "APP"), ":APP") -Retries 4 -SleepMs 600 -PortName $resolvedPort | Out-Null
Invoke-Mpremote -Args @("connect", "__PORT__", "fs", "cp", "-r", (Join-Path $projectRoot "BSP"), ":BSP") -Retries 4 -SleepMs 600 -PortName $resolvedPort | Out-Null
Invoke-Mpremote -Args @("connect", "__PORT__", "fs", "cp", "-r", (Join-Path $projectRoot "Module"), ":Module") -Retries 4 -SleepMs 600 -PortName $resolvedPort | Out-Null

Write-Host "[3.5/4] 校验板端文件是否完整..." -ForegroundColor Cyan
$checkScript = @'
import os
root = "/flash"
try:
    ls = os.listdir(root)
except Exception:
    root = "."
    ls = os.listdir(root)

need = ("APP", "BSP", "Module", "main.py", "config.py")
missing = [x for x in need if x not in ls]
print("DEPLOY_ROOT", root)
print("FLASH_LS", ls)
print("MISSING", missing)
'@

$checkStr = Invoke-Mpremote -Args @("connect", "__PORT__", "exec", $checkScript) -Retries 3 -SleepMs 400 -PortName $resolvedPort
Write-Host $checkStr

if ($checkStr -notmatch 'MISSING\s*\[\s*\]') {
    throw "部署校验失败：板端缺少必要目录/文件（见 MISSING 列表）。请关闭占用串口的软件后重试部署。"
}

Write-Host "[4/4] 软复位..." -ForegroundColor Cyan
Invoke-Mpremote -Args @("connect", "__PORT__", "soft-reset") -Retries 3 -SleepMs 400 -PortName $resolvedPort | Out-Null

Write-Host "部署完成: $resolvedPort" -ForegroundColor Green
