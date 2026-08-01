param(
    [ValidateSet('High', 'Low')]
    [string]$ActiveLevel = 'High',
    [switch]$Watch,
    [switch]$Interactive,
    [ValidateRange(50, 10000)]
    [int]$IntervalMs = 300,
    [ValidateRange(1, 300)]
    [int]$StepTimeoutSeconds = 30,
    [string]$ProgrammerPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# STM32G47x GPIOE base is 0x48001000 and IDR is at offset 0x10.
# The six limit inputs are J1..J6 on PE7..PE12.
$GpioeIdrAddress = '0x48001010'
$LimitPins = @(
    [pscustomobject]@{ Joint = 'J1'; Pin = 'PE7';  Bit = 7  }
    [pscustomobject]@{ Joint = 'J2'; Pin = 'PE8';  Bit = 8  }
    [pscustomobject]@{ Joint = 'J3'; Pin = 'PE9';  Bit = 9  }
    [pscustomobject]@{ Joint = 'J4'; Pin = 'PE10'; Bit = 10 }
    [pscustomobject]@{ Joint = 'J5'; Pin = 'PE11'; Bit = 11 }
    [pscustomobject]@{ Joint = 'J6'; Pin = 'PE12'; Bit = 12 }
)

# J2 and J6 are not wired on the current assembly. Keep their GPIO values
# visible for diagnostics, but exclude them from the test and pass/fail result.
$DisabledJointNames = @('J2', 'J6')
$EnabledLimitPins = @(
    $LimitPins | Where-Object { $DisabledJointNames -notcontains $_.Joint }
)

function Resolve-ProgrammerPath {
    param([string]$RequestedPath)

    if ($RequestedPath) {
        if (-not (Test-Path -LiteralPath $RequestedPath -PathType Leaf)) {
            throw "STM32_Programmer_CLI not found: $RequestedPath"
        }
        return (Resolve-Path -LiteralPath $RequestedPath).Path
    }

    $command = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    $candidates = @(
        'C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    throw @'
STM32_Programmer_CLI.exe was not found. Install STM32CubeProgrammer or pass:
  -ProgrammerPath 'C:\path\to\STM32_Programmer_CLI.exe'
'@
}

function Read-GpioeIdr {
    param([string]$CliPath)

    # HOTPLUG avoids resetting the MCU. This command only reads one peripheral
    # register and sends no UART/CAN motion command.
    $output = & $CliPath `
        '-c' 'port=SWD' 'mode=HOTPLUG' `
        '-r32' $GpioeIdrAddress '4' 2>&1 | Out-String
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "ST-Link read failed (exit $exitCode):`n$output"
    }

    $pattern = '(?im)^\s*0x48001010\s*:\s*([0-9a-f]{8})\s*$'
    $match = [regex]::Match($output, $pattern)
    if (-not $match.Success) {
        throw "GPIOE_IDR value was not found in programmer output:`n$output"
    }

    return [Convert]::ToUInt32($match.Groups[1].Value, 16)
}

function Get-LimitSnapshot {
    param(
        [uint32]$Idr,
        [string]$ConfiguredActiveLevel
    )

    $activeHigh = $ConfiguredActiveLevel -eq 'High'
    $snapshot = [ordered]@{}
    foreach ($limit in $LimitPins) {
        $high = (($Idr -shr $limit.Bit) -band 1) -ne 0
        $snapshot[$limit.Joint] = [pscustomobject]@{
            Joint = $limit.Joint
            Pin = $limit.Pin
            High = $high
            Disabled = $DisabledJointNames -contains $limit.Joint
            Active = if ($activeHigh) { $high } else { -not $high }
        }
    }
    return $snapshot
}

function Show-LimitSnapshot {
    param(
        [uint32]$Idr,
        [System.Collections.IDictionary]$Snapshot
    )

    Write-Host ('GPIOE_IDR=0x{0:X8}  active_level={1}' -f $Idr, $ActiveLevel)
    foreach ($item in $Snapshot.Values) {
        if ($item.Disabled) {
            Write-Host ('  {0} {1,-4}  DISABLED (not wired)' -f $item.Joint, $item.Pin)
            continue
        }
        $level = if ($item.High) { 'HIGH' } else { 'LOW ' }
        $state = if ($item.Active) { 'ACTIVE' } else { 'inactive' }
        Write-Host ('  {0} {1,-4}  {2}  {3}' -f
            $item.Joint, $item.Pin, $level, $state)
    }
}

function Wait-ForJointLevel {
    param(
        [string]$CliPath,
        [string]$Joint,
        [bool]$ExpectedHigh,
        [int]$TimeoutSeconds
    )

    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        $idr = Read-GpioeIdr $CliPath
        $snapshot = Get-LimitSnapshot $idr $ActiveLevel
        if ($snapshot[$Joint].High -eq $ExpectedHigh) {
            return $true
        }
        Start-Sleep -Milliseconds $IntervalMs
    }
    return $false
}

function Invoke-InteractiveTest {
    param([string]$CliPath)

    $initialIdr = Read-GpioeIdr $CliPath
    $baseline = Get-LimitSnapshot $initialIdr $ActiveLevel
    Show-LimitSnapshot $initialIdr $baseline
    Write-Host ''
    Write-Host 'Operate and release each requested switch. No motor command will be sent.'

    $failed = @()
    foreach ($limit in $EnabledLimitPins) {
        $joint = $limit.Joint
        $baselineHigh = $baseline[$joint].High
        Write-Host ''
        Write-Host ("[$joint] Operate the switch and hold it...")
        if (-not (Wait-ForJointLevel $CliPath $joint (-not $baselineHigh) $StepTimeoutSeconds)) {
            Write-Host "[$joint] FAIL: no level change before timeout." -ForegroundColor Red
            $failed += $joint
            continue
        }

        Write-Host "[$joint] Level change detected; release the switch..."
        if (-not (Wait-ForJointLevel $CliPath $joint $baselineHigh $StepTimeoutSeconds)) {
            Write-Host "[$joint] FAIL: signal did not return to baseline." -ForegroundColor Red
            $failed += $joint
            continue
        }

        Write-Host "[$joint] PASS" -ForegroundColor Green
    }

    Write-Host ''
    if ($failed.Count -gt 0) {
        throw ('Limit switch test failed: {0}' -f ($failed -join ', '))
    }
    Write-Host 'All six limit switches passed.' -ForegroundColor Green
}

$cliPath = Resolve-ProgrammerPath $ProgrammerPath

if ($Interactive) {
    Invoke-InteractiveTest $cliPath
    exit 0
}

$previousIdr = $null
do {
    $idr = Read-GpioeIdr $cliPath
    if ($null -eq $previousIdr -or $idr -ne $previousIdr) {
        $snapshot = Get-LimitSnapshot $idr $ActiveLevel
        Show-LimitSnapshot $idr $snapshot
        if ($Watch) {
            Write-Host ('sample_time={0:yyyy-MM-dd HH:mm:ss.fff}' -f (Get-Date))
            Write-Host ''
        }
        $previousIdr = $idr
    }

    if ($Watch) {
        Start-Sleep -Milliseconds $IntervalMs
    }
} while ($Watch)
