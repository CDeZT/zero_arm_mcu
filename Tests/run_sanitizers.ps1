param(
    [string]$ToolchainRoot = '.tools/llvm-mingw-20260616-ucrt-x86_64',
    [string]$BuildDirectory = 'build/host-tests-sanitize'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$toolchain = (Resolve-Path -LiteralPath $ToolchainRoot).Path
$compiler = Join-Path $toolchain 'bin/clang.exe'
$runtimeBin = Join-Path $toolchain 'x86_64-w64-mingw32/bin'
if (-not (Test-Path -LiteralPath $compiler)) {
    throw "LLVM-MinGW clang not found: $compiler"
}

$compilerForCMake = $compiler.Replace('\', '/')
$env:Path = "$(Join-Path $toolchain 'bin');$runtimeBin;$env:Path"

& cmake `
    -S Tests `
    -B $BuildDirectory `
    -G Ninja `
    -DCMAKE_BUILD_TYPE=Debug `
    "-DCMAKE_C_COMPILER=$compilerForCMake" `
    '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all' `
    '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
if ($LASTEXITCODE -ne 0) {
    throw "Sanitizer configure failed: $LASTEXITCODE"
}

& cmake --build $BuildDirectory --parallel
if ($LASTEXITCODE -ne 0) {
    throw "Sanitizer build failed: $LASTEXITCODE"
}

$ctestJson = & ctest `
    --test-dir $BuildDirectory `
    --show-only=json-v1
if ($LASTEXITCODE -ne 0) {
    throw "CTest discovery failed: $LASTEXITCODE"
}

$testPlan = $ctestJson | ConvertFrom-Json
$tests = @($testPlan.tests)
if ($tests.Count -eq 0) {
    throw 'CTest discovery returned no tests'
}

foreach ($test in $tests) {
    $command = @($test.command)
    if ($command.Count -eq 0) {
        throw "CTest entry has no command: $($test.name)"
    }
    $arguments = if ($command.Count -gt 1) {
        @($command[1..($command.Count - 1)])
    } else {
        @()
    }

    Write-Output "sanitizer_run=$($test.name)"
    Push-Location -LiteralPath $BuildDirectory
    try {
        & $command[0] @arguments
        if ($LASTEXITCODE -ne 0) {
            throw (
                "Sanitizer test failed: $($test.name) " +
                "($LASTEXITCODE)")
        }
    } finally {
        Pop-Location
    }
}

Write-Output "sanitizer=PASS tests=$($tests.Count)"
