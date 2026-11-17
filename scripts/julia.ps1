param(
    [Parameter(Position = 0)]
    [string]$Command = "check",

    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ExtraArgs
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$SrcDir   = Join-Path $RepoRoot "src"
$LocalJulia = Join-Path $RepoRoot ".julia-local\julia-1.12.1\bin\julia.exe"
$LocalDepot = Join-Path $RepoRoot ".julia-local\depot"

function Resolve-JuliaExecutable {
    $candidates = New-Object System.Collections.Generic.List[string]

    if ($env:JULIA_BIN -and (Test-Path -LiteralPath $env:JULIA_BIN -PathType Leaf)) {
        $candidates.Add($env:JULIA_BIN)
    }
    if (Test-Path -LiteralPath $LocalJulia -PathType Leaf) {
        $candidates.Add($LocalJulia)
    }

    $userJuliaup = Join-Path $env:USERPROFILE ".juliaup\bin\julia.exe"
    if (Test-Path -LiteralPath $userJuliaup -PathType Leaf) {
        $candidates.Add($userJuliaup)
    }

    $localProgJulia = Join-Path $env:LOCALAPPDATA "Programs\Julia-1.12.1\bin\julia.exe"
    if (Test-Path -LiteralPath $localProgJulia -PathType Leaf) {
        $candidates.Add($localProgJulia)
    }

    $cmdMatches = Get-Command julia -All -ErrorAction SilentlyContinue
    foreach ($cmd in $cmdMatches) {
        $src = $cmd.Source
        if ($src -and ($src -notlike "*\Microsoft\WindowsApps\*") -and (Test-Path -LiteralPath $src -PathType Leaf)) {
            $candidates.Add($src)
        }
    }

    foreach ($cand in ($candidates | Select-Object -Unique)) {
        $verOut = & $cand --startup-file=no --version 2>$null
        if ($LASTEXITCODE -eq 0 -and $verOut -match "1\.12\.1") {
            return $cand
        }
    }

    throw "Julia 1.12.1 executable was not found (WindowsApps aliases were skipped). Checked .julia-local and standard install paths."
}

$JuliaExe = Resolve-JuliaExecutable
$UsingLocalDepot = ($JuliaExe -eq $LocalJulia) -and (Test-Path -LiteralPath $LocalDepot -PathType Container)

if ($UsingLocalDepot) {
    $env:JULIA_DEPOT_PATH = $LocalDepot
}
$env:GKSwstype = "nul"

$TargetScript = $null
$JuliaFlags = @("--startup-file=no", "--project=$SrcDir")
$UsePrecompileFastFlags = $true

switch ($Command.ToLowerInvariant()) {
    "setup" {
        $TargetScript = Join-Path $PSScriptRoot "setup.jl"
        $UsePrecompileFastFlags = $false
    }
    "check" {
        $TargetScript = Join-Path $PSScriptRoot "check_environment.jl"
    }
    "test" {
        $TargetScript = Join-Path $PSScriptRoot "test_section8.jl"
    }
    "single" {
        $TargetScript = Join-Path $SrcDir "Sim.jl"
    }
    "multi" {
        $TargetScript = Join-Path $SrcDir "Sim_mobs.jl"
    }
    "fig2" {
        $TargetScript = Join-Path $SrcDir "Fig2.jl"
    }
    "fig3" {
        $TargetScript = Join-Path $SrcDir "Fig3.jl"
    }
    "fig4" {
        $TargetScript = Join-Path $SrcDir "Fig4.jl"
    }
    default {
        if (Test-Path -LiteralPath $Command -PathType Leaf) {
            $TargetScript = (Resolve-Path -LiteralPath $Command).Path
        } elseif (Test-Path -LiteralPath (Join-Path $RepoRoot $Command) -PathType Leaf) {
            $TargetScript = (Resolve-Path -LiteralPath (Join-Path $RepoRoot $Command)).Path
        } else {
            throw "Unknown command or script path: '$Command'. Supported: setup, check, single, multi, fig2, fig3, fig4, or a .jl file path."
        }
    }
}

if ($UsePrecompileFastFlags) {
    $env:JULIA_PKG_PRECOMPILE_AUTO = "0"
    if ($UsingLocalDepot) {
        $JuliaFlags += @("--pkgimages=no", "--compiled-modules=existing")
    }
}

Push-Location $RepoRoot
try {
    & $JuliaExe @JuliaFlags $TargetScript @ExtraArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Julia exited with code $LASTEXITCODE while running $TargetScript"
    }
} finally {
    Pop-Location
}
