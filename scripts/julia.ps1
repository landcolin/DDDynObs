param(
    [Parameter(Position = 0)]
    [ValidateSet('setup', 'check', 'single', 'multi', 'repl')]
    [string]$Task = 'repl',
    [string]$JuliaExecutable = $env:JULIA_EXE
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $projectRoot 'src'
$manifest = Get-Content -LiteralPath (Join-Path $projectPath 'Manifest.toml') -Raw
$requiredVersion = [regex]::Match($manifest, '(?m)^julia_version\s*=\s*"([^"]+)"').Groups[1].Value

# Resolve the actual juliaup installation, avoiding inaccessible WindowsApps aliases.
if (-not $JuliaExecutable) {
    $juliaupRoot = Join-Path $env:USERPROFILE '.julia\juliaup'
    $juliaupConfig = Join-Path $juliaupRoot 'juliaup.json'
    if (Test-Path -LiteralPath $juliaupConfig) {
        $config = Get-Content -LiteralPath $juliaupConfig -Raw | ConvertFrom-Json
        $installed = $config.InstalledVersions.PSObject.Properties |
            Where-Object { $_.Name -like "$requiredVersion+*" } |
            Select-Object -First 1
        if ($installed) {
            $JuliaExecutable = Join-Path $juliaupRoot $installed.Value.BinaryPath
        }
    }
}
if (-not $JuliaExecutable) {
    $command = Get-Command julia -ErrorAction SilentlyContinue
    if ($command) { $JuliaExecutable = $command.Source }
}
if (-not $JuliaExecutable -or -not (Test-Path -LiteralPath $JuliaExecutable)) {
    throw "Julia $requiredVersion was not found. Install it with juliaup, or pass -JuliaExecutable <path>."
}

$version = & $JuliaExecutable --startup-file=no -e 'print(VERSION)'
if ($LASTEXITCODE -ne 0 -or "$version" -ne $requiredVersion) {
    throw "This project requires Julia $requiredVersion; found '$version' at $JuliaExecutable."
}

$arguments = @('--startup-file=no', "--project=$projectPath")
switch ($Task) {
    'setup'  { $arguments += Join-Path $PSScriptRoot 'setup.jl' }
    'check'  { $arguments += Join-Path $PSScriptRoot 'check_environment.jl' }
    'single' { $arguments += Join-Path $projectPath 'Sim.jl' }
    'multi'  { $arguments += Join-Path $projectPath 'Sim_mobs.jl' }
}
& $JuliaExecutable @arguments
if ($LASTEXITCODE -ne 0) { throw "Julia task '$Task' failed with exit code $LASTEXITCODE." }
