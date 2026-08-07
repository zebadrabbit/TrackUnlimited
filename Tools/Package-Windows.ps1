<#
.SYNOPSIS
    Packages TrackUnlimited for Windows via Unreal Automation Tool.

.DESCRIPTION
    The reproducible package command, in the repo, so "how do I build this"
    has an answer that is not somebody's shell history.

    UNVERIFIED AS WRITTEN. Nobody has run this yet — it was written from the
    documented UAT interface rather than from a successful build, and the
    Phase 3.5 packaging card lists the failures that are expected on the first
    real attempt. Treat the first run as the test.

.PARAMETER EnginePath
    Root of the Unreal Engine install, e.g. "C:\Program Files\Epic Games\UE_5.8".
    Taken from -EnginePath, then $env:UE_ROOT, then the engine association in
    the .uproject via the registry. Deliberately NOT hardcoded: it differs per
    machine, and a path baked into a script is the first thing to break for a
    contributor.

.PARAMETER Configuration
    Development (default) or Shipping.

    Development first, on purpose. Shipping compiles out DrawDebug entirely, so
    the ride-profile traces, the block-boundary gates and the restraint boxes
    all vanish — and discovering that at the same time as every other packaging
    failure makes both harder to diagnose.

.EXAMPLE
    .\Tools\Package-Windows.ps1
    .\Tools\Package-Windows.ps1 -Configuration Shipping -Output D:\builds\tu
#>
[CmdletBinding()]
param(
    [string] $EnginePath,
    [ValidateSet('Development', 'Shipping')]
    [string] $Configuration = 'Development',
    [string] $Output
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Uproject = Join-Path $RepoRoot 'TrackUnlimited.uproject'
if (-not (Test-Path $Uproject)) {
    throw "TrackUnlimited.uproject not found at $Uproject. Run this from the repo."
}

# ---- Find the engine, and say clearly what to do if we cannot.
if (-not $EnginePath) { $EnginePath = $env:UE_ROOT }
if (-not $EnginePath) {
    # The .uproject records an engine association; for a launcher install that is
    # a version string the registry maps to a path.
    $Assoc = (Get-Content $Uproject -Raw | ConvertFrom-Json).EngineAssociation
    if ($Assoc) {
        $Key = 'HKLM:\SOFTWARE\EpicGames\Unreal Engine\' + $Assoc
        if (Test-Path $Key) {
            $EnginePath = (Get-ItemProperty $Key).InstalledDirectory
        }
    }
}
if (-not $EnginePath -or -not (Test-Path $EnginePath)) {
    throw @"
Could not find Unreal Engine.

Pass it explicitly:      .\Tools\Package-Windows.ps1 -EnginePath "C:\Program Files\Epic Games\UE_5.8"
or set it once:          `$env:UE_ROOT = "C:\Program Files\Epic Games\UE_5.8"

Deliberately not hardcoded — it differs per machine, and a baked-in path is the
first thing to break for a contributor.
"@
}

$RunUat = Join-Path $EnginePath 'Engine\Build\BatchFiles\RunUAT.bat'
if (-not (Test-Path $RunUat)) {
    throw "RunUAT.bat not found under $EnginePath. Is that an engine root rather than a project?"
}

if (-not $Output) { $Output = Join-Path $RepoRoot ('Build\Windows-' + $Configuration) }

Write-Host "TrackUnlimited package" -ForegroundColor Cyan
Write-Host "  engine        $EnginePath"
Write-Host "  configuration $Configuration"
Write-Host "  output        $Output"
Write-Host ""

# -nop4        we are not on Perforce
# -build       compile the game module rather than assuming a prior editor build
# -cook        content, for the target platform
# -stage/-pak  lay it out and pak it, which is what a real download looks like
# -archive     copy the result somewhere predictable
# -nodebuginfo keeps the artefact small; drop it when a crash needs symbols
$UatArgs = @(
    'BuildCookRun'
    "-project=`"$Uproject`""
    '-noP4'
    '-platform=Win64'
    "-clientconfig=$Configuration"
    '-build'
    '-cook'
    '-stage'
    '-pak'
    '-archive'
    "-archivedirectory=`"$Output`""
    '-utf8output'
)
if ($Configuration -eq 'Shipping') { $UatArgs += '-nodebuginfo' }

Write-Host "> RunUAT.bat $($UatArgs -join ' ')" -ForegroundColor DarkGray
& $RunUat @UatArgs
if ($LASTEXITCODE -ne 0) {
    throw "UAT failed with exit code $LASTEXITCODE. The log is under $EnginePath\Engine\Programs\AutomationTool\Saved\Logs."
}

Write-Host ""
Write-Host "Packaged to $Output" -ForegroundColor Green
Write-Host @"

THE SMOKE TEST IS NOT AUTOMATED YET. Run the .exe, confirm it boots and that a
preset layout loads and runs. The packaging card lists what usually breaks
first:

  - OnConstruction drives the editor preview and does NOT run in a packaged
    game. Anything that only rebuilds there will be missing.
  - DrawDebug is compiled out in Shipping, so the ride-profile traces, the
    block-boundary gates and the restraint boxes disappear rather than fail.
  - Assets referenced only from the editor are cooked out unless explicitly
    referenced.
  - Background throttling: an unfocused window runs the sim SLOWLY. That is now
    safe — the step is fixed, so it stays bit-identical — but whether a packaged
    build should slow, pause or run at full rate is still an open product
    decision on that card.
"@ -ForegroundColor DarkGray
