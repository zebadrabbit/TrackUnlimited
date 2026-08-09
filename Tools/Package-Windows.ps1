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
    [string] $Output,

    # Run the packaged build's own smoke test and fail if it does not pass.
    [switch] $SmokeTest
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

if (-not $SmokeTest) {
    Write-Host ""
    Write-Host "Pass -SmokeTest to run it. Without that this only proves it COMPILED and COOKED." -ForegroundColor DarkGray
    return
}

# ---- The packaged smoke test.
#
# The half the packaging card was waiting on. -TUSmokeTest already existed and
# had only ever run in the editor, which is the one place the failures it exists
# to catch cannot happen: OnConstruction runs there, uncooked assets resolve
# there, and the map you have open is the map you get.
$Exe = Get-ChildItem -Path $Output -Filter 'TrackUnlimited.exe' -Recurse -File |
       Select-Object -First 1
if (-not $Exe) { throw "Packaged TrackUnlimited.exe not found under $Output." }

# -nullrhi and -nosound so it runs without a GPU or an audio device, which is
# what a CI agent has. -unattended so nothing waits for a dialog.
$SmokeArgs = @('-TUSmokeTest', '-unattended', '-nullrhi', '-nosound',
               '-stdout', '-FullStdOutLogOutput')

Write-Host ""
Write-Host "Smoke test: $($Exe.FullName)" -ForegroundColor Cyan
$Log = & $Exe.FullName @SmokeArgs 2>&1
$Status = $LASTEXITCODE
$Log | Where-Object { $_ -match 'smoke:' } | ForEach-Object { Write-Host "  $_" }

# ---- TWO CONDITIONS, AND THE SECOND IS THE ONE THAT MATTERS.
#
# MEASURED, NOT ASSUMED: a run that reported FAILED still exited 0 under
# UnrealEditor-Cmd in -game. RequestExitWithStatus asks for a graceful shutdown
# and the status does not survive it there. Whether a packaged build does better
# is untested -- so the exit code is treated as a hint and the VERDICT LINE is
# the authority. Forcing the exit instead would guarantee the code and risk
# truncating the log that carries the reason, which is the worse trade.
#
# A build whose default map has no ATUCoasterRide in it never runs BeginPlay, so
# the test never executes and the process exits 0 -- indistinguishable from a
# pass by status alone. So the verdict LINE must be present, not merely
# non-failing. A check that cannot tell success from never-having-run is worse
# than no check, because it gets believed.
$Verdict = $Log | Where-Object { $_ -match 'smoke: (PASSED|FAILED)' } | Select-Object -Last 1
if (-not $Verdict) {
    $Tail = ($Log | Select-Object -Last 25) -join "`n"
    throw @"
The smoke test never reported a verdict (process exit $Status).

That is NOT a pass. It means the run never reached ATUCoasterRide::BeginPlay --
most likely the packaged build opened a map with no ride actor in it. Check
GameDefaultMap in Config/DefaultEngine.ini and MapsToCook in DefaultGame.ini.

Last of the output:
$Tail
"@
}
if ($Status -ne 0 -or $Verdict -notmatch 'PASSED') {
    throw "Smoke test FAILED (exit $Status). The 'smoke:' lines above say which check."
}

Write-Host ""
Write-Host "Smoke test passed." -ForegroundColor Green
Write-Host @"

WHAT THIS DOES AND DOES NOT COVER. It proves the packaged build boots, loads a
map with the ride in it, and that the document layer survives cooking -- save,
open, undo, redo, insert, remove, multi-select, the presets and the browser's
four row states. It does NOT look at anything on screen. These still want an eye:

  - DrawDebug is compiled out in Shipping, so the ride-profile traces, the
    block-boundary gates and the restraint boxes disappear rather than fail.
  - OnConstruction drives the editor preview and does NOT run in a packaged
    game. Anything that only rebuilds there will be missing.
  - Assets referenced only from the editor are cooked out unless referenced.
  - Background throttling: whether an unfocused packaged build should slow,
    pause or run at full rate is still an open product decision on that card.
"@ -ForegroundColor DarkGray
