[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string]$Checkpoint,
    [ValidateSet("flat", "rough")] [string]$Task = "flat",
    [int]$NumEnvs = 1,
    [int]$Steps = 100000,
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot "runtime"),
    [string]$Experience = "",
    [switch]$Keyboard,
    [double]$Vx = 0.0,
    [double]$Vy = 0.0,
    [double]$YawRate = 0.0,
    [double]$PushBodyWeight = 0.20,
    [double]$PushDuration = 0.10
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$pythonExe = Join-Path $RuntimeRoot ".venv\Scripts\python.exe"
$playScript = Join-Path $repoRoot "isaac_lab\play.py"
$resolvedCheckpoint = (Resolve-Path -LiteralPath $Checkpoint).Path
if (-not (Test-Path $pythonExe)) { throw "Native runtime is not installed. Run .\Install-YertleNative.ps1 first." }
if (-not (Test-Path $playScript)) { throw "play.py was not found at $playScript." }
if ($NumEnvs -lt 1 -or $Steps -lt 1) { throw "NumEnvs and Steps must be at least 1." }

$env:ACCEPT_EULA = "Y"
$env:PRIVACY_CONSENT = "Y"
$arguments = @($playScript, "--checkpoint", $resolvedCheckpoint, "--task", $Task, "--num_envs", $NumEnvs, "--steps", $Steps, "--rendering_mode", "performance", "--vx", $Vx, "--vy", $Vy, "--yaw-rate", $YawRate, "--push-body-weight", $PushBodyWeight, "--push-duration", $PushDuration)
if ($Experience) { $arguments += @("--experience", $Experience) }
if ($Keyboard) { $arguments += "--keyboard" }
Write-Host "Launching native Isaac Sim player with checkpoint: $resolvedCheckpoint"
& $pythonExe @arguments
exit $LASTEXITCODE
