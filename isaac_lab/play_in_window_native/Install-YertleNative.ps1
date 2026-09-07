[CmdletBinding()]
param(
    [string]$RuntimeRoot = (Join-Path $PSScriptRoot "runtime"),
    [switch]$SkipPythonInstall,
    [switch]$SkipUsdConversion
)

$ErrorActionPreference = "Stop"
$scriptRoot = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent $scriptRoot)
$requirements = Join-Path $scriptRoot "requirements.native.txt"
$isaacLabRoot = Join-Path $RuntimeRoot "IsaacLab"
$venvRoot = Join-Path $RuntimeRoot ".venv"
$pythonExe = Join-Path $venvRoot "Scripts\python.exe"
$isaacLabBat = Join-Path $isaacLabRoot "isaaclab.bat"

function Get-Python311 {
    $candidate = Get-Command py -ErrorAction SilentlyContinue
    if ($candidate) {
        & py -3.11 -c "import sys; print(sys.executable)" 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { return $true }
    }
    return $false
}

function Get-GitExe {
    $command = Get-Command git -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    $defaultPath = Join-Path $env:ProgramFiles "Git\cmd\git.exe"
    if (Test-Path $defaultPath) { return $defaultPath }
    return $null
}

if (-not (Get-Command nvidia-smi -ErrorAction SilentlyContinue)) {
    Write-Warning "nvidia-smi was not found. Install a supported NVIDIA Windows driver before running the player."
} else {
    & nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
}

if (-not (Get-Python311)) {
    if ($SkipPythonInstall) { throw "Python 3.11 was not found. Install it, then run this script again." }
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "Python 3.11 is required and winget is unavailable. Install Python 3.11, then rerun this script."
    }
    Write-Host "Installing Python 3.11 with winget..."
    & winget install --exact --id Python.Python.3.11 --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw "winget could not install Python 3.11." }
    if (-not (Get-Python311)) {
        throw "Python 3.11 was installed but is not available in this terminal. Open a new PowerShell window and rerun this script."
    }
}
$gitExe = Get-GitExe
if (-not $gitExe) {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "Git is required and winget is unavailable. Install Git for Windows, then rerun this script."
    }
    Write-Host "Installing Git for Windows with winget..."
    & winget install --exact --id Git.Git --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw "winget could not install Git for Windows." }
    $gitExe = Get-GitExe
    if (-not $gitExe) { throw "Git was installed but is not available in this terminal. Open a new PowerShell window and rerun this script." }
}

New-Item -ItemType Directory -Force -Path $RuntimeRoot | Out-Null
if (-not (Test-Path $pythonExe)) {
    Write-Host "Creating Python 3.11 virtual environment..."
    & py -3.11 -m venv $venvRoot
}

& $pythonExe -m pip install --upgrade pip
& $pythonExe -m pip install -r $requirements

if (-not (Test-Path (Join-Path $isaacLabRoot ".git"))) {
    Write-Host "Cloning Isaac Lab v2.3.2..."
    & $gitExe clone --depth 1 --branch v2.3.2 https://github.com/isaac-sim/IsaacLab.git $isaacLabRoot
} else {
    $installedRef = (& $gitExe -C $isaacLabRoot describe --tags --exact-match 2>$null)
    if ($installedRef -ne "v2.3.2") {
        throw "Existing Isaac Lab checkout is '$installedRef', not v2.3.2. Delete $isaacLabRoot and rerun, or choose another RuntimeRoot."
    }
}

Write-Host "Installing Isaac Lab core and RSL-RL..."
& $isaacLabBat --install rsl_rl
if ($LASTEXITCODE -ne 0) { throw "Isaac Lab RSL-RL installation failed." }

$urdfPath = Join-Path $repoRoot "simulation\yertle.urdf"
$usdPath = Join-Path $repoRoot "simulation\usd\yertle.usd"
if (-not $SkipUsdConversion -and -not (Test-Path $usdPath)) {
    if (-not (Test-Path $urdfPath)) { throw "Yertle URDF was not found at $urdfPath." }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $usdPath) | Out-Null
    Write-Host "Converting Yertle URDF to USD..."
    & $isaacLabBat -p (Join-Path $isaacLabRoot "scripts\tools\convert_urdf.py") $urdfPath $usdPath --merge-joints --headless
    if ($LASTEXITCODE -ne 0) { throw "URDF to USD conversion failed." }
}

$manifest = [ordered]@{
    python = (& $pythonExe -c "import sys; print(sys.version.split()[0])")
    isaac_sim = "5.1.0"
    isaac_lab = "2.3.2"
    runtime_root = $RuntimeRoot
    installed_at = (Get-Date).ToUniversalTime().ToString("o")
}
$manifest | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $RuntimeRoot "yertle-native-runtime.json")
Write-Host "Installation complete. Run .\Play-YertleNative.ps1 -Checkpoint <model_XXXX.pt>"
