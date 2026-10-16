param([string]$PythonPath='D:\tools\isg\e\Scripts\python.exe',[int]$Frames=0)
$ErrorActionPreference='Stop'
$sourceDirectory=(Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$scriptPath=Join-Path $PSScriptRoot 'sole_region_viewer.py'
if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) { throw "Python not found: $PythonPath" }
$env:OMNI_KIT_ACCEPT_EULA='YES'
$env:PYTHONUNBUFFERED='1'
Push-Location -LiteralPath $sourceDirectory
try {
    & $PythonPath $scriptPath --frames $Frames --device cpu --rendering_mode performance --experience isaaclab.python.yertle_min_d3d12.kit
    if ($LASTEXITCODE -ne 0) { throw "Sole viewer failed: $LASTEXITCODE" }
}
finally { Pop-Location }
