# Yertle Windows-native player

This folder creates a reproducible Windows GUI runtime separate from Docker.
It pins Python 3.11, Isaac Sim 5.1.0, Isaac Lab 2.3.2, and RSL-RL.

## First installation

Open PowerShell in this folder and run:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\Install-YertleNative.ps1
```

The script installs Python 3.11 through `winget` when necessary, creates the
local `runtime/.venv`, installs the NVIDIA Isaac Sim pip packages, clones Isaac
Lab v2.3.2, installs RSL-RL, and generates `simulation/usd/yertle.usd` once.
It requires internet access, Git, an NVIDIA GPU driver, and several GB of disk
space. `runtime/`, checkpoints, and policies are excluded from Git.

## Run a policy with a Windows GUI

```powershell
.\Play-YertleNative.ps1 `
  -Checkpoint "C:\path\to\model_1999.pt" `
  -Task flat `
  -NumEnvs 1
```

The script launches the standard Isaac Lab GUI experience using the native
Windows renderer. Supply `-Keyboard` to use the Windows keyboard controls in
`play.py`. Add a custom compatible Kit file only when needed:

```powershell
.\Play-YertleNative.ps1 -Checkpoint "C:\path\to\model_1999.pt" `
  -Experience "C:\path\to\isaaclab.python.yertle_min_d3d12.kit"
```

The model must match the Yertle code/configuration that produced it.
