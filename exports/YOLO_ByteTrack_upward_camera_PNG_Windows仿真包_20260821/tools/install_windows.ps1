[CmdletBinding()]
param([switch]$Elevated)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
$PackageRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RuntimeDir = Join-Path $PackageRoot "runtime"
$StatePath = Join-Path $RuntimeDir "install_stage.txt"
$Distro = "Ubuntu-20.04"
$BlocksUrl = "https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip"
$BlocksZipSize = 142533463

function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Set-InstallStage([string]$Stage) {
    New-Item -ItemType Directory -Force -Path $RuntimeDir | Out-Null
    Set-Content -Encoding UTF8 -Path $StatePath -Value $Stage
}

function Get-Python310Command {
    if (Get-Command py.exe -ErrorAction SilentlyContinue) {
        & py.exe -3.10 -c "import sys; assert sys.version_info[:2] == (3, 10)" 2>$null
        if ($LASTEXITCODE -eq 0) { return @("py.exe", "-3.10") }
    }
    if (Get-Command python.exe -ErrorAction SilentlyContinue) {
        & python.exe -c "import sys; assert sys.version_info[:2] == (3, 10)" 2>$null
        if ($LASTEXITCODE -eq 0) { return @("python.exe") }
    }
    return $null
}

function Invoke-Python([string[]]$PythonCommand, [string[]]$Arguments) {
    $exe = $PythonCommand[0]
    $prefix = @()
    if ($PythonCommand.Count -gt 1) { $prefix = $PythonCommand[1..($PythonCommand.Count - 1)] }
    & $exe @prefix @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Python command failed with exit code $LASTEXITCODE" }
}

function Set-MirroredNetworking {
    $path = Join-Path $env:USERPROFILE ".wslconfig"
    $lines = [System.Collections.Generic.List[string]]::new()
    if (Test-Path $path) {
        foreach ($line in Get-Content $path) { [void]$lines.Add([string]$line) }
    }
    $sectionStart = -1
    $sectionEnd = $lines.Count
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i].Trim().ToLowerInvariant() -eq "[wsl2]") {
            $sectionStart = $i
            for ($j = $i + 1; $j -lt $lines.Count; $j++) {
                if ($lines[$j].Trim().StartsWith("[")) { $sectionEnd = $j; break }
            }
            break
        }
    }
    if ($sectionStart -lt 0) {
        if ($lines.Count -gt 0 -and $lines[$lines.Count - 1] -ne "") { [void]$lines.Add("") }
        [void]$lines.Add("[wsl2]")
        [void]$lines.Add("networkingMode=mirrored")
    } else {
        $updated = $false
        for ($i = $sectionStart + 1; $i -lt $sectionEnd; $i++) {
            if ($lines[$i] -match "^\s*networkingMode\s*=") {
                $lines[$i] = "networkingMode=mirrored"
                $updated = $true
                break
            }
        }
        if (-not $updated) { $lines.Insert($sectionEnd, "networkingMode=mirrored") }
    }
    Set-Content -Encoding ASCII -Path $path -Value $lines
}

if (-not (Test-Administrator)) {
    Write-Host "Requesting administrator privileges for Windows features and WSL setup..."
    $arguments = @(
        "-NoLogo", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", ('"' + $PSCommandPath + '"'), "-Elevated"
    )
    $process = Start-Process powershell.exe -Verb RunAs -Wait -PassThru -ArgumentList $arguments
    exit $process.ExitCode
}

if ([Environment]::OSVersion.Version -lt [Version]"10.0.22621") {
    throw "Windows 11 22H2 (build 22621) or newer is required."
}

New-Item -ItemType Directory -Force -Path $RuntimeDir | Out-Null
Set-InstallStage "checking_nvidia"
if (-not (Get-Command nvidia-smi.exe -ErrorAction SilentlyContinue)) {
    throw "NVIDIA driver was not found. Install a current NVIDIA Windows driver, reboot, and rerun this installer."
}
& nvidia-smi.exe --query-gpu=name,driver_version --format=csv,noheader
if ($LASTEXITCODE -ne 0) { throw "nvidia-smi failed; repair the NVIDIA driver before continuing." }

Set-InstallStage "installing_python"
$python = Get-Python310Command
if (-not $python) {
    if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
        throw "winget is required for automatic Python installation. Install App Installer, then rerun."
    }
    & winget.exe install --id Python.Python.3.10 --exact --silent --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw "Python 3.10 installation failed." }
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [Environment]::GetEnvironmentVariable("Path", "User")
    $python = Get-Python310Command
    if (-not $python) { throw "Python 3.10 was installed but is not discoverable. Sign out and rerun the installer." }
}

Set-InstallStage "installing_python_dependencies"
$venvPython = Join-Path $PackageRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $venvPython)) {
    Invoke-Python $python @("-m", "venv", (Join-Path $PackageRoot ".venv"))
}
& $venvPython -m pip install --upgrade "pip==25.1.1" "setuptools==80.9.0" "wheel==0.45.1"
if ($LASTEXITCODE -ne 0) { throw "pip bootstrap failed." }
& $venvPython -m pip install --requirement (Join-Path $PackageRoot "requirements-windows.txt")
if ($LASTEXITCODE -ne 0) { throw "Python dependency installation failed." }
& $venvPython -c "import torch; assert torch.cuda.is_available(), 'CUDA unavailable in PyTorch'; print('torch', torch.__version__, 'cuda', torch.version.cuda, torch.cuda.get_device_name(0))"
if ($LASTEXITCODE -ne 0) { throw "Pinned CUDA PyTorch validation failed." }

Set-InstallStage "installing_blocks"
$blocksZip = Join-Path $RuntimeDir "Blocks-1.8.1.zip"
if ((-not (Test-Path $blocksZip)) -or ((Get-Item $blocksZip).Length -ne $BlocksZipSize)) {
    Remove-Item -Force -ErrorAction SilentlyContinue $blocksZip
    Invoke-WebRequest -UseBasicParsing -Uri $BlocksUrl -OutFile $blocksZip
}
if ((Get-Item $blocksZip).Length -ne $BlocksZipSize) {
    throw "Blocks.zip size check failed; expected $BlocksZipSize bytes."
}
$blocksDir = Join-Path $RuntimeDir "Blocks"
$blocksExeMarker = Join-Path $RuntimeDir "blocks_path.txt"
$blocksInstalled = $false
if (Test-Path $blocksExeMarker) {
    $markedBlocksExe = (Get-Content $blocksExeMarker -Raw).Trim()
    $blocksInstalled = [bool]($markedBlocksExe -and (Test-Path $markedBlocksExe))
}
if (-not $blocksInstalled) {
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $blocksDir
    New-Item -ItemType Directory -Force -Path $blocksDir | Out-Null
    Expand-Archive -Force -Path $blocksZip -DestinationPath $blocksDir
    $blocksExe = Get-ChildItem -Path $blocksDir -Filter "Blocks.exe" -File -Recurse | Select-Object -First 1
    if (-not $blocksExe) { throw "Blocks.exe was not found after extracting Blocks.zip." }
    Set-Content -Encoding UTF8 -Path $blocksExeMarker -Value $blocksExe.FullName
}

Set-InstallStage "enabling_wsl"
$restartNeeded = $false
foreach ($feature in @("Microsoft-Windows-Subsystem-Linux", "VirtualMachinePlatform")) {
    $state = Get-WindowsOptionalFeature -Online -FeatureName $feature
    if ($state.State -ne "Enabled") {
        $result = Enable-WindowsOptionalFeature -Online -FeatureName $feature -All -NoRestart
        if ($result.RestartNeeded) { $restartNeeded = $true }
    }
}
if ($restartNeeded) {
    Set-InstallStage "reboot_required_for_wsl"
    exit 3010
}

Set-MirroredNetworking
& wsl.exe --update
if ($LASTEXITCODE -ne 0) { throw "wsl --update failed." }
& wsl.exe --shutdown
$distros = @(& wsl.exe --list --quiet | ForEach-Object { $_.Replace([char]0, "").Trim() } | Where-Object { $_ })
if ($distros -notcontains $Distro) {
    & wsl.exe --install --distribution $Distro --no-launch
    if ($LASTEXITCODE -ne 0) { throw "Automatic installation of $Distro failed." }
    $distros = @(& wsl.exe --list --quiet | ForEach-Object { $_.Replace([char]0, "").Trim() } | Where-Object { $_ })
    if ($distros -notcontains $Distro) {
        Set-InstallStage "reboot_required_for_distribution"
        exit 3010
    }
}
& wsl.exe --set-default-version 2
if ($LASTEXITCODE -ne 0) { throw "Could not set WSL2 as the default version." }
& wsl.exe --set-version $Distro 2
if ($LASTEXITCODE -ne 0) { throw "Could not convert $Distro to WSL2." }

Set-InstallStage "installing_px4_v1.11.3"
$wslRoot = (& wsl.exe -d $Distro -u root -- wslpath -a $PackageRoot).Trim()
if (-not $wslRoot) { throw "Could not translate the package path into WSL." }
& wsl.exe -d $Distro -u root -- bash "$wslRoot/tools/setup_px4_wsl.sh"
if ($LASTEXITCODE -ne 0) { throw "PX4 v1.11.3 setup in WSL2 failed." }

Set-InstallStage "validating_package"
& $venvPython (Join-Path $PackageRoot "tools\check_package.py") --runtime
if ($LASTEXITCODE -ne 0) { throw "Package validation failed." }

Set-InstallStage "complete"
Write-Host ""
Write-Host "Installation complete. Run: run_experiments.bat smoke fast"
