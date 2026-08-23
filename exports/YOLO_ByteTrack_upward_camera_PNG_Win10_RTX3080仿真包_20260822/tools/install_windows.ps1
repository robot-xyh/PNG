[CmdletBinding()]
param([switch]$Elevated)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$PackageRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RuntimeDir = Join-Path $PackageRoot "runtime"
$StatePath = Join-Path $RuntimeDir "install_stage.txt"
$Distro = "PNG-PX4-Ubuntu20.04"
$DistroInstall = Join-Path $env:LOCALAPPDATA "PNG-PX4\WSL\Ubuntu20.04"

$BlocksUrl = "https://github.com/microsoft/AirSim/releases/download/v1.8.1-windows/Blocks.zip"
$BlocksZipSize = 259463081
$BlocksSha256 = "47c526a5f0acff42c211d2479b9de9f10286a8162678ef19adc09040a23ca1db"

$RootfsUrl = "https://partner-images.canonical.com/core/focal/20230630/ubuntu-focal-core-cloudimg-amd64-root.tar.gz"
$RootfsSize = 27767198
$RootfsSha256 = "23a0b488bf439da0d5776748b39f668f726129dcc57d7b21544f1706652e2082"

function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Set-InstallStage([string]$Stage) {
    New-Item -ItemType Directory -Force -Path $RuntimeDir | Out-Null
    Set-Content -Encoding UTF8 -Path $StatePath -Value $Stage
    Write-Host "[stage] $Stage"
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

function Test-VerifiedFile([string]$Path, [long]$Size, [string]$Sha256) {
    if (-not (Test-Path $Path -PathType Leaf)) { return $false }
    if ((Get-Item $Path).Length -ne $Size) { return $false }
    $actual = (Get-FileHash -Algorithm SHA256 -Path $Path).Hash.ToLowerInvariant()
    return $actual -eq $Sha256.ToLowerInvariant()
}

function Invoke-VerifiedDownload([string]$Url, [string]$Path, [long]$Size, [string]$Sha256) {
    if (Test-VerifiedFile $Path $Size $Sha256) {
        Write-Host "Using verified download: $Path"
        return
    }
    Remove-Item -Force -ErrorAction SilentlyContinue $Path
    Write-Host "Downloading $Url"
    Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $Path
    if (-not (Test-VerifiedFile $Path $Size $Sha256)) {
        Remove-Item -Force -ErrorAction SilentlyContinue $Path
        throw "Download integrity check failed: $Url"
    }
}

function Get-WslDistributionVersion([string]$Name) {
    $listing = (& wsl.exe --list --verbose 2>&1 | Out-String).Replace([char]0, "")
    foreach ($line in $listing.Split([Environment]::NewLine)) {
        $normalized = $line.Trim().TrimStart([char]'*').Trim()
        if ($normalized -match ("(?i)^" + [regex]::Escape($Name) + "\s+\S+\s+([12])\s*$")) {
            return [int]$Matches[1]
        }
    }
    return $null
}

if (-not (Test-Administrator)) {
    Write-Host "Requesting administrator privileges for Windows features and WSL1 setup..."
    $arguments = @(
        "-NoLogo", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", ('"' + $PSCommandPath + '"'), "-Elevated"
    )
    $process = Start-Process powershell.exe -Verb RunAs -Wait -PassThru -ArgumentList $arguments
    exit $process.ExitCode
}

$windowsVersion = [Environment]::OSVersion.Version
if ($windowsVersion.Major -ne 10 -or $windowsVersion.Build -lt 19045) {
    throw "Windows 10 22H2 build 19045 or newer is required; found $windowsVersion."
}

New-Item -ItemType Directory -Force -Path $RuntimeDir | Out-Null
Set-InstallStage "checking_nvidia"
if (-not (Get-Command nvidia-smi.exe -ErrorAction SilentlyContinue)) {
    throw "NVIDIA driver was not found. Install driver 560.76 or newer, reboot, and rerun."
}
& nvidia-smi.exe --query-gpu=name,driver_version,memory.total --format=csv,noheader
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
    if (-not $python) { throw "Python 3.10 was installed but is not discoverable. Sign out and rerun." }
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
& $venvPython (Join-Path $PackageRoot "tools\validate_win10_gpu.py") `
    --model (Join-Path $PackageRoot "vision_guidance\best.pt") `
    --output (Join-Path $RuntimeDir "gpu_validation.json")
if ($LASTEXITCODE -ne 0) { throw "RTX 3080 CUDA, FP16, or YOLO validation failed." }

Set-InstallStage "installing_blocks"
$blocksZip = Join-Path $RuntimeDir "Blocks-1.8.1-windows.zip"
Invoke-VerifiedDownload $BlocksUrl $blocksZip $BlocksZipSize $BlocksSha256
$blocksDir = Join-Path $RuntimeDir "Blocks"
$blocksExe = Join-Path $blocksDir "WindowsNoEditor\Blocks.exe"
$blocksExeMarker = Join-Path $RuntimeDir "blocks_path.txt"
if (-not (Test-Path $blocksExe -PathType Leaf)) {
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $blocksDir
    Expand-Archive -Force -Path $blocksZip -DestinationPath $RuntimeDir
}
if (-not (Test-Path $blocksExe -PathType Leaf)) {
    throw "Blocks.exe was not found at the expected Windows release path: $blocksExe"
}
Set-Content -Encoding UTF8 -Path $blocksExeMarker -Value $blocksExe

Set-InstallStage "enabling_wsl1"
$wslFeature = Get-WindowsOptionalFeature -Online -FeatureName "Microsoft-Windows-Subsystem-Linux"
if ($wslFeature.State -ne "Enabled") {
    $result = Enable-WindowsOptionalFeature -Online -FeatureName "Microsoft-Windows-Subsystem-Linux" -All -NoRestart
    if ($result.RestartNeeded) {
        Set-InstallStage "reboot_required_for_wsl1"
        exit 3010
    }
}

$wslVersion = Get-WslDistributionVersion $Distro
if ($null -eq $wslVersion) {
    Set-InstallStage "importing_ubuntu20_wsl1"
    $rootfsArchive = Join-Path $RuntimeDir "ubuntu-focal-20230630-amd64-root.tar.gz"
    Invoke-VerifiedDownload $RootfsUrl $rootfsArchive $RootfsSize $RootfsSha256
    if (Test-Path $DistroInstall) {
        Remove-Item -Recurse -Force $DistroInstall
    }
    New-Item -ItemType Directory -Force -Path (Split-Path $DistroInstall -Parent) | Out-Null
    & wsl.exe --import $Distro $DistroInstall $rootfsArchive --version 1
    if ($LASTEXITCODE -ne 0) {
        & wsl.exe --unregister $Distro 2>$null
        throw "Could not import the dedicated Ubuntu 20.04 WSL1 distribution."
    }
    $wslVersion = Get-WslDistributionVersion $Distro
}
if ($wslVersion -ne 1) {
    throw "$Distro must be WSL1. Existing distributions are never converted automatically."
}

Set-InstallStage "installing_px4_v1.11.3"
$wslRoot = (& wsl.exe -d $Distro -u root -- wslpath -a $PackageRoot).Trim()
if ($LASTEXITCODE -ne 0 -or -not $wslRoot) { throw "Could not translate the package path into WSL1." }
& wsl.exe -d $Distro -u root -- bash "$wslRoot/tools/setup_px4_wsl.sh"
if ($LASTEXITCODE -ne 0) { throw "PX4 v1.11.3 setup in WSL1 failed." }

Set-InstallStage "validating_package"
& $venvPython (Join-Path $PackageRoot "tools\check_package.py") --runtime
if ($LASTEXITCODE -ne 0) { throw "Package validation failed." }

Set-InstallStage "complete"
Write-Host ""
Write-Host "Installation complete. Run: run_experiments.bat smoke fast"
Write-Host "Then validate PX4 SITL with: run_experiments.bat smoke sitl"
