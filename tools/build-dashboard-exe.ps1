[CmdletBinding()]
param(
    [string]$Python = ".\\.venv\\Scripts\\python.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$pythonPath = Join-Path $repoRoot $Python
$scriptPath = Join-Path $PSScriptRoot "scv2_dashboard.py"
$outputPath = Join-Path $repoRoot "SCV2 Dashboard.exe"
$workPath = Join-Path $PSScriptRoot "dashboard-pyinstaller-work"
$distPath = Join-Path $PSScriptRoot "dashboard-pyinstaller-dist"
$specPath = Join-Path $PSScriptRoot "dashboard-pyinstaller-spec"

if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) {
    throw "Dashboard Python environment was not found: $pythonPath. Create .venv and install tools/requirements-dashboard.txt first."
}

$pyInstallerInstalled = (& $pythonPath -c "import importlib.util; print(int(importlib.util.find_spec('PyInstaller') is not None))").Trim()
if ($LASTEXITCODE -ne 0) {
    throw "Could not check whether PyInstaller is installed."
}
if ($pyInstallerInstalled -ne "1") {
    & $pythonPath -m pip install pyinstaller
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller installation failed."
    }
}

Remove-Item -LiteralPath $workPath -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $distPath -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $specPath -Recurse -Force -ErrorAction SilentlyContinue

& $pythonPath -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name "SCV2 Dashboard" `
    --workpath $workPath `
    --distpath $distPath `
    --specpath $specPath `
    $scriptPath
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller failed to build the dashboard."
}

Copy-Item -LiteralPath (Join-Path $distPath "SCV2 Dashboard.exe") -Destination $outputPath -Force
& $outputPath --self-test
if ($LASTEXITCODE -ne 0) {
    throw "The packaged dashboard failed its self-test."
}

Write-Host "Portable dashboard created: $outputPath"
