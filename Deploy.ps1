param([switch]$TeachOnly, [string]$Python)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if ($PSVersionTable.PSEdition -ne 'Desktop' -or $PSVersionTable.PSVersion -lt [version]'5.1') {
    throw 'Run Deploy.cmd with Windows PowerShell 5.1.'
}
Add-Type -AssemblyName System.Windows.Forms,System.Drawing
foreach ($arm in 'left','right') {
    Write-Host "Testing the $arm arm in memory"
    & .\BimanualArmTeach.ps1 -Arm $arm -SelfTest
}
if ($TeachOnly) { Write-Host 'DEPLOY OK (teach console)'; exit 0 }
$venv = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venv)) {
    $launcherArgs = @()
    if (-not $Python) {
        if (Get-Command py.exe -ErrorAction SilentlyContinue) { $Python = 'py.exe'; $launcherArgs = @('-3') }
        else { $Python = 'python.exe' }
    }
    & $Python @launcherArgs -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)"
    if ($LASTEXITCODE -ne 0) { throw 'Install Python 3.10+ or use Deploy.cmd -Python C:\path\to\python.exe.' }
    & $Python @launcherArgs -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Could not create .venv.' }
}
& $venv -m pip --disable-pip-version-check install --no-index --find-links vendor --require-hashes -r requirements.txt
if ($LASTEXITCODE -ne 0) { throw 'Local pySerial installation failed.' }
$env:PYTHONUTF8 = '1'
& $venv -m unittest discover -s demo -p 'verify_*.py'
if ($LASTEXITCODE -ne 0) { throw 'Simulated demo checks failed.' }
Write-Host 'DEPLOY OK (teach console and gripper demo)'
