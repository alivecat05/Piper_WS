param([switch]$Export)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

function Run-Checked($Program, $Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Command failed: $Program (exit $LASTEXITCODE)" }
}

try {
    New-Item -ItemType Directory -Force '.setup' | Out-Null
    $condaExe = Join-Path $PSScriptRoot '.conda\Scripts\conda.exe'
    if (-not (Test-Path $condaExe)) {
        Write-Host 'Installing local Conda (Miniforge)...'
        $baseUrl = 'https://github.com/conda-forge/miniforge/releases/latest/download'
        $installer = Join-Path $PSScriptRoot '.setup\Miniforge3-Windows-x86_64.exe'
        Invoke-WebRequest "$baseUrl/Miniforge3-Windows-x86_64.exe" -OutFile $installer
        Invoke-WebRequest "$baseUrl/Miniforge3-Windows-x86_64.exe.sha256" -OutFile "$installer.sha256"
        $expected = ((Get-Content "$installer.sha256" -Raw).Trim() -split '\s+')[0]
        if ((Get-FileHash $installer -Algorithm SHA256).Hash -ne $expected) { throw 'Installer checksum mismatch' }
        Run-Checked $installer @('/S', '/InstallationType=JustMe', '/RegisterPython=0', '/AddToPath=0', "/D=$PSScriptRoot\.conda")
        if (-not (Test-Path $condaExe)) { throw 'Conda installation did not finish successfully' }
    }

    $envName = if ($Export) { 'export' } else { 'teleop' }
    $envFile = if ($Export) { 'environment-export.yml' } else { 'environment.yml' }
    $envPath = Join-Path $PSScriptRoot ".conda\envs\$envName"
    $operation = if (Test-Path "$envPath\python.exe") { 'update' } else { 'create' }
    $envArgs = @('env', $operation, '--prefix', $envPath, '--file', $envFile)
    if ($operation -eq 'create') { $envArgs += '--yes' }
    Run-Checked $condaExe $envArgs
    if (-not $Export) {
        Run-Checked $condaExe @('run', '--prefix', $envPath, '--no-capture-output', 'python', '-m', 'pip', 'install', './vendor/python-can-agx-cando-master', './vendor/pyAgxArm-master')
        if (-not (Test-Path '.setup\platform-tools\adb.exe')) {
            Invoke-WebRequest 'https://dl.google.com/android/repository/platform-tools-latest-windows.zip' -OutFile '.setup\platform-tools.zip'
            Expand-Archive -LiteralPath '.setup\platform-tools.zip' -DestinationPath '.setup' -Force
        }
    }
    if (-not (Test-Path 'config.json')) { Copy-Item 'config.example.json' 'config.json' }
    Write-Host "Ready: $envName"
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
