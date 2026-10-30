param([switch]$Export, [string]$CondaRoot)
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
Set-Location -LiteralPath $PSScriptRoot

function Run-Checked($Program, $Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Command failed: $Program (exit $LASTEXITCODE)" }
}

function Download-File($Url, $Destination) {
    # Never expose an interrupted download as a finished installer.
    $partial = "$Destination.part"
    $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
    for ($attempt = 1; $attempt -le 3; $attempt++) {
        try {
            if ($curl) {
                Run-Checked $curl.Source @('--fail', '--location', '--silent', '--show-error',
                    '--connect-timeout', '30', '--max-time', '900', '--output', $partial, $Url)
            } else {
                Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $partial -TimeoutSec 900
            }
            Move-Item -LiteralPath $partial -Destination $Destination -Force
            return
        } catch {
            if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
            if ($attempt -eq 3) { throw }
            Write-Warning "Download failed (attempt $attempt/3): $($_.Exception.Message). Retrying..."
            Start-Sleep -Seconds 3
        }
    }
}

function Get-MiniforgeInstaller($Installer) {
    # Pin the release and published SHA256 so mirror updates cannot mix versions.
    # https://github.com/conda-forge/miniforge/releases/tag/26.9.4-1
    $version = '26.9.4-1'
    $expected = '8cb96eed35effdd5b2d0860c5e85eed2224e72e71dd45a3fd6b1afe3d70585bb'
    $fileName = "Miniforge3-$version-Windows-x86_64.exe"
    if (Test-Path -LiteralPath $Installer) {
        if ((Get-FileHash -LiteralPath $Installer -Algorithm SHA256).Hash -eq $expected) {
            Write-Host 'Using verified cached Miniforge installer.'
            return
        }
        Write-Warning 'Cached installer is incomplete or a different version; downloading again.'
    }
    $urls = @(
        "https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/$fileName",
        "https://github.com/conda-forge/miniforge/releases/download/$version/$fileName"
    )
    foreach ($url in $urls) {
        try {
            Download-File $url $Installer
            if ((Get-FileHash -LiteralPath $Installer -Algorithm SHA256).Hash -ne $expected) {
                throw 'Installer SHA256 checksum mismatch'
            }
            return
        } catch {
            Write-Warning "Cannot use ${url}: $($_.Exception.Message)"
        }
    }
    throw 'Unable to download a verified Miniforge installer. Check network/proxy access and rerun setup.cmd.'
}

function Install-Miniforge($Installer, $Destination) {
    # NSIS requires /D last and unquoted.
    $arguments = "/S /InstallationType=JustMe /RegisterPython=0 /AddToPath=0 /NoShortcuts=1 /D=$Destination"
    $process = Start-Process -FilePath $Installer -ArgumentList $arguments -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "Miniforge installation failed (exit $($process.ExitCode)), destination: $Destination"
    }
}

$setupLock = $null
try {
    New-Item -ItemType Directory -Force '.setup' | Out-Null
    try {
        $setupLock = [IO.File]::Open((Join-Path $PSScriptRoot '.setup\setup.lock'), [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    } catch [IO.IOException] {
        throw 'Another setup is running for this project. Wait for it to finish before retrying.'
    }
    $defaultRoot = Join-Path $PSScriptRoot '.conda'
    $rootFile = Join-Path $PSScriptRoot '.setup\conda-root.txt'
    if (-not $CondaRoot) {
        $CondaRoot = if (Test-Path -LiteralPath $rootFile) {
            (Get-Content -LiteralPath $rootFile -Raw).Trim()
        } else { $defaultRoot }
    }
    if (-not $CondaRoot) { throw 'Conda installation path is empty.' }
    $CondaRoot = [IO.Path]::GetFullPath($CondaRoot)
    if ($CondaRoot -match '[^\x21-\x7E]' -and -not (Test-Path -LiteralPath "$CondaRoot\Scripts\conda.exe")) {
        # Keep environments separate per project, without Chinese characters/spaces.
        $hasher = [Security.Cryptography.SHA256]::Create()
        try {
            $projectId = [BitConverter]::ToString($hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($PSScriptRoot.ToLowerInvariant()))).Replace('-', '').Substring(0, 12)
        } finally { $hasher.Dispose() }
        $CondaRoot = Join-Path $env:LOCALAPPDATA "PiperCapture\$projectId\conda"
        if ($CondaRoot -match '[^\x21-\x7E]') {
            throw 'Use an ASCII path without spaces: setup.cmd -CondaRoot C:\PiperCaptureEnv'
        }
        Write-Host "Using ASCII Conda path: $CondaRoot"
    }
    $condaExe = Join-Path $CondaRoot 'Scripts\conda.exe'
    if (-not (Test-Path -LiteralPath $condaExe)) {
        Write-Host 'Installing local Conda (Miniforge)...'
        $installer = Join-Path $PSScriptRoot '.setup\Miniforge3-Windows-x86_64.exe'
        Get-MiniforgeInstaller $installer
        New-Item -ItemType Directory -Force (Split-Path -Parent $CondaRoot) | Out-Null
        Install-Miniforge $installer $CondaRoot
        if (-not (Test-Path -LiteralPath $condaExe)) { throw 'Conda installation did not finish successfully' }
    }
    Run-Checked $condaExe @('--version')
    if ($CondaRoot -ne $defaultRoot) {
        # Batch launchers read this ASCII path with set /p.
        if ($CondaRoot -match '[^\x20-\x7E]') { throw 'The external Conda path must contain only ASCII characters.' }
        Set-Content -LiteralPath $rootFile -Value $CondaRoot -Encoding ASCII
    } elseif (Test-Path -LiteralPath $rootFile) {
        Remove-Item -LiteralPath $rootFile
    }

    $envName = if ($Export) { 'export' } else { 'capture' }
    $envFile = if ($Export) { 'environment-export.yml' } else { 'environment.yml' }
    $envPath = Join-Path $CondaRoot "envs\$envName"
    $operation = if (Test-Path "$envPath\python.exe") { 'update' } else { 'create' }
    $envArgs = @('env', $operation, '--prefix', $envPath, '--file', $envFile)
    if ($operation -eq 'create') { $envArgs += '--yes' }
    Run-Checked $condaExe $envArgs
    if (-not $Export) {
        Run-Checked $condaExe @('run', '--prefix', $envPath, '--no-capture-output', 'python', '-m', 'pip', 'install', './vendor/python-can-agx-cando-master', './vendor/pyAgxArm-master')
    }
    if (-not (Test-Path 'config.json')) { Copy-Item 'config.example.json' 'config.json' }
    Write-Host "Ready: $envName"
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
} finally {
    if ($setupLock) { $setupLock.Dispose() }
}
