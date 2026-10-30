Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot

$targets = @(
    (Join-Path $repoRoot "build"),
    (Join-Path $repoRoot "dist"),
    (Join-Path $repoRoot ".pytest_cache"),
    (Join-Path $repoRoot ".coverage")
)

$targets += Get-ChildItem -Path $repoRoot -Directory -Filter "*.egg-info" -Force -ErrorAction SilentlyContinue |
    Select-Object -ExpandProperty FullName

$targets += Get-ChildItem -Path $repoRoot -Recurse -Directory -Filter "__pycache__" -Force -ErrorAction SilentlyContinue |
    Select-Object -ExpandProperty FullName

$targets += Get-ChildItem -Path $repoRoot -Recurse -File -Include "*.pyc", "*.pyo", "*.pyd" -Force -ErrorAction SilentlyContinue |
    Select-Object -ExpandProperty FullName

$failed = @()

$targets |
    Sort-Object -Unique |
    Sort-Object { $_.Length } -Descending |
    Where-Object { $_ -and (Test-Path -LiteralPath $_) } |
    ForEach-Object {
        $target = $_
        try {
            Write-Host "Removing $target"
            Remove-Item -LiteralPath $target -Recurse -Force
        }
        catch {
            $failed += $target
            Write-Warning "Failed to remove $target : $($_.Exception.Message)"
        }
    }

Write-Host "Build cache cleanup complete."

if ($failed.Count -gt 0) {
    Write-Warning "Some paths could not be removed. Re-run in an elevated shell if needed."
}
