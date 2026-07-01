<#
Install OpenAI Python SDK into a local virtualenv (.venv) and print next steps.
Usage (PowerShell):
  pwsh -ExecutionPolicy Bypass -File .\install_openai_python.ps1
#>

Write-Host "== Checking Python =="
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { Write-Error "python not found in PATH. Install Python 3.8+ and retry."; exit 2 }

Write-Host "Using: $(python --version)"

if (-not (Test-Path -Path ".venv")) {
    Write-Host "Creating virtual environment .venv..."
    python -m venv .venv
    if ($LASTEXITCODE -ne 0) { Write-Error "Failed to create venv."; exit $LASTEXITCODE }
}

$venvPy = Join-Path -Path ".venv" -ChildPath "Scripts\python.exe"
if (-not (Test-Path $venvPy)) { Write-Error "Virtualenv python not found at $venvPy"; exit 3 }

Write-Host "Upgrading pip in venv..."
& $venvPy -m pip install --upgrade pip

Write-Host "Installing openai package..."
& $venvPy -m pip install openai
if ($LASTEXITCODE -ne 0) { Write-Error "pip install openai failed."; exit $LASTEXITCODE }

Write-Host "\nDone. To use the SDK:" 
Write-Host "  PowerShell: .\.venv\Scripts\Activate.ps1; python -c \"import openai; print('openai', openai)\""
Write-Host "  Or run scripts with: .\.venv\Scripts\python your_script.py"
Write-Host "Remember to set your OPENAI_API_KEY environment variable before using the SDK. Example (PowerShell):"
Write-Host "  $env:OPENAI_API_KEY='sk-xxx'"
