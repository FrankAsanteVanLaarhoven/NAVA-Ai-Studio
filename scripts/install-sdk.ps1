# NAVΛ Studio SDK - Windows Installation Script
# PowerShell script for automated Windows installation

param(
    [switch]$BuildFromSource = $false
)

$ErrorActionPreference = "Stop"

# Colors (using Write-Host)
function Write-Success { param($msg) Write-Host "✓ $msg" -ForegroundColor Green }
function Write-Info { param($msg) Write-Host "→ $msg" -ForegroundColor Cyan }
function Write-Warning { param($msg) Write-Host "⚠ $msg" -ForegroundColor Yellow }
function Write-Error-Custom { param($msg) Write-Host "✗ $msg" -ForegroundColor Red }

# Banner
Write-Host ""
Write-Host "╔═══════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║   NAVΛ STUDIO IDE - Windows SDK Installer   ║" -ForegroundColor Cyan
Write-Host "║   Van Laarhoven Navigation Calculus          ║" -ForegroundColor Cyan
Write-Host "╚═══════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Check if running as Administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Warning "Not running as Administrator. Some features may require elevation."
}

Write-Info "Detected OS: Windows"
Write-Host ""

if ($BuildFromSource) {
    Write-Info "Building from source..."
    Write-Host ""
    
    # Check prerequisites
    Write-Info "Checking prerequisites..."
    
    # Check Node.js
    try {
        $nodeVersion = & node --version 2>$null
        Write-Success "Node.js $nodeVersion"
    } catch {
        Write-Error-Custom "Node.js not found"
        Write-Host "Please install Node.js 18+ from: https://nodejs.org/" -ForegroundColor Yellow
        exit 1
    }
    
    # Check npm
    try {
        $npmVersion = & npm --version 2>$null
        Write-Success "npm $npmVersion"
    } catch {
        Write-Error-Custom "npm not found"
        exit 1
    }
    
    # Check Rust
    try {
        $cargoVersion = & cargo --version 2>$null
        Write-Success "Rust $cargoVersion"
    } catch {
        Write-Warning "Rust not found. Installing..."
        
        # Download and run rustup
        $rustupUrl = "https://win.rustup.rs/x86_64"
        $rustupPath = "$env:TEMP\rustup-init.exe"
        
        Write-Info "Downloading Rust installer..."
        Invoke-WebRequest -Uri $rustupUrl -OutFile $rustupPath
        
        Write-Info "Installing Rust..."
        Start-Process -FilePath $rustupPath -ArgumentList "-y" -Wait
        
        # Refresh environment
        $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")
        
        Write-Success "Rust installed"
    }
    
    # Check Visual Studio Build Tools
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        Write-Warning "Visual Studio Build Tools not detected"
        Write-Host "Installing Microsoft Visual C++ Build Tools..." -ForegroundColor Yellow
        Write-Host "This may take several minutes..." -ForegroundColor Yellow
        
        try {
            winget install Microsoft.VisualStudio.2022.BuildTools --silent --accept-package-agreements --accept-source-agreements
            Write-Success "Build Tools installed"
        } catch {
            Write-Warning "Could not install Build Tools automatically"
            Write-Host "Please install manually from: https://visualstudio.microsoft.com/downloads/" -ForegroundColor Yellow
        }
    }
    
    # Install dependencies
    Write-Host ""
    Write-Info "Installing project dependencies..."
    npm install
    
    # Build
    Write-Host ""
    Write-Info "Building NAVΛ Studio..."
    Write-Host "This may take 15-20 minutes on first build..." -ForegroundColor Yellow
    
    npm run build
    npm run tauri:build
    
    Write-Host ""
    Write-Success "Build complete!"
    Write-Host ""
    Write-Host "Installer location:" -ForegroundColor Cyan
    Write-Host "  src-tauri\target\release\bundle\msi\" -ForegroundColor Green
    Write-Host ""
    
    # Ask if user wants to install
    $install = Read-Host "Install now? (Y/n)"
    if ($install -ne "n" -and $install -ne "N") {
        $msiPath = Get-ChildItem -Path "src-tauri\target\release\bundle\msi\*.msi" | Select-Object -First 1
        if ($msiPath) {
            Write-Info "Installing..."
            Start-Process msiexec.exe -ArgumentList "/i `"$($msiPath.FullName)`" /qb" -Wait
            Write-Success "Installation complete!"
        }
    }
    
} else {
    # Download pre-built
    Write-Info "Downloading pre-built installer..."
    Write-Host ""
    
    $githubRepo = "FrankAsanteVanLaarhoven/NAVA-Ai-Studio"
    $installerName = "NAVA-Studio_1.0.0_x64_en-US.msi"
    $downloadUrl = "https://github.com/$githubRepo/releases/latest/download/$installerName"
    $installerPath = "$env:TEMP\$installerName"
    
    try {
        Write-Info "Downloading from: $downloadUrl"
        Invoke-WebRequest -Uri $downloadUrl -OutFile $installerPath -UseBasicParsing
        Write-Success "Downloaded successfully"
        
        # Verify download
        if (Test-Path $installerPath) {
            $fileSize = (Get-Item $installerPath).Length / 1MB
            Write-Info "Downloaded: $([math]::Round($fileSize, 2)) MB"
            
            Write-Host ""
            Write-Info "Installing NAVΛ Studio..."
            
            # Run MSI installer
            Start-Process msiexec.exe -ArgumentList "/i `"$installerPath`" /qb" -Wait
            
            Write-Host ""
            Write-Success "Installation complete!"
            
            # Clean up
            Remove-Item $installerPath -Force
            
        } else {
            Write-Error-Custom "Download failed"
            exit 1
        }
        
    } catch {
        Write-Error-Custom "Failed to download installer"
        Write-Host "Error: $_" -ForegroundColor Red
        Write-Host ""
        Write-Host "Please download manually from:" -ForegroundColor Yellow
        Write-Host "  https://github.com/$githubRepo/releases" -ForegroundColor Cyan
        exit 1
    }
}

Write-Host ""
Write-Host "╔════════════════════════════════════════════╗" -ForegroundColor Green
Write-Host "║   ✓ NAVΛ Studio Installation Complete!    ║" -ForegroundColor Green
Write-Host "╚════════════════════════════════════════════╝" -ForegroundColor Green
Write-Host ""
Write-Host "Next Steps:" -ForegroundColor Cyan
Write-Host "  1. Launch NAVΛ Studio from Start Menu" -ForegroundColor White
Write-Host "  2. Press Ctrl+Shift+P for Command Palette" -ForegroundColor White
Write-Host "  3. Try 'Help: Getting Started'" -ForegroundColor White
Write-Host "  4. Explore example projects" -ForegroundColor White
Write-Host ""
Write-Host "Documentation: docs\GETTING_STARTED.md" -ForegroundColor Cyan
Write-Host "Support: https://github.com/$githubRepo/issues" -ForegroundColor Cyan
Write-Host ""
Write-Host "Happy Navigation Calculus Programming! 🚀⋋" -ForegroundColor Green
Write-Host ""

# Open Start Menu (optional)
$openStart = Read-Host "Open Start Menu to launch? (Y/n)"
if ($openStart -ne "n" -and $openStart -ne "N") {
    Start-Process "shell:AppsFolder"
}

