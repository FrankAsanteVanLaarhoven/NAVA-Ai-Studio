# NAVΛ Studio Build Scripts

This directory contains all build, deployment, and installation scripts for NAVΛ Studio IDE.

---

## 📜 Available Scripts

### 🔨 Build Scripts

#### `build.sh`
**Purpose**: Build the complete desktop application  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/build.sh
```
**Output**: Installers in `src-tauri/target/release/bundle/`  
**Time**: 15-25 minutes (first build), 2-5 minutes (incremental)

---

#### `package.sh`
**Purpose**: Package the application for distribution  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/package.sh
```
**Output**: `dist/navlambda-studio-<date>.tar.gz`  
**Includes**: Installers, docs, examples

---

#### `create-sdk-package.sh`
**Purpose**: Create complete SDK distribution package  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/create-sdk-package.sh

# Skip build and use existing artifacts
./scripts/create-sdk-package.sh --no-build
```
**Output**: Complete SDK bundle with:
- ✅ All platform installers
- ✅ Complete documentation
- ✅ Example projects
- ✅ Installation scripts
- ✅ Checksums

**Time**: 20-30 minutes (with build), 2-3 minutes (no build)

---

### 📥 Installation Scripts

#### `install-sdk.sh`
**Purpose**: Automated installation for end users  
**Platform**: macOS, Linux  
**Usage**:
```bash
# Download and run
curl -fsSL https://raw.githubusercontent.com/.../install-sdk.sh | bash

# Or download first
wget https://raw.githubusercontent.com/.../install-sdk.sh
chmod +x install-sdk.sh
./install-sdk.sh
```
**Features**:
- ✅ Auto-detects OS and architecture
- ✅ Downloads pre-built installer OR builds from source
- ✅ Installs dependencies automatically
- ✅ User-friendly progress indicators
- ✅ Error handling and recovery

**Options**:
1. Download pre-built (recommended for users)
2. Build from source (for developers)

---

#### `install-sdk.ps1`
**Purpose**: Automated installation for Windows  
**Platform**: Windows  
**Usage**:
```powershell
# Download and run
irm https://raw.githubusercontent.com/.../install-sdk.ps1 | iex

# Or download first
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/.../install-sdk.ps1" -OutFile "install-sdk.ps1"
.\install-sdk.ps1

# Build from source
.\install-sdk.ps1 -BuildFromSource
```
**Features**:
- ✅ Admin privilege detection
- ✅ Automatic Rust installation
- ✅ Visual Studio Build Tools check
- ✅ Progress indicators
- ✅ Error handling

**Parameters**:
- `-BuildFromSource`: Build instead of downloading

---

### 🚢 Deployment Scripts

#### `deploy.sh`
**Purpose**: Deploy to cloud services  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/deploy.sh
```
**Supports**:
- Docker containers
- Kubernetes
- AWS ECS
- Google Cloud Run
- Azure Container Instances

---

### 🧪 Testing Scripts

#### `test.sh`
**Purpose**: Run comprehensive test suite  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/test.sh
```
**Runs**:
- Unit tests
- Integration tests
- E2E tests
- Linting
- Type checking

---

### 🛠️ Development Scripts

#### `setup-dev-environment.sh`
**Purpose**: Set up development environment  
**Platform**: macOS, Linux  
**Usage**:
```bash
./scripts/setup-dev-environment.sh
```
**Installs**:
- Node.js dependencies
- Rust toolchain
- Platform-specific dependencies
- Development tools
- Pre-commit hooks

---

## 🎯 Common Workflows

### For End Users

**Install NAVΛ Studio**:
```bash
# macOS/Linux
./scripts/install-sdk.sh

# Windows
.\scripts\install-sdk.ps1
```

---

### For Developers

**Set up development environment**:
```bash
./scripts/setup-dev-environment.sh
npm install
```

**Development mode** (with hot reload):
```bash
npm run tauri:dev
```

**Build production**:
```bash
./scripts/build.sh
```

**Run tests**:
```bash
./scripts/test.sh
```

---

### For Distributors

**Create complete SDK package**:
```bash
# Build and package everything
./scripts/create-sdk-package.sh

# Output: dist/navlambda-studio-sdk-v1.0.0-<timestamp>.tar.gz
```

**Deploy to GitHub Releases**:
```bash
# After building
gh release create v1.0.0 \
  src-tauri/target/release/bundle/**/*.{dmg,msi,AppImage,deb,rpm} \
  --title "NAVΛ Studio v1.0.0" \
  --notes "Release notes"
```

**Deploy to cloud**:
```bash
./scripts/deploy.sh
```

---

## 📋 Script Dependencies

### All Scripts
- Bash 4.0+ (macOS/Linux) or PowerShell 5.1+ (Windows)
- Git

### Build Scripts
- Node.js 18+
- npm 9+
- Rust 1.75+
- Platform-specific tools (see BUILD_INSTRUCTIONS.md)

### Deployment Scripts
- Docker (for container deployments)
- kubectl (for Kubernetes)
- Cloud CLI tools (aws-cli, gcloud, azure-cli)

---

## 🔧 Configuration

Most scripts read configuration from:
- `package.json` - Version, name, metadata
- `tauri.conf.json` - Tauri configuration
- `Cargo.toml` - Rust dependencies
- Environment variables (optional)

### Environment Variables

```bash
# Build optimization
export CARGO_BUILD_JOBS=8
export RUSTC_WRAPPER=sccache

# Skip certain steps
export SKIP_WASM_BUILD=1
export SKIP_TESTS=1

# Custom paths
export TAURI_PRIVATE_KEY=/path/to/key
export TAURI_KEY_PASSWORD=password
```

---

## 🐛 Troubleshooting

### Script won't execute
```bash
# Make executable
chmod +x scripts/*.sh
```

### Build fails
```bash
# Clean and rebuild
rm -rf node_modules src-tauri/target
npm install
./scripts/build.sh
```

### Missing dependencies
```bash
# Run setup script
./scripts/setup-dev-environment.sh
```

### Permission errors (Linux)
```bash
# Run with sudo where needed
sudo ./scripts/install-sdk.sh
```

---

## 📊 Script Performance

| Script | Time (First Run) | Time (Incremental) |
|--------|------------------|-------------------|
| `build.sh` | 15-25 min | 2-5 min |
| `package.sh` | 20-30 min | 3-5 min |
| `create-sdk-package.sh` | 25-35 min | 3-7 min |
| `install-sdk.sh` (download) | 30 sec - 2 min | - |
| `install-sdk.sh` (build) | 20-30 min | - |
| `deploy.sh` | 5-10 min | 2-5 min |
| `test.sh` | 2-5 min | 1-2 min |

---

## 🎓 Learning Path

1. **New Users**: Start with `install-sdk.sh` or `install-sdk.ps1`
2. **Developers**: Run `setup-dev-environment.sh`, then `build.sh`
3. **Distributors**: Use `create-sdk-package.sh`
4. **DevOps**: Explore `deploy.sh` and CI/CD integration

---

## 📝 Contributing

When adding new scripts:
1. Follow existing naming conventions
2. Add error handling
3. Include progress indicators
4. Document in this README
5. Test on all target platforms
6. Add to CI/CD pipeline

---

## 🔗 Related Documentation

- **BUILD_INSTRUCTIONS.md** - Detailed build guide
- **DESKTOP_SDK_INSTALLATION.md** - Installation guide
- **SDK_QUICK_START.md** - Quick start guide
- **docs/deployment-guide.md** - Deployment documentation

---

## 💡 Tips

### Speed up builds
```bash
# Install sccache
cargo install sccache
export RUSTC_WRAPPER=sccache

# Parallel builds
export CARGO_BUILD_JOBS=$(nproc)
```

### Debug builds
```bash
# Add debug output
set -x  # Enable debug mode
./scripts/build.sh
set +x  # Disable debug mode
```

### Dry run
Most scripts support dry-run mode (check script for details).

---

## 🌟 Advanced Usage

### Custom Build Targets

```bash
# macOS Universal
npm run tauri:build -- --target universal-apple-darwin

# Specific Linux distro
npm run tauri:build -- --bundles deb

# Multiple targets
npm run tauri:build -- --bundles deb,appimage,rpm
```

### CI/CD Integration

See `.github/workflows/` for GitHub Actions examples.

### Cross-compilation

Check `docs/compilation-targets.md` for cross-compilation setup.

---

**Questions?** Check the main documentation or open an issue on GitHub.

**NAVΛ Studio** - Building the Future of Navigation Calculus 🚀⋋

