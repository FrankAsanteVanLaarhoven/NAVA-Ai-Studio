# 📦 Creating Your First NAVΛ Studio Release

Your landing page is trying to download from GitHub releases that don't exist yet. Here's how to fix that!

---

## 🎯 Two Options

### Option 1: Create Real Releases (Production Ready)

#### Step 1: Build Installers

```bash
# Navigate to your project
cd "/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE"

# Build for your current platform (macOS)
npm run tauri:build

# Or use the automated script
./scripts/build.sh
```

**This creates:**
- macOS: `src-tauri/target/release/bundle/dmg/NAVΛ Studio.dmg`
- Windows: `src-tauri/target/release/bundle/msi/NAVΛ Studio.msi` (if on Windows)
- Linux: `src-tauri/target/release/bundle/appimage/NAVΛ Studio.AppImage` (if on Linux)

**Build time:** 15-25 minutes first time, 2-5 minutes after

---

#### Step 2: Commit Your Code

```bash
# Add all your new files
git add .

# Commit
git commit -m "Add landing page and SDK documentation"

# Push to GitHub
git push origin main
```

---

#### Step 3: Create GitHub Release

**Option A: Using GitHub CLI (Easiest)**

```bash
# Install GitHub CLI if you don't have it
brew install gh

# Login
gh auth login

# Create release and upload installers
gh release create v1.0.0 \
  "src-tauri/target/release/bundle/dmg/NAVΛ Studio.dmg" \
  --title "NAVΛ Studio v1.0.0 - First Release" \
  --notes "🎉 First stable release of NAVΛ Studio IDE

## What's New
- ⋋ Native Van Laarhoven Navigation Calculus support
- 🎨 3D navigation visualization
- ⚡ Multi-target compilation
- 🚀 Desktop apps for macOS, Windows, Linux
- 📱 Beautiful landing page
- 🛠️ Complete SDK and documentation

## Download
- macOS: Universal binary (Intel + Apple Silicon)
- Windows: MSI installer
- Linux: AppImage, DEB, RPM

## Getting Started
See GETTING_STARTED.md for installation and usage guide.

Built with ❤️ by Frank Van Laarhoven"
```

**Option B: Using GitHub Web Interface**

1. Go to: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases
2. Click **"Draft a new release"**
3. Create tag: `v1.0.0`
4. Title: `NAVΛ Studio v1.0.0`
5. Upload your built installers (drag & drop)
6. Write release notes
7. Click **"Publish release"**

---

#### Step 4: Test Download Links

After creating the release, test your download links:
- macOS: `https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/NAVΛ-Studio.dmg`
- Update landing page if filenames differ

---

### Option 2: Quick Testing Fix (Development)

If you just want to test the landing page without releases:

#### Update Landing Page to Show "Coming Soon"

I've already updated the landing page to detect if releases exist and show appropriate messages.

**What happens now:**
- ✅ Detects no releases → Shows "Coming Soon" dialog
- ✅ Offers to visit GitHub repo
- ✅ Shows installation command preview
- ✅ No more 404 errors!

**Test it:**
```bash
open download.html
# Click any download button
```

---

## 🔧 Build Instructions by Platform

### macOS (You are here!)

```bash
# Install dependencies (if not done)
npm install

# Build
npm run tauri:build -- --target universal-apple-darwin

# Output: src-tauri/target/release/bundle/dmg/
```

### Windows

```powershell
# Install dependencies
npm install

# Build
npm run tauri:build

# Output: src-tauri/target/release/bundle/msi/
```

### Linux

```bash
# Install dependencies
sudo apt-get install libwebkit2gtk-4.0-dev build-essential curl wget libssl-dev libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev
npm install

# Build
npm run tauri:build -- --bundles deb,appimage,rpm

# Output: src-tauri/target/release/bundle/
```

---

## 📊 Cross-Platform Release Strategy

### If You Only Have macOS

**Option 1:** Build macOS only initially
```bash
npm run tauri:build
gh release create v1.0.0-macos "bundle/dmg/*.dmg"
```

**Option 2:** Use GitHub Actions (recommended)

Create `.github/workflows/release.yml`:

```yaml
name: Release

on:
  push:
    tags:
      - 'v*'

jobs:
  release:
    strategy:
      matrix:
        platform: [macos-latest, ubuntu-22.04, windows-latest]
    runs-on: ${{ matrix.platform }}
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
      - uses: dtolnay/rust-toolchain@stable
      - name: Install dependencies (Ubuntu)
        if: matrix.platform == 'ubuntu-22.04'
        run: |
          sudo apt-get update
          sudo apt-get install -y libwebkit2gtk-4.0-dev build-essential curl wget libssl-dev libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev
      - run: npm ci
      - run: npm run tauri:build
      - uses: softprops/action-gh-release@v1
        with:
          files: src-tauri/target/release/bundle/**/*
```

Then just push a tag and GitHub will build for all platforms!

---

## 🎯 Quick Start (Recommended Path)

### Today (Testing)

1. ✅ Use updated landing page (already done)
2. ✅ Landing page shows "Coming Soon"
3. ✅ No more 404 errors

### This Week (First Release)

1. **Build macOS version:**
   ```bash
   npm run tauri:build
   ```

2. **Create release:**
   ```bash
   gh release create v1.0.0 "src-tauri/target/release/bundle/dmg/*.dmg"
   ```

3. **Test download** - Landing page will now work!

### Next Week (Full Multi-Platform)

1. Set up GitHub Actions for automated builds
2. Create releases for all platforms automatically
3. Full production deployment

---

## 🐛 Troubleshooting

### "Command not found: gh"
```bash
brew install gh
gh auth login
```

### "Build failed"
```bash
# Clean and rebuild
rm -rf node_modules src-tauri/target
npm install
npm run tauri:build
```

### "No .dmg found"
```bash
# Check build output
ls -la src-tauri/target/release/bundle/dmg/
```

---

## ✅ Verification Checklist

Before creating a release:

- [ ] App builds successfully
- [ ] Installer works on fresh machine
- [ ] All features functional
- [ ] Documentation complete
- [ ] Version numbers updated
- [ ] Release notes written
- [ ] GitHub repository ready

---

## 📱 Current Status

✅ **Landing page**: Updated with fallback  
✅ **Documentation**: Complete  
✅ **Build scripts**: Ready  
⏳ **Installers**: Need to build  
⏳ **GitHub release**: Need to create  

---

## 🎉 Next Steps

1. **Build your first installer** (20 min)
2. **Test it locally** (5 min)
3. **Create GitHub release** (5 min)
4. **Test download from landing page** (2 min)
5. **Share with the world!** 🚀

---

**Need help?** Let me know which option you prefer:
1. Build now (I'll guide you)
2. Set up GitHub Actions (automated)
3. Keep testing mode for now

---

Built with ❤️ for NAVΛ Studio

