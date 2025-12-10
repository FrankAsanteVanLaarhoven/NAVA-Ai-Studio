# NAVΛ SDK Installer Build Guide

## Overview

The SDK build system creates platform-specific installers that can be dragged to Applications (macOS) or installed via installer (Windows/Linux). All installers include integrity checks to prevent corruption.

## Building Installers

### Quick Build (All Platforms)

```bash
cd sdk
./scripts/build-all.sh
```

This will:
1. Build all SDK components (Native, Web, Bindings, Extension)
2. Automatically create installers for your platform
3. Generate checksums for verification
4. Verify installer integrity

### Platform-Specific Installers

#### macOS (.dmg)
- **Location**: `sdk/installers/NAVΛ-SDK-YYYYMMDD.dmg`
- **Installation**: 
  1. Open the .dmg file
  2. Drag "NAVΛ SDK.app" to Applications folder
  3. Launch from Applications
- **Verification**: DMG is automatically mounted and verified during build
- **Checksum**: `sdk/installers/NAVΛ-SDK-*.dmg.sha256`

#### Windows (.exe)
- **Location**: `sdk/installers/NAVΛ-SDK-Setup-YYYYMMDD.exe`
- **Installation**:
  1. Run the .exe installer
  2. Follow the installation wizard
  3. Choose installation directory
  4. Launch from Start Menu
- **Checksum**: `sdk/installers/NAVΛ-SDK-Setup-*.exe.sha256`

#### Linux (.deb)
- **Location**: `sdk/installers/nava-sdk_1.0.0_ARCH.deb`
- **Installation**:
  1. `sudo dpkg -i nava-sdk_1.0.0_*.deb`
  2. If dependencies missing: `sudo apt-get install -f`
  3. Launch from Applications menu
- **Checksum**: `sdk/installers/nava-sdk_*.deb.sha256`

## Integrity Verification

### Verify Installers

```bash
cd sdk
./scripts/verify-installer.sh
```

This will:
- Check file sizes
- Verify checksums
- Test DMG mounting (macOS)
- Report any corruption issues

### Manual Verification

```bash
# Check DMG (macOS)
hdiutil verify sdk/installers/NAVΛ-SDK-*.dmg

# Check checksum
shasum -a 256 sdk/installers/NAVΛ-SDK-*.dmg
cat sdk/installers/NAVΛ-SDK-*.dmg.sha256
```

## Build Process

The build process includes:

1. **File Verification**: All files are verified before copying
2. **Size Checks**: Empty or too-small files are rejected
3. **DMG Testing**: macOS DMGs are mounted and verified
4. **Checksum Generation**: SHA256 checksums for all installers
5. **Error Handling**: Build fails if any verification step fails

## Troubleshooting

### DMG Corruption Errors

If you see "disk image is corrupted":

1. **Rebuild the installer**:
   ```bash
   cd sdk
   rm -rf installers/
   ./scripts/build-all.sh
   ```

2. **Verify the build**:
   ```bash
   ./scripts/verify-installer.sh
   ```

3. **Check file permissions**:
   ```bash
   ls -lh sdk/installers/
   ```

### Download Issues

If downloads fail:

1. Check that installers exist: `ls sdk/installers/`
2. Verify file sizes are reasonable (not 0 bytes)
3. Rebuild installers if needed
4. Check browser console for errors

## File Structure

```
sdk/
├── scripts/
│   ├── build-all.sh          # Main build script
│   ├── build-installers.sh   # Installer creation
│   └── verify-installer.sh   # Integrity verification
├── installers/               # Generated installers
│   ├── NAVΛ-SDK-*.dmg        # macOS installer
│   ├── NAVΛ-SDK-*.dmg.sha256 # Checksum
│   ├── NAVΛ-SDK-Setup-*.exe  # Windows installer
│   ├── NAVΛ-SDK-Setup-*.exe.sha256
│   ├── nava-sdk_*.deb        # Linux installer
│   └── nava-sdk_*.deb.sha256
└── assets/                   # Logo and icons
```

## Best Practices

1. **Always verify** installers before distribution
2. **Keep checksums** with installers for verification
3. **Test installers** on clean systems before release
4. **Rebuild** if any component changes
5. **Check logs** for any warnings during build

