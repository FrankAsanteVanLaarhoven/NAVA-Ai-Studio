# NAVΛ Studio macOS Installation Guide

## Download and Installation

1. **Download** the `NAVΛ-Studio-1.0.0-macOS.dmg` file from our website
2. **Open** the downloaded DMG file
3. **Drag** the NAVΛ Studio app to your Applications folder

## Bypassing macOS Security Warnings

macOS may show a security warning because the app is not yet notarized by Apple. Here's how to bypass it:

### Method 1: Right-Click (Recommended)
1. **Right-click** (or Control-click) on the NAVΛ Studio app in your Applications folder
2. Select **"Open"** from the context menu
3. Click **"Open"** in the security dialog that appears
4. After this one-time process, you can open the app normally

### Method 2: System Preferences
1. Try to open the app normally (it will show the warning)
2. Go to **System Preferences** → **Security & Privacy**
3. Click the **"Open Anyway"** button
4. Confirm by clicking **"Open"**

## Why This Warning Appears

This warning appears because:
- The app is not yet signed with an Apple Developer certificate
- macOS Gatekeeper blocks apps from unidentified developers by default
- This is standard for development and beta software

## Future Improvements

We're working on:
- ✅ Apple Developer account registration
- ✅ Code signing for official releases
- ✅ Apple notarization for seamless installation
- ✅ Mac App Store distribution

For now, the right-click method works perfectly for all features.