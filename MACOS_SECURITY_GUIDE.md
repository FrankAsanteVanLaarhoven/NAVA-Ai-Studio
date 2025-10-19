# macOS Security & Code Signing Guide

## Current Status: Unsigned App

Your NAVΛ Studio app is currently **unsigned**, which means macOS will show a security warning when users try to open it for the first time.

---

## 🔓 For Users: How to Open Unsigned Apps

When you first try to open NAVΛ Studio, macOS will show this warning:
```
"NAVΛ Studio.app" cannot be opened because it is from an unidentified developer.
```

### Solution: Right-Click Method

1. **Locate** the app in your Applications folder or Downloads
2. **Right-click** (or Control+click) on **NAVΛ Studio.app**
3. Select **"Open"** from the context menu
4. Click **"Open"** again in the security dialog that appears

✅ **After this one-time process**, you can open the app normally (double-click, dock, Spotlight, etc.)

### Alternative: System Preferences Method

1. Go to **System Preferences** → **Security & Privacy**
2. Click the **"Open Anyway"** button (appears after attempting to open the app)
3. Confirm by clicking **"Open"**

---

## 🔐 For Developers: Code Signing (Future)

To remove the security warning completely, you need to **code sign** your app with an Apple Developer certificate.

### Benefits of Code Signing

- ✅ No security warnings for users
- ✅ Professional appearance
- ✅ Required for Mac App Store distribution
- ✅ Enables Gatekeeper approval
- ✅ Notarization eligibility

### Requirements

1. **Apple Developer Account** ($99/year)
   - Sign up at: https://developer.apple.com
   
2. **Developer ID Certificate**
   - Create in Apple Developer Portal
   - Download and install in Keychain Access

### How to Code Sign

Once you have a Developer ID certificate:

```bash
# Sign the app
codesign --deep --force --verify --verbose \
  --sign "Developer ID Application: Your Name (TEAM_ID)" \
  "NAVΛ Studio.app"

# Verify the signature
codesign --verify --verbose=4 "NAVΛ Studio.app"
spctl --assess --verbose=4 --type execute "NAVΛ Studio.app"
```

### Notarization (Recommended)

After signing, you can also **notarize** your app with Apple:

```bash
# Create a zip for notarization
ditto -c -k --keepParent "NAVΛ Studio.app" "NAVΛ Studio.zip"

# Submit for notarization
xcrun notarytool submit "NAVΛ Studio.zip" \
  --apple-id "your-email@example.com" \
  --team-id "TEAM_ID" \
  --password "app-specific-password"

# Check status
xcrun notarytool info SUBMISSION_ID \
  --apple-id "your-email@example.com" \
  --team-id "TEAM_ID" \
  --password "app-specific-password"

# Staple the notarization ticket
xcrun stapler staple "NAVΛ Studio.app"
```

### Automated Code Signing with Tauri

When we fix the Tauri build issues, Tauri can automatically code sign during the build process.

In `tauri.conf.json`:

```json
{
  "tauri": {
    "bundle": {
      "macOS": {
        "signingIdentity": "Developer ID Application: Your Name (TEAM_ID)",
        "entitlements": "entitlements.plist"
      }
    }
  }
}
```

---

## 📝 Current Distribution Strategy

**For now, we're using unsigned apps with user instructions:**

✅ **Pros:**
- No Apple Developer account needed ($0)
- Immediate distribution
- Works perfectly for beta/internal testing
- Simple instructions for users

⚠️ **Cons:**
- Users see security warning (one time)
- Not suitable for Mac App Store
- Requires user action to bypass Gatekeeper

---

## 🚀 Next Steps

### Immediate (Current)
- ✅ Distribute unsigned DMG with clear instructions
- ✅ Landing page includes security warning info
- ✅ Users know to right-click > Open

### Short-term (Optional)
- Consider Apple Developer account if distributing widely
- Implement code signing for professional releases
- Set up automated signing in CI/CD

### Long-term (Professional)
- Full notarization workflow
- Mac App Store distribution
- Automatic updates via Sparkle or similar

---

## 📚 Resources

- [Apple Code Signing Guide](https://developer.apple.com/library/archive/documentation/Security/Conceptual/CodeSigningGuide/)
- [Gatekeeper and Runtime Protection](https://support.apple.com/en-us/HT202491)
- [Notarizing macOS Software](https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution)
- [Tauri Code Signing Docs](https://tauri.app/v1/guides/distribution/sign-macos)

---

## 🎯 Summary

**You're all set for now!** Your app works perfectly - users just need to right-click > Open the first time. This is a standard approach for development and beta distribution.

When you're ready to scale to larger audiences, consider getting an Apple Developer account and implementing code signing.

---

*Created: October 13, 2025*
*NAVΛ Studio - Van Laarhoven Navigation Calculus IDE*

