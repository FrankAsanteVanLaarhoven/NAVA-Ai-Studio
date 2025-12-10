#!/bin/bash
# Verify installer integrity and checksums

set -e

INSTALLERS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../installers" && pwd )"

echo "🔍 Verifying NAVΛ SDK Installers..."
echo ""

if [ ! -d "$INSTALLERS_DIR" ]; then
  echo "❌ Installers directory not found: $INSTALLERS_DIR"
  exit 1
fi

# Check each installer
for installer in "$INSTALLERS_DIR"/*.{dmg,exe,deb,AppImage} 2>/dev/null; do
  if [ -f "$installer" ]; then
    filename=$(basename "$installer")
    size=$(stat -f%z "$installer" 2>/dev/null || stat -c%s "$installer" 2>/dev/null || echo "0")
    
    echo "📦 $filename"
    echo "   Size: $size bytes"
    
    # Check for checksum file
    checksum_file="${installer}.sha256"
    if [ -f "$checksum_file" ]; then
      expected_checksum=$(cat "$checksum_file")
      actual_checksum=$(shasum -a 256 "$installer" 2>/dev/null | awk '{print $1}' || sha256sum "$installer" 2>/dev/null | awk '{print $1}')
      
      if [ "$expected_checksum" == "$actual_checksum" ]; then
        echo "   ✅ Checksum verified: $actual_checksum"
      else
        echo "   ❌ Checksum mismatch!"
        echo "      Expected: $expected_checksum"
        echo "      Actual:   $actual_checksum"
      fi
    else
      echo "   ⚠️  No checksum file found"
    fi
    
    # Additional verification for DMG
    if [[ "$installer" == *.dmg ]]; then
      if command -v hdiutil &> /dev/null; then
        echo "   Testing DMG mount..."
        MOUNT_POINT=$(hdiutil attach -nobrowse -noverify -noautoopen "$installer" 2>/dev/null | grep -E '^/dev/' | awk '{print $3}' || echo "")
        if [ -n "$MOUNT_POINT" ] && [ -d "$MOUNT_POINT" ]; then
          echo "   ✅ DMG mounts successfully"
          hdiutil detach "$MOUNT_POINT" >/dev/null 2>&1 || true
        else
          echo "   ❌ DMG is corrupted or cannot be mounted"
        fi
      fi
    fi
    
    echo ""
  fi
done

echo "✅ Verification complete"

