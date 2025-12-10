#!/bin/bash
# Generate PWA icons from a master icon
# Usage: ./scripts/generate-pwa-icons.sh <master-icon.png>

if [ -z "$1" ]; then
  echo "Usage: $0 <master-icon.png>"
  echo "Example: $0 public/favicon.svg"
  exit 1
fi

MASTER_ICON="$1"
OUTPUT_DIR="public/icons"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Check if ImageMagick is available
if command -v convert &> /dev/null; then
  echo "Using ImageMagick to generate icons..."
  
  # Generate PNG icons from SVG or PNG master
  for size in 72 96 128 144 152 192 384 512; do
    convert "$MASTER_ICON" -resize "${size}x${size}" -background transparent \
      "$OUTPUT_DIR/icon-${size}x${size}.png"
    echo "Generated icon-${size}x${size}.png"
  done
  
  echo "All icons generated successfully!"
elif command -v sips &> /dev/null; then
  echo "Using macOS sips to generate icons..."
  
  # macOS alternative using sips
  for size in 72 96 128 144 152 192 384 512; do
    sips -z "$size" "$size" "$MASTER_ICON" --out "$OUTPUT_DIR/icon-${size}x${size}.png"
    echo "Generated icon-${size}x${size}.png"
  done
  
  echo "All icons generated successfully!"
else
  echo "Error: Neither ImageMagick nor sips found."
  echo "Please install ImageMagick (brew install imagemagick) or use an online tool."
  echo ""
  echo "You can also manually create icons using:"
  echo "- https://realfavicongenerator.net/"
  echo "- https://www.pwabuilder.com/imageGenerator"
  exit 1
fi

