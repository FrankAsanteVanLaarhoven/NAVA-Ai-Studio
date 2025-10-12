# NAVΛ Studio Icons

This directory contains the application icons in various sizes for different platforms.

## Required Icons

- `32x32.png` - Small window icon
- `128x128.png` - Standard window icon
- `128x128@2x.png` - Retina display icon
- `icon.icns` - macOS application icon
- `icon.ico` - Windows application icon
- `icon.png` - Linux application icon

## Generating Icons

You can use the following tools to generate icons:

### From SVG

```bash
# Install ImageMagick
brew install imagemagick  # macOS
apt-get install imagemagick  # Linux

# Generate PNG icons
convert icon.svg -resize 32x32 32x32.png
convert icon.svg -resize 128x128 128x128.png
convert icon.svg -resize 256x256 128x128@2x.png
```

### macOS ICNS

```bash
mkdir icon.iconset
convert icon.svg -resize 16x16 icon.iconset/icon_16x16.png
convert icon.svg -resize 32x32 icon.iconset/icon_16x16@2x.png
convert icon.svg -resize 32x32 icon.iconset/icon_32x32.png
convert icon.svg -resize 64x64 icon.iconset/icon_32x32@2x.png
convert icon.svg -resize 128x128 icon.iconset/icon_128x128.png
convert icon.svg -resize 256x256 icon.iconset/icon_128x128@2x.png
convert icon.svg -resize 256x256 icon.iconset/icon_256x256.png
convert icon.svg -resize 512x512 icon.iconset/icon_256x256@2x.png
convert icon.svg -resize 512x512 icon.iconset/icon_512x512.png
convert icon.svg -resize 1024x1024 icon.iconset/icon_512x512@2x.png
iconutil -c icns icon.iconset
```

### Windows ICO

```bash
convert icon.svg -define icon:auto-resize=256,128,96,64,48,32,16 icon.ico
```

## Icon Design Guidelines

The NAVΛ Studio icon should:
- Feature the ⋋ (lambda navigation) symbol prominently
- Use the VNC color scheme (green: #00ff00, cyan: #00ccff)
- Be recognizable at small sizes
- Work well on both light and dark backgrounds
- Be professional and modern

## Placeholder Icons

Until custom icons are created, you can use these placeholder commands:

```bash
# Create simple placeholder icons with the ⋋ symbol
# These will be replaced with professional designs
```

