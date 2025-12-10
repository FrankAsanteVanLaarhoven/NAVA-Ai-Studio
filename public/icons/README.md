# PWA Icons

This directory should contain the following icon files for the Progressive Web App:

- `icon-72x72.png` - 72x72 pixels
- `icon-96x96.png` - 96x96 pixels
- `icon-128x128.png` - 128x128 pixels
- `icon-144x144.png` - 144x144 pixels
- `icon-152x152.png` - 152x152 pixels (iOS)
- `icon-192x192.png` - 192x192 pixels (Android, maskable)
- `icon-384x384.png` - 384x384 pixels
- `icon-512x512.png` - 512x512 pixels (maskable)

## Creating Icons

You can create these icons from your logo using tools like:
- [PWA Asset Generator](https://github.com/onderceylan/pwa-asset-generator)
- [RealFaviconGenerator](https://realfavicongenerator.net/)
- [PWA Builder](https://www.pwabuilder.com/imageGenerator)

The icons should feature the NAVΛ logo (λΛ) with a green glow effect on a dark background.

## Quick Generate Script

You can use ImageMagick or similar tools to generate all sizes from a master 512x512 icon:

```bash
# If you have a master icon-512x512.png
for size in 72 96 128 144 152 192 384; do
  convert icon-512x512.png -resize ${size}x${size} icon-${size}x${size}.png
done
```

