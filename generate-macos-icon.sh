#!/bin/bash

# Create iconset directory
mkdir -p src-tauri/icons/icon.iconset

# Generate all required sizes for macOS .icns
python3 << 'EOF'
from PIL import Image, ImageDraw, ImageFont
import os

def generate_icon(size):
    img = Image.new('RGBA', (size, size), color='#1a1a1a')
    draw = ImageDraw.Draw(img)
    
    font_size = int(size * 0.7)
    
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf', font_size)
    except:
        try:
            font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', font_size)
        except:
            font = ImageFont.load_default()
    
    text = '⋋'
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    
    x = (size - text_width) // 2 - bbox[0]
    y = (size - text_height) // 2 - bbox[1]
    
    draw.text((x, y), text, fill='#00ff00', font=font)
    
    return img

# macOS iconset sizes
sizes = [
    (16, 'icon_16x16.png'),
    (32, 'icon_16x16@2x.png'),
    (32, 'icon_32x32.png'),
    (64, 'icon_32x32@2x.png'),
    (128, 'icon_128x128.png'),
    (256, 'icon_128x128@2x.png'),
    (256, 'icon_256x256.png'),
    (512, 'icon_256x256@2x.png'),
    (512, 'icon_512x512.png'),
    (1024, 'icon_512x512@2x.png'),
]

iconset_dir = 'src-tauri/icons/icon.iconset'
os.makedirs(iconset_dir, exist_ok=True)

for size, filename in sizes:
    img = generate_icon(size)
    filepath = os.path.join(iconset_dir, filename)
    img.save(filepath, 'PNG')
    print(f'✓ Generated {filename}')

print('\n✅ All iconset files generated!')
EOF

# Convert iconset to icns
iconutil -c icns src-tauri/icons/icon.iconset -o src-tauri/icons/icon.icns

# Clean up iconset directory
rm -rf src-tauri/icons/icon.iconset

echo "✅ Generated icon.icns for macOS!"
