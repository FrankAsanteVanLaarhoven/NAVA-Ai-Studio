#!/usr/bin/env python3
from PIL import Image, ImageDraw, ImageFont
import os

# Icon sizes needed for Tauri
sizes = [32, 128, 256, 512]

def generate_icon(size):
    # Create image with dark background
    img = Image.new('RGBA', (size, size), color='#1a1a1a')
    draw = ImageDraw.Draw(img)
    
    # Calculate font size (70% of image size)
    font_size = int(size * 0.7)
    
    try:
        # Try to use a system font that supports the navigation symbol
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf', font_size)
    except:
        try:
            font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', font_size)
        except:
            # Fallback to default font
            font = ImageFont.load_default()
    
    # Navigation symbol ⋋
    text = '⋋'
    
    # Get text bounding box for centering
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    
    # Calculate position to center the text
    x = (size - text_width) // 2 - bbox[0]
    y = (size - text_height) // 2 - bbox[1]
    
    # Draw the navigation symbol in bright green
    draw.text((x, y), text, fill='#00ff00', font=font)
    
    return img

# Ensure icons directory exists
icons_dir = 'src-tauri/icons'
os.makedirs(icons_dir, exist_ok=True)

# Generate icons for all sizes
for size in sizes:
    img = generate_icon(size)
    filename = f'{size}x{size}.png'
    filepath = os.path.join(icons_dir, filename)
    img.save(filepath, 'PNG')
    print(f'✓ Generated {filename}')

# Generate specific named files
specific_files = [
    (32, '32x32.png'),
    (128, '128x128.png'),
    (128, 'icon.png'),
    (256, '128x128@2x.png')
]

for size, name in specific_files:
    img = generate_icon(size)
    filepath = os.path.join(icons_dir, name)
    img.save(filepath, 'PNG')
    print(f'✓ Generated {name}')

print('\n✅ All icons generated successfully with navigation symbol ⋋!')
