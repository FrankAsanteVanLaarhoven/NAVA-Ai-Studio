import { createCanvas } from 'canvas';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Icon sizes needed for Tauri
const sizes = [32, 128, 256, 512];

function generateIcon(size) {
  const canvas = createCanvas(size, size);
  const ctx = canvas.getContext('2d');

  // Fill background with dark color (#1a1a1a)
  ctx.fillStyle = '#1a1a1a';
  ctx.fillRect(0, 0, size, size);

  // Set text properties for the navigation symbol
  ctx.fillStyle = '#00ff00'; // Bright green color
  ctx.font = `bold ${Math.floor(size * 0.7)}px Arial, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';

  // Draw the navigation symbol ⋋ (U+22CB)
  ctx.fillText('⋋', size / 2, size / 2);

  return canvas;
}

// Ensure icons directory exists
const iconsDir = path.join(__dirname, 'src-tauri', 'icons');
if (!fs.existsSync(iconsDir)) {
  fs.mkdirSync(iconsDir, { recursive: true });
}

// Generate icons for all sizes
sizes.forEach(size => {
  const canvas = generateIcon(size);
  const buffer = canvas.toBuffer('image/png');
  const filename = `${size}x${size}.png`;
  const filepath = path.join(iconsDir, filename);

  fs.writeFileSync(filepath, buffer);
  console.log(`✓ Generated ${filename}`);
});

// Generate specific named files
const specificFiles = [
  { size: 32, name: '32x32.png' },
  { size: 128, name: '128x128.png' },
  { size: 128, name: 'icon.png' },
  { size: 256, name: '128x128@2x.png' }
];

specificFiles.forEach(({ size, name }) => {
  const canvas = generateIcon(size);
  const buffer = canvas.toBuffer('image/png');
  const filepath = path.join(iconsDir, name);
  fs.writeFileSync(filepath, buffer);
  console.log(`✓ Generated ${name}`);
});

console.log('\n✅ All icons generated successfully with navigation symbol ⋋!');
