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

  // Set text properties
  ctx.fillStyle = '#00ff00'; // Green color
  ctx.font = `bold ${size * 0.75}px Arial, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';

  // Draw the navigation symbol ⋋
  ctx.fillText('⋋', size / 2, size / 2);

  return canvas;
}

// Generate icons for all sizes
sizes.forEach(size => {
  const canvas = generateIcon(size);
  const buffer = canvas.toBuffer('image/png');
  const filename = size === 32 ? '32x32.png' : size === 128 ? '128x128.png' : `${size}x${size}.png`;
  const filepath = path.join(__dirname, 'src-tauri', 'icons', filename);

  fs.writeFileSync(filepath, buffer);
  console.log(`Generated ${filename}`);
});

// Also generate the main icon.png (using 128x128 as base)
const mainCanvas = generateIcon(128);
const mainBuffer = mainCanvas.toBuffer('image/png');
fs.writeFileSync(path.join(__dirname, 'src-tauri', 'icons', 'icon.png'), mainBuffer);
console.log('Generated icon.png');

// Generate 128x128@2x.png for retina displays
const retinaCanvas = generateIcon(256); // 128 * 2
const retinaBuffer = retinaCanvas.toBuffer('image/png');
fs.writeFileSync(path.join(__dirname, 'src-tauri', 'icons', '128x128@2x.png'), retinaBuffer);
console.log('Generated 128x128@2x.png');

console.log('All icons generated successfully!');