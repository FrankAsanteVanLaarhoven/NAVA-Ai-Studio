// Generate extension icons from master logo
// This script should be run to generate all required icon sizes

const fs = require('fs');
const path = require('path');

const sizes = [16, 32, 48, 64, 128, 256, 512];
const assetsDir = path.join(__dirname, '../assets');
const masterIcon = path.join(__dirname, '../../assets/icon-512.png');

// Ensure assets directory exists
if (!fs.existsSync(assetsDir)) {
  fs.mkdirSync(assetsDir, { recursive: true });
}

console.log('📦 Generating extension icons...');
console.log('⚠️  Note: This script requires ImageMagick or similar tool');
console.log('   For now, please manually create icons or use an online tool:');
console.log('   - https://realfavicongenerator.net/');
console.log('   - https://www.pwabuilder.com/imageGenerator');
console.log('');
console.log('Required icon sizes:');
sizes.forEach(size => {
  console.log(`   - icon-${size}x${size}.png (${size}x${size}px)`);
});

// If ImageMagick is available, generate icons
const { execSync } = require('child_process');
try {
  execSync('which convert', { stdio: 'ignore' });
  console.log('');
  console.log('✅ ImageMagick found, generating icons...');
  
  sizes.forEach(size => {
    const outputPath = path.join(assetsDir, `icon-${size}x${size}.png`);
    try {
      execSync(`convert "${masterIcon}" -resize ${size}x${size} "${outputPath}"`, {
        stdio: 'ignore'
      });
      console.log(`   ✓ Generated icon-${size}x${size}.png`);
    } catch (error) {
      console.error(`   ✗ Failed to generate icon-${size}x${size}.png`);
    }
  });
  
  console.log('');
  console.log('✅ All icons generated successfully!');
} catch (error) {
  console.log('');
  console.log('⚠️  ImageMagick not found. Please generate icons manually.');
  console.log('   Copy the master icon and resize to required sizes.');
}

