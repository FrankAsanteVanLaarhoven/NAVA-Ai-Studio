// Quick script to check model status
const fs = require('fs');
const path = require('path');

// Check environment variables
const envFile = path.join(__dirname, '.env');
let envVars = {};
if (fs.existsSync(envFile)) {
  const envContent = fs.readFileSync(envFile, 'utf8');
  envContent.split('\n').forEach(line => {
    const match = line.match(/^([^=]+)=(.*)$/);
    if (match) {
      envVars[match[1].trim()] = match[2].trim();
    }
  });
}

console.log('📊 Model Status Check');
console.log('====================\n');

// Check API keys
console.log('🔑 API Keys Status:');
console.log('  GEMINI_API_KEY:', envVars.VITE_GEMINI_API_KEY ? '✅ Set' : '❌ Not set');
console.log('  OPENROUTER_API_KEY:', envVars.VITE_OPENROUTER_API_KEY ? '✅ Set' : '❌ Not set');
console.log('  OPENAI_API_KEY:', envVars.VITE_OPENAI_API_KEY ? '✅ Set' : '❌ Not set');
console.log('');

// Check NAVA Local
console.log('🤖 NAVA Local Model:');
console.log('  Server: http://localhost:8080');
console.log('  Status: Check with: curl http://localhost:8080/health');
console.log('');

console.log('💡 To check in browser console:');
console.log('  Open DevTools → Console → Run:');
console.log('  modelConfigService.getEnabledModels()');
