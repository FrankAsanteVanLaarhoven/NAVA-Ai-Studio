import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
      '@components': path.resolve(__dirname, './src/components'),
      '@hooks': path.resolve(__dirname, './src/hooks'),
      '@services': path.resolve(__dirname, './src/services'),
      '@styles': path.resolve(__dirname, './src/styles'),
      '@types': path.resolve(__dirname, './src/types'),
    },
  },
  publicDir: 'public',
  clearScreen: false,
  server: {
    port: 5173,
    strictPort: false,
    open: true,
    headers: {
      'Cache-Control': 'no-store, no-cache, must-revalidate, proxy-revalidate, max-age=0',
      'Pragma': 'no-cache',
      'Expires': '0',
      'Surrogate-Control': 'no-store'
    },
    hmr: {
      overlay: true
    },
    // Serve SDK installers
    fs: {
      allow: ['..'],
    },
  },
  envPrefix: ['VITE_', 'TAURI_'],
  build: {
    target: process.env.TAURI_PLATFORM == 'windows' ? 'chrome105' : 'safari13',
    minify: !process.env.TAURI_DEBUG ? 'esbuild' : false,
    sourcemap: !!process.env.TAURI_DEBUG,
    rollupOptions: {
      input: {
        // Main entry point - redirects to workspace
        main: path.resolve(__dirname, 'index.html'),
        // Primary workspace interface (main application entry)
        workspace: path.resolve(__dirname, 'workspace.html'),
        // Full IDE application (accessed from workspace)
        app: path.resolve(__dirname, 'app.html'),
        // Downloads and SDK page (accessed from workspace)
        download: path.resolve(__dirname, 'download.html'),
        // Documentation page
        docs: path.resolve(__dirname, 'docs.html'),
      },
    },
  },
});

