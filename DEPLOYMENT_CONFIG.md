# 🚀 NAVΛ Studio IDE - Cloud Deployment Configuration

## 📋 **DEPLOYMENT SETTINGS**

Based on your screenshot, here are the correct configuration values:

---

## ⚙️ **Configuration Values**

### **1. Branch**
```
main
```
✅ Correct - Deploy from your main branch

---

### **2. Region**
```
Frankfurt (EU Central) - 3 existing services
```
✅ You've selected Frankfurt, which is good for EU users
💡 **Alternative**: Ohio (US East) for US users

---

### **3. Root Directory**
```
(leave empty or use: /)
```

**Why**: Your project root is already at the top level.

If deploying specific parts:
- Frontend only: `./` (root)
- Simulation platform: `./simulation-platform`

---

### **4. Build Command**

For **Vite/Web App** (workspace.html, app.html, download.html):
```bash
npm install && npm run build
```

For **Tauri Desktop App**:
```bash
npm install && npm run tauri:build
```

**Current Issue**: You have `npm install; npm run build` with semicolon.
**Fix**: Use `&&` instead of `;` for better error handling.

---

### **5. Start Command** ⚠️ **REQUIRED - THIS IS YOUR ERROR**

This is where you're getting the error! Here are the correct options:

#### **Option A: Static Site Deployment** (Recommended)
If deploying as a static site (workspace.html, app.html):
```bash
npx serve dist -p $PORT
```

#### **Option B: Vite Preview**
```bash
npm run preview
```

#### **Option C: Custom Server**
If you have a custom server:
```bash
node server.js
```

---

## 📦 **COMPLETE CONFIGURATION**

### **For Railway/Render/Similar Platforms:**

```yaml
Branch: main
Region: Frankfurt (EU Central)
Root Directory: (empty)
Build Command: npm install && npm run build
Start Command: npx serve dist -p $PORT
```

---

## 🔧 **REQUIRED CHANGES TO YOUR PROJECT**

### **1. Add `serve` to package.json** (if using static deployment)

Add this to your `package.json`:

```json
{
  "scripts": {
    "preview": "vite preview --port $PORT --host 0.0.0.0"
  },
  "devDependencies": {
    "serve": "^14.2.1"
  }
}
```

### **2. Update `vite.config.ts`** for production

Ensure your `vite.config.ts` has correct base path:

```typescript
export default defineConfig({
  base: './',
  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        app: resolve(__dirname, 'app.html'),
        workspace: resolve(__dirname, 'workspace.html'),
        download: resolve(__dirname, 'download.html'),
      }
    }
  },
  server: {
    port: 5173,
    host: '0.0.0.0'
  },
  preview: {
    port: process.env.PORT || 4173,
    host: '0.0.0.0'
  }
})
```

---

## 🎯 **RECOMMENDED DEPLOYMENT OPTIONS**

### **Option 1: Static Site (Best for Web App)**

**Platform**: Vercel, Netlify, or Cloudflare Pages

**Settings**:
```
Build Command: npm run build
Output Directory: dist
Install Command: npm install
```

**No start command needed** - it's static!

---

### **Option 2: Node Server (For Dynamic Features)**

**Platform**: Railway, Render, Fly.io

**Settings**:
```
Branch: main
Build Command: npm install && npm run build
Start Command: npx serve dist -p $PORT
Port: $PORT (automatic)
```

---

### **Option 3: Docker Deployment**

Create `Dockerfile`:

```dockerfile
FROM node:20-alpine AS builder

WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM node:20-alpine

WORKDIR /app
COPY --from=builder /app/dist ./dist
RUN npm install -g serve

EXPOSE 3000
CMD ["serve", "-s", "dist", "-p", "3000"]
```

**Settings**:
```
Dockerfile: Dockerfile
Port: 3000
```

---

## ✅ **QUICK FIX FOR YOUR SCREENSHOT**

Based on your error, change the **Start Command** to:

```bash
npx serve dist -s -p $PORT
```

Or simpler:

```bash
npm run preview
```

Then add to `package.json`:
```json
"scripts": {
  "preview": "vite preview --port ${PORT:-4173} --host 0.0.0.0"
}
```

---

## 🌐 **ENVIRONMENT VARIABLES**

Add these environment variables in your deployment platform:

```env
NODE_ENV=production
PORT=3000
VITE_API_URL=https://your-api-url.com
```

---

## 📝 **DEPLOYMENT CHECKLIST**

Before deploying, ensure:

- ✅ All files committed to GitHub
- ✅ `package.json` has `build` script
- ✅ `vite.config.ts` configured for production
- ✅ Environment variables set
- ✅ Start command specified
- ✅ Port configuration correct
- ✅ Build output directory is `dist`

---

## 🚀 **STEP-BY-STEP DEPLOYMENT**

### **If using Railway/Render:**

1. **Connect GitHub Repository**
   - Link: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio

2. **Configure Build Settings**:
   ```
   Branch: main
   Root Directory: (empty)
   Build Command: npm install && npm run build
   Start Command: npx serve dist -s -p $PORT
   ```

3. **Set Environment Variables**:
   ```
   NODE_ENV=production
   ```

4. **Deploy** 🎉

---

### **If using Vercel (Easiest for Static Sites):**

1. Import GitHub repository
2. Vercel auto-detects Vite
3. Override settings:
   ```
   Build Command: npm run build
   Output Directory: dist
   Install Command: npm install
   ```
4. Deploy automatically ✨

---

## 🔍 **TROUBLESHOOTING**

### **Error: "Start Command Required"**
**Solution**: Add `npx serve dist -s -p $PORT`

### **Error: "Build Failed"**
**Solution**: Ensure all dependencies in `package.json`

### **Error: "Port already in use"**
**Solution**: Use `$PORT` environment variable

### **Error: "Cannot find module"**
**Solution**: Run `npm install` before `npm run build`

---

## 💡 **BEST PRACTICE**

For NAVΛ Studio IDE, I recommend:

### **Static Deployment on Vercel**:
```
Framework: Vite
Build Command: npm run build
Output Directory: dist
Node Version: 20.x
```

**Why**:
- ✅ Free tier available
- ✅ Automatic HTTPS
- ✅ CDN distribution
- ✅ Zero configuration
- ✅ GitHub integration
- ✅ Instant deployments

---

## 🌟 **EXAMPLE: SUCCESSFUL DEPLOYMENT**

After deployment, your app will be available at:
- **Vercel**: `https://nava-studio.vercel.app`
- **Railway**: `https://nava-studio-production.up.railway.app`
- **Render**: `https://nava-studio.onrender.com`

---

## 📦 **ALTERNATIVE: DEPLOY TO GITHUB PAGES**

Since your code is on GitHub:

1. **Add to `package.json`**:
```json
{
  "scripts": {
    "predeploy": "npm run build",
    "deploy": "gh-pages -d dist"
  },
  "devDependencies": {
    "gh-pages": "^6.1.0"
  }
}
```

2. **Run**:
```bash
npm install gh-pages --save-dev
npm run deploy
```

3. **Enable GitHub Pages**:
   - Go to: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/settings/pages
   - Source: Deploy from branch
   - Branch: `gh-pages`
   - Folder: `/` (root)

4. **Your site will be live at**:
   ```
   https://frankasantevanlaarhoven.github.io/NAVA-Ai-Studio/
   ```

---

## ✅ **FINAL ANSWER FOR YOUR SCREENSHOT**

**Change the Start Command from**:
```bash
$ yarn start
```

**To**:
```bash
npx serve dist -s -p $PORT
```

Or:
```bash
npm run preview
```

Then click **Deploy** ✨

---

*Your NAVΛ Studio IDE will be live on the web!* 🚀⋋

