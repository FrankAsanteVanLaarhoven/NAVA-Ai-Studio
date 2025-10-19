# 🚀 NAVΛ Studio IDE - Render Deployment Guide

## ⚡ **QUICK FIX - Copy These Values**

For your Render deployment, use these **exact values**:

---

## 📋 **REQUIRED CONFIGURATION**

### **1. Branch**
```
main
```

### **2. Region**
```
Frankfurt (EU Central)
```
✅ You already have this correct

---

### **3. Root Directory**
```
(leave empty)
```
**Important**: Leave this field **blank** or empty. Your project root is already at the top level.

---

### **4. Build Command** ⚠️ Fix the semicolon
**Current** (with semicolon - can cause issues):
```bash
npm install; npm run build
```

**Change to** (with &&):
```bash
npm install && npm run build
```

**Why**: `&&` ensures the second command only runs if the first succeeds.

---

### **5. Start Command** ⚠️ **THIS IS REQUIRED**

**Option A (Recommended for Static Sites):**
```bash
npx serve dist -s -l 10000
```

**Option B (Using Vite Preview):**
```bash
npm run preview
```

**Option C (For Node Apps):**
```bash
node server.js
```

**For NAVΛ Studio, use Option A** ✅

---

## 🎯 **COMPLETE RENDER CONFIGURATION**

Copy and paste these exact values:

| Field          | Value                              |
| -------------- | ---------------------------------- |
| **Branch**     | `main`                             |
| **Region**     | `Frankfurt (EU Central)`           |
| **Root Directory** | *(leave empty)*                |
| **Build Command** | `npm install && npm run build` |
| **Start Command** | `npx serve dist -s -l 10000`   |

---

## 🔧 **BEFORE DEPLOYING - ADD PREVIEW SCRIPT**

If you want to use `npm run preview` as your start command, first update your `package.json`:

### **Add to package.json scripts:**

```json
{
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "preview": "vite preview --port 10000 --host 0.0.0.0"
  }
}
```

Then commit and push:
```bash
git add package.json
git commit -m "chore: Add preview script for Render deployment"
git push origin main
```

---

## 🌐 **ENVIRONMENT VARIABLES** (Optional)

In Render dashboard, add these environment variables:

```
NODE_ENV=production
PORT=10000
```

---

## 📦 **RENDER AUTO-DEPLOY SETTINGS**

✅ Enable **Auto-Deploy** - deploys automatically on git push  
✅ Set **Branch** to `main`  
✅ Set **Node Version** to `20.x` (in Environment section)

---

## ✅ **VERIFICATION CHECKLIST**

Before clicking "Create Web Service":

- ✅ Branch: `main`
- ✅ Region: Frankfurt (EU Central)
- ✅ Root Directory: (empty)
- ✅ Build Command: `npm install && npm run build`
- ✅ Start Command: `npx serve dist -s -l 10000`
- ✅ Environment Variables: Added (optional)

---

## 🚀 **DEPLOYMENT STEPS**

1. **Fill in the configuration** with values above
2. Click **"Create Web Service"**
3. Wait for build to complete (~3-5 minutes)
4. Your app will be live at: `https://nava-studio.onrender.com`

---

## 🔍 **TROUBLESHOOTING**

### **If Build Fails:**

**Error**: "npm: command not found"
**Solution**: Render should auto-detect Node.js, but if not, add:
```
Environment: Node
Node Version: 20
```

### **If Start Command Fails:**

**Error**: "serve: command not found"
**Solution**: The `npx` prefix will auto-install `serve`. If it still fails, use:
```bash
npm run preview
```
(Make sure you added the preview script to package.json first)

### **If Port Issues:**

Render uses dynamic ports. Make sure your start command uses:
- `npx serve dist -s -l 10000` (serve listens on port 10000)
- Or use environment variable: `npx serve dist -s -l $PORT`

---

## 💡 **RENDER-SPECIFIC TIPS**

### **Free Tier Limits:**
- ✅ Your app will spin down after 15 min of inactivity
- ✅ First request after spin-down takes ~30 seconds
- ✅ Upgrade to paid tier ($7/mo) for 24/7 uptime

### **Custom Domain:**
After deployment, you can add a custom domain:
1. Go to Settings
2. Add Custom Domain
3. Point your DNS to Render

---

## 📊 **EXPECTED DEPLOYMENT TIME**

```
Install dependencies: ~2 minutes
Build application:    ~1 minute
Deploy to CDN:        ~30 seconds
-----------------------------------
Total:                ~3-4 minutes
```

---

## 🎉 **AFTER SUCCESSFUL DEPLOYMENT**

Your NAVΛ Studio IDE will be available at:
```
https://your-app-name.onrender.com
```

Test these URLs:
- `https://your-app-name.onrender.com/` - Landing page
- `https://your-app-name.onrender.com/workspace.html` - Desktop OS
- `https://your-app-name.onrender.com/app.html` - IDE
- `https://your-app-name.onrender.com/download.html` - SDK Download

---

## 🔄 **AUTO-DEPLOY ON GIT PUSH**

Once configured, every time you push to GitHub:
```bash
git push origin main
```

Render will:
1. Detect the push
2. Pull latest code
3. Run build command
4. Deploy automatically ✨

---

## 📈 **MONITORING YOUR DEPLOYMENT**

In Render dashboard:
- **Logs**: View build and runtime logs
- **Metrics**: CPU, Memory, Request stats
- **Shell**: Access to container shell
- **Environment**: Manage environment variables

---

## ✅ **FINAL ANSWER FOR YOUR SCREENSHOT**

**Copy these exact values into Render:**

```
Branch: main
Region: Frankfurt (EU Central)  
Root Directory: (empty)
Build Command: npm install && npm run build
Start Command: npx serve dist -s -l 10000
```

Then click **"Create Web Service"** 🚀

---

## 🌟 **ALTERNATIVE: RENDER.YAML** (Advanced)

For automated configuration, create `render.yaml` in your repo:

```yaml
services:
  - type: web
    name: nava-studio-ide
    env: node
    region: frankfurt
    plan: free
    buildCommand: npm install && npm run build
    startCommand: npx serve dist -s -l 10000
    envVars:
      - key: NODE_ENV
        value: production
```

Commit this file and Render will auto-configure! 🎯

---

*Your NAVΛ Studio IDE will be live on Render in minutes!* ⚡⋋

