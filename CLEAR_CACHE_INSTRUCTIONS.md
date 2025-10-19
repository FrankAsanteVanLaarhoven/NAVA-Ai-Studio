# 🔥 CLEAR BROWSER CACHE - STEP BY STEP GUIDE

## ✅ SERVER IS RUNNING ON PORT 5173

The development server is ready at:
```
http://localhost:5173/
```

---

## 🎯 FOLLOW THESE EXACT STEPS:

### **Step 1: Open DevTools**
1. Open your browser (Chrome/Edge/Safari)
2. Go to: `http://localhost:5173/app.html`
3. Press:
   - **Mac**: `Cmd + Option + I`
   - **Windows**: `F12` or `Ctrl + Shift + I`

### **Step 2: Hard Reload**

#### **Method A: Right-Click Reload (BEST)**
1. **Right-click** the refresh button (🔄) next to the address bar
2. Select **"Empty Cache and Hard Reload"**
3. Wait for page to reload

#### **Method B: Keyboard Shortcut**
1. **Mac**: `Cmd + Shift + R`
2. **Windows**: `Ctrl + Shift + F5`
3. Hold all keys together, then release

#### **Method C: Manual Cache Clear**
1. Open DevTools (`F12` or `Cmd+Option+I`)
2. Go to **"Application"** tab (Chrome) or **"Storage"** tab (Firefox)
3. Click **"Clear storage"** or **"Clear site data"**
4. Check **"Cache"** and **"Local Storage"**
5. Click **"Clear"** or **"Clear data"**
6. Refresh page

---

## 🚀 IF STILL NOT SHOWING:

### **Nuclear Option: Complete Browser Reset**
1. **Close ALL browser tabs**
2. **Quit browser completely** (Cmd+Q on Mac, Alt+F4 on Windows)
3. **Open browser fresh**
4. **Type in address bar**: `http://localhost:5173/app.html`
5. **Press Enter**

### **Alternative: Try Incognito/Private Mode**
1. Open **Incognito/Private window**:
   - Chrome: `Cmd+Shift+N` (Mac) or `Ctrl+Shift+N` (Windows)
   - Safari: `Cmd+Shift+N`
   - Firefox: `Cmd+Shift+P` (Mac) or `Ctrl+Shift+P` (Windows)
2. Go to: `http://localhost:5173/app.html`
3. This bypasses all cache!

---

## 🔍 WHAT YOU SHOULD SEE:

When the page loads correctly, you'll see:

```
┌─────────────────────────────────────────────────────────┐
│  NAVΛ Studio  [▶ Run] [⚙ Compile] [👁 Visualize]       │  ← Toolbar
├─────────────────────────────────────────────────────────┤
│ 📂 NAVΛ STUDIO IDE — 📄 No file open  │  12 files • ✓  │  ← NEW! Status Bar
├─────────────────────────────────────────────────────────┤
│ [☰]  EXPLORER                                           │
│       🔄  📄  📁  ⋮   ← NEW! Toolbar buttons           │
│       ▼ NAVΛ STUDIO IDE                                 │
│         ▶ assets                                        │
│         ▶ src                                           │
└─────────────────────────────────────────────────────────┘
```

### **Key Features to Verify:**
- ✅ **Green bar below toolbar** (Project Status Bar)
- ✅ **Project name** "📂 NAVΛ STUDIO IDE" on left
- ✅ **File status** "✓ Saved" on right
- ✅ **Explorer toolbar** with 4 buttons: 🔄 📄 📁 ⋮
- ✅ **Neon green** theme on hover
- ✅ **Collapse buttons** (◀ ▶) in panel headers

---

## ❌ COMMON MISTAKES:

### **Wrong URL:**
```
❌ http://localhost:5174/app.html  (wrong port)
❌ http://localhost:5173/workspace.html  (different page)
❌ http://localhost:5173/  (redirects to workspace)

✅ http://localhost:5173/app.html  (CORRECT!)
```

### **Not Hard Refreshing:**
- Regular refresh (`F5` or `Cmd+R`) **doesn't clear cache**
- You MUST use **Hard Refresh** (`Cmd+Shift+R` or `Ctrl+Shift+F5`)
- Or **Right-click refresh → "Empty Cache and Hard Reload"**

---

## 🛠️ TROUBLESHOOTING:

### **Issue: Server not responding**
```bash
# Check if server is running:
lsof -i:5173

# If nothing, start server:
cd "/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE"
npm run dev
```

### **Issue: See error in console**
1. Open DevTools (`F12`)
2. Go to **Console** tab
3. Look for **red errors**
4. Copy error message and share

### **Issue: Blank page**
1. Check Console for errors
2. Try: `http://localhost:5173/` (should redirect)
3. Check Network tab - are files loading?

---

## ✅ VERIFICATION CHECKLIST:

- [ ] Server running on port 5173 (`lsof -i:5173`)
- [ ] Browser cache cleared (Hard reload)
- [ ] URL is exactly: `http://localhost:5173/app.html`
- [ ] DevTools Console shows no red errors
- [ ] Page loaded (not blank screen)
- [ ] Can see Project Status Bar at top
- [ ] Can see Explorer Toolbar buttons (🔄 📄 📁 ⋮)

---

## 📞 STILL NOT WORKING?

If you've tried everything above and it's still not showing:

1. **Take a screenshot** of what you see
2. **Open DevTools Console** (F12)
3. **Copy any red error messages**
4. **Check the URL** in your address bar
5. **Verify the port number** matches the terminal output

---

## 🎉 SUCCESS!

Once you see the Project Status Bar and Explorer Toolbar, you're all set! 

**Try these features:**
- Click **🔄 Refresh** button in Explorer
- Click **📄 New File** to create a file
- Click **📁 New Folder** to create a folder
- Hover over buttons to see **neon green glow**
- Click **◀ ▶** buttons to collapse panels

---

**🚀 Fresh server is running on port 5173 - Just hard refresh your browser!**

