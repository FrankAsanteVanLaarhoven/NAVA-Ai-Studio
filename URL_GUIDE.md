# 🌐 NAVΛ STUDIO IDE - URL ROUTING GUIDE

## ✅ CORRECT URLS FOR DEVELOPMENT

### **Primary URLs (Port 5173):**

1. **Main Entry Point**:
   ```
   http://localhost:5173/
   or
   http://localhost:5173/index.html
   ```
   → **Redirects to**: `http://localhost:5173/workspace.html`

2. **Workspace (Landing Page)**:
   ```
   http://localhost:5173/workspace.html
   ```
   → Shows workspace with widgets, clock, weather, news, etc.
   → Click "Open IDE" button to access full IDE

3. **Full IDE Application**:
   ```
   http://localhost:5173/app.html
   ```
   → **Complete NAVΛ Studio IDE** with:
   - ✅ Project Status Bar (NEW!)
   - ✅ File Explorer with toolbar (NEW!)
   - ✅ Code Editor (Monaco)
   - ✅ Multi-Language Notebook
   - ✅ AI Assistant
   - ✅ Simulation Panel
   - ✅ All features

4. **Download Page**:
   ```
   http://localhost:5173/download.html
   ```
   → SDK downloads and desktop app

---

## ❌ INCORRECT URL

You were using:
```
http://localhost:5174/app.html  ❌ WRONG PORT!
```

**Correct URL:**
```
http://localhost:5173/app.html  ✅ CORRECT!
```

---

## 🎯 QUICK ACCESS

### **To see all new features:**

1. **Open browser**
2. **Navigate to**: `http://localhost:5173/app.html`
3. **Hard refresh**: `Cmd+Shift+R` (Mac) or `Ctrl+Shift+F5` (Windows)

### **What you'll see:**

```
┌───────────────────────────────────────────────────────────────┐
│  NAVΛ Studio  [▶ Run] [⚙ Compile] [👁 Visualize] [☁ Deploy]  │ ← Toolbar
├───────────────────────────────────────────────────────────────┤
│ 📂 NAVΛ STUDIO IDE — 📄 workspace.html  │  12 files • ✓ Saved │ ← NEW! Status Bar
├───────────────────────────────────────────────────────────────┤
│ ▦│ [EXPLORER]                           │ Code Editor │ [AI] │
│  │ 🔄 📄 📁 ⋮  ← NEW! Toolbar          │             │      │
│  │ ▼ NAVΛ STUDIO IDE                   │             │      │
│  │   ▶ assets                          │             │      │
│  │   ▶ src                             │             │      │
└──────────────────────────────────────────────────────────────┘
```

---

## 🔄 HOW ROUTING WORKS

### **Entry Flow:**
```
http://localhost:5173/
         ↓ (redirects)
http://localhost:5173/workspace.html
         ↓ (click "Open IDE")
http://localhost:5173/app.html
```

### **Vite Configuration:**
```typescript
// vite.config.ts
rollupOptions: {
  input: {
    main: 'index.html',        // → redirects to workspace
    workspace: 'workspace.html', // → landing page
    app: 'app.html',            // → FULL IDE ✅
    download: 'download.html'   // → downloads
  }
}
```

---

## 🚀 DEVELOPMENT SERVER

### **Start Server:**
```bash
npm run dev
```

### **Server will start on:**
```
Port: 5173 (default)
```

If port 5173 is busy, Vite automatically tries:
- 5174
- 5175
- etc.

**Check terminal output** to see which port was assigned!

---

## 🛠️ TROUBLESHOOTING

### **Issue 1: "Cannot GET /app.html"**
**Solution**: Check the port number in terminal output
```bash
# Look for:
  VITE v5.x.x  ready in xxx ms
  
  ➜  Local:   http://localhost:5173/  ← USE THIS PORT!
```

### **Issue 2: Old version showing**
**Solution**: Hard refresh
- Mac: `Cmd + Shift + R`
- Windows: `Ctrl + Shift + F5`
- Or: Open DevTools → Right-click refresh → "Empty Cache and Hard Reload"

### **Issue 3: Port 5173 already in use**
**Solution**: Kill existing process
```bash
# Mac/Linux:
lsof -ti:5173 | xargs kill

# Then restart:
npm run dev
```

---

## 📱 ALL AVAILABLE PAGES

| URL | Description | New Features |
|-----|-------------|--------------|
| `/` or `/index.html` | Redirect page | Auto-redirects to workspace |
| `/workspace.html` | Landing page | Widgets, clock, news feed |
| `/app.html` | **FULL IDE** | ✅ Status Bar, ✅ Explorer Toolbar |
| `/download.html` | Downloads | SDK, Desktop app |

---

## ✅ VERIFICATION CHECKLIST

- [ ] Server running on **port 5173**
- [ ] Navigate to `http://localhost:5173/app.html`
- [ ] Hard refresh browser (`Cmd+Shift+R`)
- [ ] See **Project Status Bar** at top
- [ ] See **Explorer Toolbar** with 🔄 📄 📁 buttons
- [ ] See **neon green** theme throughout
- [ ] Test **collapse buttons** in panels
- [ ] Test **refresh/new file/new folder** buttons

---

## 🎉 YOUR NEW FEATURES ARE HERE!

All implemented features are in:
```
http://localhost:5173/app.html  ✅
```

**NOT in:**
```
http://localhost:5173/workspace.html  ❌ (different page)
http://localhost:5174/app.html        ❌ (wrong port)
```

---

## 🚀 QUICK START

1. **Open Terminal**
2. **Check if server is running**: `lsof -i:5173`
3. **If not running**: `npm run dev`
4. **Open Browser**: `http://localhost:5173/app.html`
5. **Hard Refresh**: `Cmd+Shift+R`
6. **Enjoy your enhanced IDE!** 🎉

---

**📍 Remember: Always use port 5173 (or the port shown in terminal) and access `/app.html` for the full IDE!**

