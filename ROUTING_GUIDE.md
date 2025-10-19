# 🚀 NAVΛ Studio - Page Routing Guide

## ✅ Current URL Structure (Correct)

### **1. Main Landing Page**
- **URL**: `http://localhost:3000/`
- **File**: `index.html`
- **Purpose**: Main entry point with Construct-style workspace
- **Features**:
  - ✅ Photorealistic 3D workspace
  - ✅ Center monitor with blueprints
  - ✅ Left sidebar navigation
  - ✅ Bottom icon bar with live clock
  - ✅ Bottom right branding
- **Top Bar Action**: "Switch to Download Interface" → goes to `/download.html`

### **2. Download Page**
- **URL**: `http://localhost:3000/download.html`
- **File**: `download.html`
- **Purpose**: SDK download page with Construct-style workspace
- **Features**:
  - ✅ Same workspace aesthetic as main page
  - ✅ Green download icon (📥) in bottom bar
  - ✅ Direct macOS DMG download (212MB)
  - ✅ Installation instructions with security notes
- **Top Bar Action**: "Switch Back to Previous Interface" → goes to `/`

### **3. IDE Application**
- **URL**: `http://localhost:3000/app.html`
- **File**: `app.html`
- **Purpose**: Full React-based IDE
- **Features**:
  - ✅ Complete IDE interface
  - ✅ Monaco editor
  - ✅ File explorer
  - ✅ Live preview
  - ✅ All IDE features
- **Access**: Click app icons from landing pages

---

## 🔄 Page Switching Flow

```
┌─────────────────────────────────────┐
│  Main Landing (/)                   │
│  - Workspace scene                  │
│  - General navigation               │
│                                     │
│  [Switch to Download Interface] ────┼─────┐
└─────────────────────────────────────┘     │
                                            │
                                            ↓
┌─────────────────────────────────────┐
│  Download Page (/download.html)     │
│  - Same workspace scene             │
│  - Download focused                 │
│  - Green 📥 download button         │
│                                     │
│  [Switch Back to Previous] ─────────┼─────┘
└─────────────────────────────────────┘

        Both pages can launch IDE (/app.html)
                      ↓
        ┌─────────────────────────┐
        │  IDE Application        │
        │  (/app.html)            │
        └─────────────────────────┘
```

---

## 📍 Access URLs

### **For Users:**
1. **Start Here**: `http://localhost:3000/`
2. **Download SDK**: Click "Switch to Download Interface" or go to `http://localhost:3000/download.html`
3. **Launch IDE**: Click 🌐 icon on either page or go to `http://localhost:3000/app.html`

### **Direct Links:**
- Landing: `http://localhost:3000/`
- Downloads: `http://localhost:3000/download.html`
- IDE: `http://localhost:3000/app.html`

---

## 🎯 Use Cases

### **Scenario 1: New Visitor**
1. Arrives at `http://localhost:3000/`
2. Sees beautiful workspace interface
3. Clicks "Switch to Download Interface"
4. Downloads macOS app via green 📥 icon

### **Scenario 2: Direct to Downloads**
1. Arrives at `http://localhost:3000/download.html`
2. Immediately sees download-focused interface
3. Clicks green 📥 icon to download
4. Can switch back to main page if needed

### **Scenario 3: Launch IDE**
1. From any page, clicks 🌐 icon
2. IDE opens at `http://localhost:3000/app.html`
3. Full development environment ready

---

## ✅ Routing is Correct!

Your pages are properly structured:
- ✅ Clean URLs (no port changes needed - using 3000)
- ✅ Consistent navigation between pages
- ✅ Both landing pages have Construct aesthetic
- ✅ Download functionality works
- ✅ IDE is accessible from all pages

---

## 🔧 No Changes Needed

The current structure is optimal:
1. `index.html` at root `/` - Standard web convention
2. `download.html` at `/download.html` - Descriptive URL
3. `app.html` at `/app.html` - Clear application entry

All pages toggle correctly and URLs are clean!

---

*Last Updated: October 13, 2025*
*NAVΛ Studio - Navigation Calculus IDE*

