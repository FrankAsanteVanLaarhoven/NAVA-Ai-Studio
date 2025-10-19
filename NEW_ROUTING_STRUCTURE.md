# 🎯 NAVΛ Studio - Routing Structure

## ✅ Current URL Structure (Workspace-First)

### **1. Main Entry Point (Workspace Interface) - DEFAULT**
- **URL**: `http://localhost:3000/` ⭐ **← REDIRECTS TO WORKSPACE**
- **File**: `index.html` (auto-redirects to workspace.html)
- **Purpose**: Automatic redirect to workspace interface
- **Behavior**: Immediately redirects to `/workspace.html`

### **2. Workspace Interface - PRIMARY APPLICATION**
- **URL**: `http://localhost:3000/workspace.html`
- **File**: `workspace.html`
- **Purpose**: Main workspace and navigation hub
- **Features**:
  - ✅ Photorealistic 3D workspace environment
  - ✅ Navigation Institute branding
  - ✅ Bottom icon bar with navigation to all sections
  - ✅ AI Assistant integration
  - ✅ Live clock and system info
  - ✅ Access to ROS Learning, IDE, and Downloads
- **Navigation**:
  - 🏠 Home (current page - highlighted)
  - 🦾 ROS Learning Center → `/app.html?activity=ros-learning`
  - ⋋ Full IDE Application → `/app.html`
  - 📥 Downloads & SDK → `/download.html`
  - 📚 Documentation (external)
  - 🤖 AI Assistant (modal)

### **3. Full IDE Application**
- **URL**: `http://localhost:3000/app.html`
- **File**: `app.html`
- **Purpose**: Full React-based IDE with Monaco editor
- **Access**: Click ⋋ icon from workspace
- **Features**:
  - Monaco code editor
  - File explorer
  - Terminal integration
  - ROS Learning mode (with `?activity=ros-learning`)

### **4. Downloads & SDK Page**
- **URL**: `http://localhost:3000/download.html`
- **File**: `download.html`
- **Purpose**: SDK downloads and installation instructions
- **Access**: Click 📥 icon from workspace
- **Features**:
  - Desktop app downloads (macOS, Windows, Linux)
  - SDK packages
  - Installation guides

---

## 🔄 Navigation Flow

```
┌──────────────────────────────────────────┐
│  http://localhost:3000/                  │
│  (index.html)                            │
│                                          │
│  Automatic Redirect                      │
│         ↓                                │
└──────────────────────────────────────────┘
                 ↓
┌──────────────────────────────────────────┐
│  http://localhost:3000/workspace.html    │
│  ⭐ PRIMARY ENTRY POINT                  │
│                                          │
│  Workspace Interface                     │
│  - Navigation Institute Hub              │
│  - Bottom icon navigation bar            │
│  - Access to all features                │
│                                          │
│  Navigation Options:                     │
│  ├─ 🦾 ROS Learning ──────────────┐     │
│  ├─ ⋋ Full IDE ──────────────┐    │     │
│  └─ 📥 Downloads ─────────┐   │    │     │
└───────────────────────────┼───┼────┼─────┘
                           │   │    │
                           ↓   ↓    ↓
        ┌──────────────────┐  │    │
        │  /download.html  │  │    │
        │  Downloads & SDK │  │    │
        └──────────────────┘  │    │
                              ↓    ↓
                    ┌─────────────────────┐
                    │  /app.html          │
                    │  Full IDE           │
                    │                     │
                    │  With optional:     │
                    │  ?activity=ros-     │
                    │   learning          │
                    └─────────────────────┘
```

---

## 🚀 What This Structure Provides

### **Benefits:**

1. **Single Entry Point**: All users start at workspace - no confusion
2. **Clean Navigation**: Clear hierarchy from workspace hub
3. **Contextual Access**: Download and IDE accessed when needed
4. **Professional UX**: Workspace-first approach like modern IDEs
5. **Easy Discovery**: All features accessible from one place

### **User Journey:**

1. **New Visitor**:
   - Visits `http://localhost:3000/`
   - Automatically redirected to workspace
   - Sees Navigation Institute interface
   - Explores features via bottom icon bar

2. **Access ROS Learning**:
   - Click 🦾 icon in workspace
   - Opens `/app.html?activity=ros-learning`
   - Full ROS courses with live terminal

3. **Launch Full IDE**:
   - Click ⋋ icon in workspace
   - Opens `/app.html`
   - Professional code editor environment

4. **Download SDK**:
   - Click 📥 icon in workspace
   - Opens `/download.html`
   - Access all downloads and installers

---

## 📁 File Structure

```
/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE/
├── index.html          ← Redirect page (auto → workspace.html)
├── workspace.html      ← PRIMARY ENTRY POINT ⭐
├── app.html            ← Full IDE application
├── download.html       ← Downloads & SDK page
└── public/
    └── downloads/
        ├── NAVΛ-Studio-1.0.0-macOS.dmg
        └── NAVΛ-Studio-1.0.0-macOS.zip
```

---

## 🎯 Quick Access Guide

### **For End Users:**
- **Start Here**: `http://localhost:3000/` (redirects to workspace)
- **Main Workspace**: `http://localhost:3000/workspace.html`
- **Direct to IDE**: `http://localhost:3000/app.html`
- **Direct to Downloads**: `http://localhost:3000/download.html`

### **For Developers:**
```bash
# Start dev server
npm run dev

# Access points:
# Main entry:     http://localhost:3000/
# Workspace:      http://localhost:3000/workspace.html
# IDE:            http://localhost:3000/app.html
# ROS Learning:   http://localhost:3000/app.html?activity=ros-learning
# Downloads:      http://localhost:3000/download.html
```

---

## ✨ Key Features

### **Workspace Interface (Main Hub)**
- 🏠 Central navigation hub
- 🦾 ROS Learning Center access
- ⋋ Full IDE application access
- 📥 Downloads & SDK access
- 📚 Documentation links
- 🤖 AI Assistant
- ⚙️ Settings and preferences
- 🌍 Language selection
- ⏻ User authentication

### **No Confusion**
- Only ONE main entry point (workspace)
- Clear navigation from workspace to other sections
- Consistent bottom icon bar across pages
- Active state highlighting shows current location

---

## 🔧 Technical Implementation

### **Redirect Mechanism**
```html
<!-- index.html -->
<meta http-equiv="refresh" content="0; url=/workspace.html">
<script>
  window.location.replace('/workspace.html');
</script>
```

### **Vite Configuration**
```typescript
// vite.config.ts
rollupOptions: {
  input: {
    main: 'index.html',        // Redirect page
    workspace: 'workspace.html', // Primary entry
    app: 'app.html',            // IDE
    download: 'download.html',  // Downloads
  },
}
```

---

## ✅ Implementation Complete

All routing is now configured with workspace as the single, clear entry point. Users accessing `http://localhost:3000/` will be automatically redirected to the workspace interface, where they can navigate to all other sections of the application.
