# 🤖 ROBOTIS & Univarm Quick Access Guide

## 🌟 New Featured Section in Workspace!

When you open the workspace, you'll now see a **prominent ROBOTIS & Univarm section** with three large buttons!

---

## 📍 Location

**URL**: http://localhost:5173/workspace.html

**Position**: Top of the main workspace area (above the blueprint canvas)

---

## 🎯 The Three Buttons

### 1. **ROBOTIS-SYSTEMIC** 🔷

**Large blue button with diamond icon**

- **What it does**: Opens the complete ROBOTIS enterprise platform
- **Opens**: http://localhost:3000 in a new tab
- **Features**:
  - WebXR robot control
  - Trust & attestation system
  - Digital twin (Kobuki/TurtleBot2)
  - Enterprise security features
  
**Prerequisites**: Run `./start-robotis-system.sh` first

---

### 2. **Univarm Starter** ⚡

**Large yellow/gold button with lightning icon**

- **What it does**: Opens the path optimization & code generation tool
- **Route**: `/app.html?activity=univarm-starter`
- **Features**:
  - One-click path optimization
  - Multi-language code generation (Rust, C++, Python, TypeScript)
  - Educational mock solver
  - Perfect for learning & prototyping

**Prerequisites**: None! Works immediately

---

### 3. **Univarm Advanced** 🦀

**Large orange button with crab icon**

- **What it does**: Opens the production path planning system
- **Route**: `/app.html?activity=univarm-advanced`
- **Features**:
  - Real Rust backend (Actix Web)
  - SSE streaming (real-time telemetry)
  - Drive simulation
  - Constraint monitoring
  - Production-ready architecture

**Prerequisites**: Run `./start-univarm-backend.sh` for full features

---

## 🎨 Visual Design

Each button has:
- **Large 56px icon** for visual impact
- **Title in 18px bold** for clarity
- **Description in 13px** explaining the system
- **Color-coded border** (blue/yellow/orange)
- **Hover effects**:
  - Lifts up 4px
  - Glowing shadow in brand color
  - Smooth transitions

---

## 🚀 Quick Start Workflows

### Workflow 1: Use Univarm Starter (Immediate)

1. Open workspace: http://localhost:5173/workspace.html
2. **Click the ⚡ yellow button** (Univarm Starter)
3. Set start/goal coordinates
4. Click "Find optimal path (⋋)"
5. Export code to Rust/C++/Python/TypeScript

**No backend needed!** Works immediately!

---

### Workflow 2: Use Univarm Advanced (Production)

**Terminal 1**:
```bash
cd ~/Desktop/"NAVA Studio IDE"
./start-univarm-backend.sh
```

**Terminal 2**:
```bash
npm run dev
```

**Browser**:
1. Open http://localhost:5173/workspace.html
2. **Click the 🦀 orange button** (Univarm Advanced)
3. Solve path
4. Click "▶ Execute Drive"
5. Watch real-time SSE telemetry!

---

### Workflow 3: Use ROBOTIS-SYSTEMIC (Full Platform)

**Terminal**:
```bash
cd ~/Desktop/"NAVA Studio IDE"
./start-robotis-system.sh
```

**Browser**:
1. Workspace opens automatically
2. **Click the 🔷 blue button** (ROBOTIS-SYSTEMIC)
3. Opens WebXR interface in new tab
4. Access XR Bench, presets, actions, trust dashboard

---

## 📊 Button Features

### Responsive Design
- 3-column grid on large screens
- Stacks on smaller screens
- Touch-friendly on tablets

### Accessibility
- Clear visual hierarchy
- High contrast colors
- Keyboard accessible
- Screen reader friendly

### Visual Feedback
- Hover state changes
- Active state indication
- Smooth animations
- Color-coded systems

---

## 🎯 What Each System Does

### 🔷 ROBOTIS-SYSTEMIC
**Best for**: Enterprise robot control, WebXR, trust systems

**Use when you need**:
- Full robot control platform
- Trust & attestation
- Evidence pack generation
- WebXR interface
- Enterprise features (RBAC, audit)

---

### ⚡ Univarm Starter
**Best for**: Learning, prototyping, code generation

**Use when you need**:
- Quick path optimization
- Multi-language code export
- Educational examples
- Rapid prototyping
- No backend required

---

### 🦀 Univarm Advanced
**Best for**: Production path planning, real-time systems

**Use when you need**:
- Real backend solver
- SSE streaming telemetry
- Drive simulation
- Constraint monitoring
- Production deployment

---

## 💡 Tips

### First Time?
**Start with ⚡ Univarm Starter** - No setup needed!

### Need Production?
**Use 🦀 Univarm Advanced** - Start backend first

### Want Full Platform?
**Use 🔷 ROBOTIS-SYSTEMIC** - Complete enterprise system

---

## 🔧 Troubleshooting

### Button Doesn't Work

**⚡ Univarm Starter**:
- Should always work (no backend needed)
- If not, check console for errors

**🦀 Univarm Advanced**:
- Requires backend: `./start-univarm-backend.sh`
- Check backend is on port 8080: `lsof -i :8080`

**🔷 ROBOTIS-SYSTEMIC**:
- Requires services: `./start-robotis-system.sh`
- Check ports 3000 and 8080 are running

### Can't See Buttons

**Refresh the page**: `Cmd+R` or `Ctrl+R`

**Clear cache**: `Cmd+Shift+R` or `Ctrl+Shift+R`

**Check you're on workspace**: http://localhost:5173/workspace.html

---

## 🎉 Quick Reference

**Location**: Top of workspace  
**Section**: "🤖 ROBOTIS & Univarm Systems"  
**Buttons**: 3 large, color-coded buttons  
**Position**: Above blueprint canvas  

**Colors**:
- 🔷 Blue = ROBOTIS-SYSTEMIC
- ⚡ Yellow = Univarm Starter
- 🦀 Orange = Univarm Advanced

---

**🎊 Easy access to all your robotics systems! Click and go!** 🚀

