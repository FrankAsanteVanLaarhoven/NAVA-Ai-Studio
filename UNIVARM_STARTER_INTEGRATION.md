# 🚀 Univarm Starter Integration - Complete Guide

**Univarm Starter** is now fully integrated into NAVΛ Studio IDE!

## ✨ What is Univarm Starter?

A minimal, production-ready app that provides:
- ⋋ **One-click path optimization** - "Find optimal path" button
- 🔧 **Multi-language code generation** - Export to Rust, C++, Python, TypeScript
- 🎯 **Real-time solver** - Configurable start/goal with live results
- 📦 **Manifest-backed actions** - Command palette integration (`λopt` prefix)
- 🔌 **Swappable engine** - Mock solver can be replaced with real NAVΛ engine

---

## 🎯 Quick Access

### **Option 1: Via Dock Icon** ⚡
1. Open NAVΛ Studio IDE at http://localhost:5175/workspace.html
2. Look for the **⚡ (lightning bolt)** icon in the dock
3. Click it to launch Univarm Starter

### **Option 2: Via Direct URL**
```
http://localhost:5175/app.html?activity=univarm-starter
```

---

## 📁 Project Structure

```
src/apps/univarm-starter/
├── App.tsx                    # Main app component
├── index.ts                   # Entry point & exports
├── manifest.ts                # App metadata & capabilities
├── styles.css                 # App-specific styles
├── ui/
│   └── Toolbar.tsx           # Action launcher & result panel
├── engine/
│   └── mock.ts               # Mock path solver (swappable)
├── codegen/
│   └── emit.ts               # Multi-language code emitter
├── prefixes/
│   └── nava-prefixes.json    # Action manifest
└── nav/
    └── univarm.navλ          # Navigation calculus definitions
```

---

## 🔧 Key Features

### 1. One-Click Path Optimization

**Action**: `Find optimal path (⋋)`  
**Prefix**: `λopt`

- Set start coordinates (x, y, z)
- Set goal coordinates (x, y, z)
- Configure sample count
- Run solver to get optimal path

### 2. Multi-Language Code Generation

Export the optimized path to:
- **Rust** - Production-ready Rust code
- **C++** - High-performance C++ implementation
- **Python** - Rapid prototyping & data science
- **TypeScript** - Web & Node.js applications

### 3. Real-Time Results

- Live cost calculation
- Path point visualization
- Configurable solver parameters
- Sample count optimization

---

## 🔌 Integration Points

### Dock Integration

The app is registered in the dock with:
- **Icon**: ⚡ (lightning bolt)
- **Name**: "Univarm ⋋"
- **Route**: `/app.html?activity=univarm-starter`
- **Position**: 9th icon (between Browser and Extensions)

### Command Palette Integration

The app registers actions in the command palette:
- **ID**: `univarm.findOptimalPath`
- **Title**: "Find optimal path (⋋)"
- **Prefix**: `λopt`
- **Shortcut**: `Cmd+Shift+P`

---

## 🎨 User Interface

### Left Panel (Control Panel)
- App header with icon and title
- Status badges showing active features
- Path solver controls (start/goal inputs)
- One-click "Find optimal path" button
- Export controls for all supported languages
- Code output textarea
- Prefix manifest viewer

### Right Panel (Information)
- Quick start guide
- Customization instructions
- Feature list
- API documentation

---

## 🔧 Customizing the Solver

The default `src/apps/univarm-starter/engine/mock.ts` provides a simple straight-line solver. To use your real NAVΛ engine:

### Replace the Mock Engine

```typescript
// src/apps/univarm-starter/engine/real-engine.ts

import { navlambdaEngine } from '@nava/kernel'; // Your real engine

export async function solveOptimalPath(params: {
  start: {x: number, y: number, z: number},
  goal: {x: number, y: number, z: number},
  samples?: number
}): Promise<{
  points: Array<{x: number, y: number, z: number}>,
  cost: number
}> {
  // Use your real NAVΛ engine here
  const result = await navlambdaEngine.optimize(params);
  
  return {
    points: result.trajectory,
    cost: result.totalCost
  };
}
```

Then update the import in `src/apps/univarm-starter/ui/Toolbar.tsx`:

```typescript
// Change this:
import { solveOptimalPath } from '../engine/mock'

// To this:
import { solveOptimalPath } from '../engine/real-engine'
```

---

## 📊 Code Generation

The code generator supports multiple languages with idiomatic output:

### Rust Example
```rust
pub fn get_path() -> Vec<(f64, f64, f64)> {
    vec![
        (0.0, 0.0, 0.0),
        (1.0, 1.0, 1.0),
        (5.0, 5.0, 5.0),
    ]
}
```

### C++ Example
```cpp
std::vector<std::tuple<double, double, double>> get_path() {
    return {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},
        {5.0, 5.0, 5.0},
    };
}
```

### Python Example
```python
def get_path():
    return [
        (0.0, 0.0, 0.0),
        (1.0, 1.0, 1.0),
        (5.0, 5.0, 5.0),
    ]
```

### TypeScript Example
```typescript
export function getPath(): Array<[number, number, number]> {
  return [
    [0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0],
    [5.0, 5.0, 5.0],
  ];
}
```

---

## 🚀 Development Workflow

### Running Standalone

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
npm run dev:vite

# App available at:
# http://localhost:5175/app.html?activity=univarm-starter
```

### Running in NAVΛ Workspace

1. Start NAVΛ Studio IDE:
   ```bash
   npm run dev:vite
   ```

2. Open workspace:
   ```
   http://localhost:5175/workspace.html
   ```

3. Click the ⚡ icon in the dock

---

## 🎯 Use Cases

### 1. Rapid Prototyping
- Quickly test path optimization algorithms
- Export code to various languages for testing
- Iterate on solver parameters

### 2. Education & Learning
- Learn navigation calculus concepts
- Understand path optimization
- See how ⋋ (lambda) operations work

### 3. Production Integration
- Replace mock engine with real NAVΛ solver
- Generate production code in your preferred language
- Integrate into larger robotics systems

### 4. Code Generation Pipeline
- Use as part of automated code generation workflow
- Export to multiple targets simultaneously
- Version control generated code

---

## 📚 API Reference

### Solver API

```typescript
interface Vec3 {
  x: number;
  y: number;
  z: number;
}

function solveOptimalPath(params: {
  start: Vec3;
  goal: Vec3;
  samples?: number;
}): Promise<{
  points: Vec3[];
  cost: number;
}>;
```

### Code Generation API

```typescript
type Target = 'rust' | 'cpp' | 'python' | 'typescript';

function emitPath(
  points: Vec3[],
  target: Target
): string;
```

---

## 🔍 Troubleshooting

### App Not Loading

**Issue**: Univarm Starter doesn't load when clicking dock icon

**Solution**:
1. Check that services are running
2. Navigate to: http://localhost:5175/app.html?activity=univarm-starter
3. Check browser console for errors

### Solver Not Working

**Issue**: "Find optimal path" button doesn't return results

**Solution**:
1. Verify start/goal coordinates are valid numbers
2. Check that mock engine is imported correctly
3. Open browser DevTools → Console for error messages

### Code Generation Empty

**Issue**: Export buttons don't generate code

**Solution**:
1. Run "Find optimal path" first to generate points
2. Ensure solver completed successfully
3. Check that target language is supported

---

## 🎉 What's Next?

### Immediate Enhancements
1. **Replace mock engine** with real NAVΛ solver
2. **Add visualization** of path points (3D canvas)
3. **Enhance UI** with charts and graphs
4. **Add more export formats** (Go, Java, etc.)

### Future Features
- Real-time 3D path visualization
- Multiple solver algorithms
- Obstacle avoidance visualization
- Path comparison tools
- Performance metrics dashboard

---

## 📖 Related Documentation

- [ROBOTIS Integration](./ROBOTIS_INTEGRATION_COMPLETE.md) - Full ROBOTIS-SYSTEMIC integration
- [⭐ Start Here](./⭐_START_HERE_ROBOTIS.md) - Quick reference guide
- [Integration Status](./INTEGRATION_STATUS.md) - Integration checklist

---

## 💡 Tips & Best Practices

1. **Start Simple**: Use the mock engine first to understand the workflow
2. **Test Parameters**: Try different start/goal coordinates
3. **Export Early**: Generate code for your target language early in development
4. **Version Control**: Save generated code for reference
5. **Iterate**: Adjust solver parameters for optimal results

---

## ✅ Integration Checklist

- [x] ✅ Univarm Starter files copied to NAVΛ Studio
- [x] ✅ App component created with full UI
- [x] ✅ Manifest defined with metadata
- [x] ✅ Dock icon added (⚡ lightning bolt)
- [x] ✅ Route registered (`/app.html?activity=univarm-starter`)
- [x] ✅ Styles implemented
- [x] ✅ Documentation complete

---

**🎊 Univarm Starter is ready to use! Click the ⚡ icon in the dock to get started!**

For questions or issues, see the troubleshooting section above or check the related documentation.

