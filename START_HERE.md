# 🚀 Quick Start Guide - NAVΛ Studio IDE

## Getting Started in 3 Steps

### 1. Start the Development Server

```bash
npm run dev
```

This will start the Vite development server. You should see output like:

```
  VITE v5.x.x  ready in xxx ms

  ➜  Local:   http://localhost:5173/
  ➜  Network: use --host to expose
  ➜  press h + enter to show help
```

### 2. Open in Browser

Open your browser and navigate to:
```
http://localhost:5173/
```

### 3. Create Your First Project

1. Click the **"Project Manager"** button in the IDE
2. Enter a project name (e.g., `my-navλ-project`)
3. Click **"Create Project"**
4. Start coding with NAVΛ!

---

## Alternative: Build for Desktop

If you want to run NAVΛ Studio as a desktop application:

```bash
# Development mode (with hot reload)
npm run tauri:dev

# Production build
npm run tauri:build
```

---

## Available Scripts

| Command | Description |
|---------|-------------|
| `npm run dev` | Start Vite development server (web) |
| `npm run build` | Build for production (web) |
| `npm run preview` | Preview production build |
| `npm run tauri:dev` | Start Tauri development mode (desktop) |
| `npm run tauri:build` | Build Tauri application (desktop) |
| `npm run lint` | Run ESLint |
| `npm run format` | Format code with Prettier |
| `npm run test` | Run tests |

---

## Your First NAVΛ Program

Once the IDE is open, try this example:

```navlambda
// Define a navigation operator
nav pathfinder(start: State, goal: State) -> Navigation {
  let distance = ∇⋋(start, goal)
  let path = 𝒩ℐ(distance)
  
  compose(
    tensor(start ⊗⋋ path),
    evolution(ℰ(path)),
    feedback(goal)
  )
}

// Use the operator
nav main() {
  let start = State::new(0, 0)
  let goal = State::new(10, 10)
  
  let result = pathfinder(start, goal)
  result.execute()
}
```

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Alt+L` | Insert ⋋ (Lambda Navigation) |
| `Alt+Shift+T` | Insert ⊗⋋ (Tensor Product) |
| `Alt+Shift+S` | Insert ⊕⋋ (Navigation Sum) |
| `Alt+Shift+M` | Insert 𝒩ℐ (Master Operator) |
| `Alt+Shift+E` | Insert ℰ (Evolution) |
| `Ctrl+S` / `Cmd+S` | Save file |
| `Ctrl+O` / `Cmd+O` | Open file |
| `Ctrl+N` / `Cmd+N` | New file |

---

## Troubleshooting

### Port Already in Use

If you see an error like "Port 5173 is already in use", you can:

1. Kill the process using that port:
   ```bash
   # On macOS/Linux
   lsof -ti:5173 | xargs kill -9
   
   # On Windows
   netstat -ano | findstr :5173
   taskkill /PID <PID> /F
   ```

2. Or use a different port:
   ```bash
   npm run dev -- --port 3000
   ```

### Module Not Found

If you see module errors, try:

```bash
# Clean install
rm -rf node_modules package-lock.json
npm install
```

### Tauri Build Issues

If Tauri build fails:

```bash
# Make sure Rust is installed
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Update Rust
rustup update

# Try building again
npm run tauri:build
```

---

## Next Steps

1. ✅ **Explore the IDE** - Try the file explorer, project manager, and editor
2. ✅ **Learn NAVΛ Syntax** - Check out `QUICK_REFERENCE.md`
3. ✅ **Read Documentation** - See `IDE_ENHANCEMENTS.md` for all features
4. ✅ **Build Something** - Create your first navigation system!

---

## Need Help?

- 📖 **Documentation**: See `IDE_ENHANCEMENTS.md`
- 🎯 **Quick Reference**: See `QUICK_REFERENCE.md`
- 🏗️ **Architecture**: See `ARCHITECTURE.md`
- 📁 **File Inventory**: See `FILES_CREATED.md`

---

**Happy Coding! 🎉⋋**
