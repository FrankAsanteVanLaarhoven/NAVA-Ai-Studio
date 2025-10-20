# 🚀 NAVΛ Studio Startup Guide

## Quick Start (2 Steps)

### Step 1: Start the Development Server

```bash
npm run dev
```

This will start the Vite development server at `http://localhost:5173/`

### Step 2: Access the Application

The development server will automatically open your browser to the workspace. If not, navigate to:

- **Workspace (Main Entry):** `http://localhost:5173/workspace.html`
- **Full IDE Application:** `http://localhost:5173/app.html`
- **Documentation:** `http://localhost:5173/docs.html`
- **Downloads:** `http://localhost:5173/download.html`

## 📱 Application Structure

```
┌─────────────────────────────────────────┐
│  index.html (redirects to workspace)   │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│         workspace.html                  │
│  ┌─────────────────────────────────┐   │
│  │  🏠 Home (Current Page)         │   │
│  │  🦾 ROS Learning → /app.html    │   │
│  │  ⋋  Full IDE App → /app.html    │   │
│  │  📥 Downloads → /download.html   │   │
│  │  📚 Docs → /docs.html           │   │
│  │  💻 Terminal (Modal)            │   │
│  │  🤖 AI Assistant (Modal)        │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

## ⚡ Key Features Available

### From Workspace
- ✅ Live video ad player
- ✅ Application launcher
- ✅ Terminal modal (click 💻)
- ✅ Documentation modal (click 📚)
- ✅ AI Assistant (click 🤖)
- ✅ Browser selector
- ✅ Draggable branding elements

### Full IDE Application (/app.html)
- ✅ Monaco Code Editor
- ✅ File Explorer
- ✅ Terminal Integration
- ✅ ROS Learning Center
- ✅ 3D Visualization
- ✅ Multi-Target Compilation
- ✅ Source Control
- ✅ Debugging Tools

### Documentation (/docs.html)
- ✅ Complete API reference
- ✅ Quick start guide
- ✅ VNC language documentation
- ✅ ROS integration docs
- ✅ Keyboard shortcuts
- ✅ Code examples

## 🐛 Troubleshooting

### Issue: "This site can't be reached" or ERR_FAILED

**Solution:** Make sure the development server is running!

```bash
# In your terminal, run:
npm run dev

# You should see:
#   VITE v5.x.x  ready in xxx ms
#   ➜  Local:   http://localhost:5173/
```

### Issue: Port 5173 is already in use

**Solution:** Either kill the existing process or use a different port:

```bash
# Kill existing process (macOS/Linux)
lsof -ti:5173 | xargs kill -9

# Or use a different port
npm run dev -- --port 3000
```

### Issue: Changes not reflecting

**Solution:** Hard refresh your browser:
- **Chrome/Edge:** `Ctrl+Shift+R` (Windows/Linux) or `Cmd+Shift+R` (Mac)
- **Firefox:** `Ctrl+F5` (Windows/Linux) or `Cmd+Shift+R` (Mac)

## 📝 Available Scripts

| Command | Description |
|---------|-------------|
| `npm run dev` | Start development server (web) |
| `npm run build` | Build for production (web) |
| `npm run preview` | Preview production build |
| `npm run tauri:dev` | Start Tauri development mode (desktop) |
| `npm run tauri:build` | Build Tauri application (desktop) |

## 🎯 Development Workflow

1. **Start the server:** `npm run dev`
2. **Edit files:** Changes hot-reload automatically
3. **Test features:** Navigate between pages using the dock icons
4. **Build for production:** `npm run build` when ready to deploy

## 🔗 Key URLs (When Dev Server is Running)

- Workspace: http://localhost:5173/workspace.html
- Full IDE: http://localhost:5173/app.html
- Documentation: http://localhost:5173/docs.html
- Downloads: http://localhost:5173/download.html

## 💡 Pro Tips

1. **Workspace is your hub** - Start here and navigate to other apps
2. **Terminal is everywhere** - Click 💻 from anywhere to access the terminal
3. **Documentation is local** - Click 📚 for offline docs
4. **Use keyboard shortcuts** - See docs for full list

## 🆘 Need Help?

- 📧 Email: support@navlambda.studio
- 🐛 Issues: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- 💬 Discord: https://discord.gg/navlambda

---

**Happy Coding! 🚀⋋**

