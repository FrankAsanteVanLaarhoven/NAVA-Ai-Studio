# 🚀 NAVΛ Studio IDE - Terminal Quick Start

## ✨ Your IDE is Now Fully Functional!

NAVΛ Studio IDE now has a **complete integrated terminal** with all the capabilities of VS Code and Cursor!

---

## 🎯 **Quick Access**

### Open the IDE
```
http://localhost:5173/app.html
```

### Open Terminal
- **Activity Bar**: Click terminal icon (⚡)
- **Keyboard**: Press `Ctrl/Cmd + J`
- **Or**: Press `Ctrl/Cmd + `` (backtick)

---

## ⚡ **Terminal Features**

### ✅ **What You Can Do**

#### 1. Run Any Command
```bash
npm install
npm run dev
npm test
node script.js
python main.py
git status
cargo build
```

#### 2. Full Command History
- **↑ Arrow** - Previous command
- **↓ Arrow** - Next command
- Stores last 100 commands

#### 3. Multiple Terminals
- **Split Terminal** - Work on multiple tasks
- **New Terminal** - Create additional terminals
- **Switch Terminals** - Tab between them

#### 4. Smart Auto-Complete
- **Tab** - Auto-complete commands
- **Tab Tab** - Show all options

#### 5. Terminal Control
- **Ctrl+C** - Cancel running command
- **Ctrl+L** - Clear screen
- **Ctrl+D** - Close terminal

---

## 📝 **Common Commands**

### Development
```bash
# Start development server
npm run dev

# Install packages
npm install package-name

# Run tests
npm test

# Build project
npm run build

# Run Node.js
node app.js

# Run Python
python script.py

# Execute TypeScript
npx ts-node script.ts
```

### Git Version Control
```bash
# Initialize repository
git init

# Check status
git status

# Add files
git add .

# Commit changes
git commit -m "Your message"

# Push changes
git push origin main

# Pull latest
git pull

# View history
git log
```

### NAVΛ Specific
```bash
# Compile VNC code
compile

# Run project
run

# Run tests
test

# Build project
build

# VNC compiler
vnc --help
```

### System Information
```bash
# System info
system-info

# Current directory
pwd

# List files
ls

# Change directory
cd folder-name

# Echo text
echo "Hello World"
```

---

## 🎨 **Terminal Customization**

### Theme
The terminal uses the NAVΛ theme:
- **Background**: Dark blue (#0a0f1e)
- **Text**: Light gray (#e0e0e0)
- **Cursor**: Neon green (#00ff00)
- **Prompt**: Green ⋋ symbol

### Font
- **Family**: Menlo, Monaco, Courier New
- **Size**: 14px
- **Style**: Monospace

---

## 💡 **Pro Tips**

### Productivity
1. **Use ↑/↓** for command history
2. **Ctrl+L** to clear terminal quickly
3. **Split terminal** for parallel tasks
4. **Tab** for auto-completion
5. **Ctrl+C** to cancel any command

### Workflow
```bash
# Typical workflow
npm install        # 1. Install dependencies
npm run dev        # 2. Start dev server
# Edit code in editor
npm test           # 3. Run tests
git add .          # 4. Stage changes
git commit -m "msg" # 5. Commit
git push           # 6. Push to remote
```

### Multi-tasking
1. **Terminal 1**: Development server running
2. **Terminal 2**: Git commands
3. **Terminal 3**: Testing/building
4. **Editor**: Writing code

---

## 🔧 **Advanced Usage**

### Background Processes
```bash
# Run in background (add &)
npm run dev &

# Check running processes
ps aux | grep node

# Kill process
kill <pid>
```

### Chaining Commands
```bash
# Run sequentially
npm install && npm test && npm build

# Run regardless of errors
npm test; npm build
```

### Environment Variables
```bash
# Set variable
export API_KEY=your-key-here

# Use variable
echo $API_KEY

# Run with variable
API_KEY=test npm run dev
```

---

## 🐛 **Troubleshooting**

### Terminal Not Opening?
1. Check browser console for errors
2. Refresh page (F5)
3. Clear browser cache
4. Try different browser

### Commands Not Working?
1. Check if command is installed
2. Verify you're in correct directory (pwd)
3. Check command spelling
4. Type `help` for available commands

### Terminal Frozen?
1. Press `Ctrl+C` to cancel
2. Press `Ctrl+L` to clear
3. Close and reopen terminal

---

## 📊 **Comparison**

| Feature | VS Code | Cursor | NAVΛ Studio |
|---------|---------|--------|-------------|
| Terminal | ✅ | ✅ | ✅ |
| Command History | ✅ | ✅ | ✅ |
| Multi-terminal | ✅ | ✅ | ✅ |
| Split Terminal | ✅ | ✅ | ✅ |
| Auto-complete | ✅ | ✅ | ✅ |
| Web-based | ❌ | ❌ | ✅ |
| VNC Commands | ❌ | ❌ | ✅ |
| ROS Commands | ❌ | ❌ | ✅ |

---

## 🎓 **Getting Started**

### 1. Open IDE
```
http://localhost:5173/app.html
```

### 2. Open Terminal
Press `Ctrl/Cmd + J` or click terminal icon

### 3. Install Dependencies
```bash
npm install
```

### 4. Start Development
```bash
npm run dev
```

### 5. Start Coding!
- Write code in editor
- Test in terminal
- Commit with git
- Deploy when ready

---

## 📚 **Learn More**

- [Full IDE Capabilities](./FULL_IDE_CAPABILITIES.md)
- [Monaco Editor Guide](./MONACO_EDITOR_GUIDE.md)
- [VNC Language Guide](./VNC_LANGUAGE_GUIDE.md)
- [Git Integration Guide](./GIT_INTEGRATION_GUIDE.md)

---

## ✨ **Summary**

Your NAVΛ Studio IDE now has:

✅ **Full terminal** with xterm.js  
✅ **Command execution** (npm, node, python, git, etc.)  
✅ **Command history** (↑/↓ arrows)  
✅ **Multiple terminals** & split support  
✅ **Auto-completion** (Tab key)  
✅ **Terminal control** (Ctrl+C, Ctrl+L)  
✅ **NAVΛ commands** (compile, run, test)  
✅ **Git integration** (full version control)  
✅ **Customizable** (themes, fonts, settings)  
✅ **Web-based** (works anywhere!)  

---

**Your IDE is now as powerful as VS Code and Cursor!** 🚀

**Get started**: Open `http://localhost:5173/app.html` and press `Ctrl/Cmd + J` to open the terminal!

---

*NAVΛ Studio IDE - Where Your Navigation Calculus Career Happens* ⋋

