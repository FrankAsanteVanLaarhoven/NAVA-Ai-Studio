# NAVΛ Studio IDE - Quick Reference Card

## 🎯 Quick Start

```bash
npm start                    # Start development server
Open http://localhost:3000   # Access IDE
```

## ⌨️ Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+S` / `Cmd+S` | Save file |
| `Ctrl+N` | New file |
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Project Manager |
| `Alt+L` | Insert ⋋ |
| `Alt+Shift+T` | Insert ⊗⋋ |
| `Alt+Shift+S` | Insert ⊕⋋ |
| `Alt+Shift+M` | Insert 𝒩ℐ |
| `Alt+Shift+E` | Insert ℰ |
| `Ctrl+Space` | Code completion |
| `Ctrl+/` | Toggle comment |
| `Ctrl+F` | Find |
| `Ctrl+H` | Replace |

## 🔤 NAVΛ Symbols

| Symbol | Name | Shortcut | Usage |
|--------|------|----------|-------|
| ⋋ | Lambda Navigation | `Alt+L` | `nav f(x) ⋋ y` |
| ⊗⋋ | Tensor Product | `Alt+Shift+T` | `a ⊗⋋ b` |
| ⊕⋋ | Navigation Sum | `Alt+Shift+S` | `a ⊕⋋ b` |
| ∪⋋ | Union | Click palette | `a ∪⋋ b` |
| ∩⋋ | Intersection | Click palette | `a ∩⋋ b` |
| ∇⋋ | Gradient | Click palette | `∇⋋(f)` |
| 𝒩ℐ | Master Operator | `Alt+Shift+M` | `𝒩ℐ(path)` |
| ℰ | Evolution | `Alt+Shift+E` | `ℰ(state)` |

## 📝 NAVΛ Keywords

```navlambda
nav          // Define navigation operator
tensor       // Tensor product
sum          // Sum type
union        // Union operation
intersection // Intersection operation
gradient     // Gradient operator
master       // Master operator
evolution    // Evolution operator
compose      // Compose operators
parallel     // Parallel composition
sequential   // Sequential composition
feedback     // Feedback loop
loop         // Loop construct
if/then/else // Conditional
let/in       // Variable binding
type         // Type definition
module       // Module definition
import       // Import module
export       // Export definition
```

## 🎨 Code Snippets

### Basic Navigation
```navlambda
nav pathfinder(start: State, goal: State) -> Navigation {
  let distance = ∇⋋(start, goal)
  let path = 𝒩ℐ(distance)
  compose(tensor(start ⊗⋋ path), evolution(ℰ(path)))
}
```

### Tensor Product
```navlambda
nav tensorOp(a: Navigation, b: Navigation) -> Navigation {
  a ⊗⋋ b
}
```

### Navigation Sum
```navlambda
nav sumOp(a: Navigation, b: Navigation) -> Navigation {
  a ⊕⋋ b
}
```

### Composition
```navlambda
nav composed(f: Navigation, g: Navigation) -> Navigation {
  compose(f, g)
}
```

### Parallel Execution
```navlambda
nav parallelOp(ops: [Navigation]) -> Navigation {
  parallel(ops)
}
```

## 📁 File Operations

### Create File
1. Right-click in File Explorer
2. Select "New File"
3. Enter name: `example.navλ`
4. Press Enter

### Open File
1. Click file in File Explorer
2. Or use `Ctrl+O`

### Save File
1. Press `Ctrl+S`
2. Or click Save button

### Delete File
1. Right-click file
2. Select "Delete"
3. Confirm

## 🗂️ Project Operations

### Create Project
1. Click "Project Manager"
2. Enter project name
3. Click "Create Project"

### Open Project
1. Click "Project Manager"
2. Click "Open Folder"
3. Select directory

### Recent Files
1. Click "Project Manager"
2. View "Recent Files" section
3. Click to open

## 🎯 Context Menu

Right-click in File Explorer:
- **New File** - Create new file
- **New Folder** - Create new folder
- **Rename** - Rename item
- **Delete** - Delete item
- **Copy Path** - Copy file path
- **Reveal in Explorer** - Show in system explorer

## 🔍 Code Completion

1. Start typing keyword: `nav`
2. Press `Ctrl+Space`
3. Select from dropdown
4. Press `Enter` to insert

## 💡 Tips & Tricks

### Productivity
- Use keyboard shortcuts for speed
- Recent files for quick access
- Symbol palette for operators
- Code completion for syntax

### Best Practices
- Save frequently (`Ctrl+S`)
- Use meaningful file names
- Organize in folders
- Comment your code

### Performance
- Close unused files
- Use minimap for navigation
- Fold large code blocks
- Use search for large files

## 🐛 Troubleshooting

### Editor Not Loading
```bash
# Clear cache and restart
Ctrl+Shift+R  # Hard refresh
```

### Syntax Highlighting Not Working
- Check file extension: `.navλ` or `.navlambda`
- Reload editor
- Check console for errors

### Files Not Saving
- Check browser storage quota
- Verify file path
- Check permissions

### Context Menu Not Appearing
- Try different element
- Check z-index conflicts
- Reload page

## 📚 File Extensions

| Extension | Description |
|-----------|-------------|
| `.navλ` | Primary NAVΛ files |
| `.navlambda` | Alternative NAVΛ files |
| `.vnc` | Van Laarhoven Navigation Calculus |

## 🎨 Theme Colors

| Element | Color | Hex |
|---------|-------|-----|
| Background | Dark | `#1e1e1e` |
| Foreground | Light Gray | `#d4d4d4` |
| Keywords | Blue | `#569cd6` |
| Operators | Green | `#00ff00` |
| Strings | Orange | `#ce9178` |
| Comments | Green | `#6a9955` |

## 📖 Documentation

- **IDE_ENHANCEMENTS.md** - Complete feature documentation
- **IDE_INTEGRATION_GUIDE.md** - Integration instructions
- **IDE_ENHANCEMENT_SUMMARY.md** - Project summary

## 🔗 Resources

- Monaco Editor: https://microsoft.github.io/monaco-editor/
- NAVΛ Documentation: [Your docs]
- GitHub Repository: [Your repo]
- Community Discord: [Your discord]

## 🆘 Support

- Check documentation first
- Search GitHub issues
- Ask in Discord
- Create new issue

---

**Print this card for quick reference! 📄**

**Happy coding with NAVΛ Studio! 🚀**
