# 🚀 How to Create Projects in NAVΛ Studio IDE

## Your IDE is Running! Let's Create Projects!

---

## 🎯 **METHOD 1: Use the AI Assistant** (Easiest - Available NOW!)

### **Step 1: Look at the RIGHT panel**
You can see: **"NAVΛ AI Assistant"**

### **Step 2: Type your project idea**

In the chat input at the bottom, type prompts like:

**Example 1: Simple Project**
```
Create a REST API for managing navigation waypoints with:
- CRUD operations
- PostgreSQL database
- Docker deployment
```

**Example 2: AeroNav Fleet**
```
Generate AeroNav Fleet Command project:
- Autonomous drone fleet management
- 7D navigation with Van Laarhoven A⋋
- Real-time 3D visualization with Three.js
- WebSocket telemetry at 30 FPS
- Formation flying using Goldbach primes
- Docker + Kubernetes deployment
- GitHub Actions CI/CD
```

**Example 3: UI Component**
```
Create a 3D navigation visualizer component that:
- Renders paths with energy gradients
- Shows real-time drone positions
- Displays formation patterns
- Uses VNC for calculations
```

### **Step 3: AI Generates Code**
The AI will:
- Analyze your requirements
- Generate VNC code
- Provide implementation guidance
- Show you file structure

### **Step 4: Copy code to Editor or Notebook**
- Copy AI-generated code
- Paste into Monaco editor (center) OR
- Paste into notebook cells (bottom)

---

## 🎯 **METHOD 2: Use the Multilanguage Notebook** (Interactive!)

### **Step 1: Open the Notebook**
**Look at the BOTTOM** of your screen - the notebook should be visible!

If not visible:
- Click the **📖 Book icon** in toolbar (top-right)
- Or press **Ctrl + J** (Cmd + J on Mac)

### **Step 2: Create Multi-Language Project**

**Cell 1 - Project Plan** (Markdown):
```markdown
# AeroNav Fleet Command

## Features
- 7D Navigation (t, x, y, z, G, I, C)
- Real-time 3D visualization
- Formation flying
- WebSocket telemetry

## Tech Stack
- VNC for navigation
- Three.js for 3D
- WebSocket for real-time
- PostgreSQL for storage
```

**Cell 2 - VNC Navigation Code**:
1. Click **"+ Code"** dropdown
2. Select **"⋋ NAVΛ (VNC)"**
3. Write:
```vnc
// Van Laarhoven A⋋ Pathfinding
class DroneNavigator⋋ {
  fn find_path⋋(start: Position7D⋋, goal: Position7D⋋) -> Path⋋ {
    let pathfinder = VanLaarhovenAStar⋋::new()
    return pathfinder.find_path⋋(start, goal)
  }
}
```
4. Press **Shift + Enter** to run!

**Cell 3 - Python Data Processing**:
1. Click **"+ Code"** → Select **"🐍 Python"**
2. Write:
```python
import numpy as np

# Generate drone waypoints
waypoints = np.random.rand(10, 3) * 100
print(f"Generated {len(waypoints)} waypoints")
```
3. Press **Shift + Enter**!

**Cell 4 - Rust Performance Code**:
1. Click **"+ Code"** → Select **"🦀 Rust"**
2. Write high-performance code
3. Run it!

---

## 🎯 **METHOD 3: Use Code Generator Directly** (Advanced!)

### **Open Browser Console**
Press **F12** or **Cmd + Option + I**

### **Run This Code:**

```javascript
// Import the code generator
const { mcpCodeGenerator } = await import('/src/services/mcp-code-generator.ts');

// Generate a project
const project = await mcpCodeGenerator.generateProject({
  prompt: "Create a drone fleet management system with 3D visualization and real-time updates",
  projectName: "drone-fleet",
  includeDeployment: true,
  includeTests: true
});

// See what was generated
console.log('Generated files:');
project.files.forEach(f => {
  console.log(`📄 ${f.path} - ${f.description}`);
  console.log(f.content.substring(0, 200) + '...\n');
});

// Save the README
console.log('README:');
console.log(project.readme);
```

---

## 🎯 **METHOD 4: Use File Explorer** (Manual!)

### **Step 1: Create New File**
1. **Right-click** in the Explorer (left sidebar)
2. **Select** "New File"
3. **Name it:** `my-project.vnc`

### **Step 2: Write VNC Code**
Use the templates I created! Here's a quick one:

**For REST API:**
```vnc
@route("/api/drones")
@method(GET)
fn get_drones⋋() -> Response⋋ {
  let drones = Database⋋::query("SELECT * FROM drones")
  return json_response⋋(200, drones)
}
```

**For UI Component:**
```vnc
class DroneStatus⋋ {
  state: {
    drones: Array<Drone⋋>,
    selected: Drone⋋?
  }
  
  fn render⋋() -> UI⋋ {
    return ui⋋ {
      <div class="drone-status">
        {this.state.drones.map⋋(|drone| {
          <DroneCard⋋ drone={drone} />
        })}
      </div>
    }
  }
}
```

**For Navigation:**
```vnc
let path = navigate⋋(
  Position7D⋋(0, 0, 0, 0, 0, 1.0, 1.0, 1.0),
  Position7D⋋(0, 10, 10, 5, 0, 1.0, 1.0, 1.0)
)

let optimized = path.optimize_energy⋋()
visualize⋋(optimized)
```

### **Step 3: Compile**
Click **"⚙ Compile"** button in toolbar!

### **Step 4: Run**
Click **"▶ Run"** button!

### **Step 5: Visualize**
Click **"👁 Visualize"** button to see 3D!

---

## 🌟 **RECOMMENDED WORKFLOW FOR AERONAV:**

### **Quick Start (5 minutes):**

1. **Ask AI Assistant** (right panel):
```
Generate the core navigation module for AeroNav with:
- Van Laarhoven A⋋ pathfinding
- 7D position tracking
- Energy optimization
- Multi-drone coordination
```

2. **Copy AI response** to notebook cell

3. **Run the code** (Shift + Enter)

4. **Iterate** - Ask AI to enhance/fix/optimize

5. **When ready** - Click **Compile** → **Run** → **Visualize**!

---

### **Full Project (Using Console):**

```javascript
// In browser console (F12):

// 1. Generate complete AeroNav project
const aeronav = await (await import('/src/services/project-scaffolder.ts')).projectScaffolder.createProject({
  name: 'aeronav-fleet-command',
  description: 'Autonomous drone fleet with VNC navigation',
  type: 'fullstack',
  features: ['navigation', 'ui', 'database', '3d-viz', 'realtime', 'multi-agent'],
  deploymentTargets: ['docker', 'kubernetes'],
  author: 'Frank Van Laarhoven',
  license: 'MIT'
});

// 2. See generated files
aeronav.files.forEach(f => {
  console.log(`✅ ${f.path}`);
});

// 3. Save files (you can copy-paste to create them)
aeronav.files.forEach(f => {
  console.log(`\n=== ${f.path} ===`);
  console.log(f.content);
});
```

---

## 🎮 **QUICK ACTIONS IN YOUR IDE:**

### **What You Can Do RIGHT NOW:**

#### **1. Explore Example Files**
- **Left sidebar** → Expand "examples" folder
- **Click** `basic-navig...`
- **See** example VNC code
- **Learn** from examples!

#### **2. Write Code in Editor**
- **Center panel** → Monaco editor
- **Type** VNC code
- **Get** IntelliSense suggestions
- **Use** ⋋ symbol (see buttons at top!)

#### **3. Use Notebook for Experiments**
- **Bottom panel** → Notebook
- **Add cells** for each component
- **Mix languages** (VNC + Python + Rust)
- **Run interactively**!

#### **4. Visualize Navigation**
- **Write** navigation code
- **Click** "👁 Visualize"
- **See** 3D path rendering on right!

#### **5. Check Navigation Metrics**
- **Right panel** → "NAVIGATION METRICS"
- **See** Active Paths, Total Energy
- **Monitor** optimization status!

---

## 🌟 **FOR AERONAV SPECIFICALLY:**

### **Phase 1: Start with AI Chat**

**Type in AI Assistant:**
```
I want to create AeroNav Fleet Command. 
Let's start with the core navigation module. 
Generate VNC code for:
1. Position7D⋋ type definition
2. DroneNavigator⋋ class
3. FleetCoordinator⋋ for multi-agent
4. Goldbach formation algorithm

Show me the code for the first file.
```

### **Phase 2: Build Incrementally**

**After AI gives you code:**
1. Copy to notebook cell
2. Test it
3. Ask AI for next component
4. Repeat!

### **Phase 3: Use Templates**

**Ask AI:**
```
Using the VNC templates in src/templates/vnc-templates.ts,
generate a complete REST API for drone fleet management.
```

---

## 🎊 **YOU'RE READY TO CREATE!**

### **Start Creating NOW:**

**Option A: Chat with AI** (Right panel)  
**Option B: Code in Notebook** (Bottom panel)  
**Option C: Write in Editor** (Center panel)  
**Option D: Use Console** (F12, run scaffolder code)  

---

## 📚 **HELPFUL RESOURCES:**

**Generated Services:**
- `src/services/mcp-code-generator.ts` - Natural language → VNC
- `src/templates/vnc-templates.ts` - 7 professional templates
- `src/services/project-scaffolder.ts` - Complete project creation

**Documentation:**
- `PLATFORM_PREPARATION_STATUS.md` - What's implemented
- `SIMULATION_GUIDE.md` - Simulation platform
- `NIF_INTEGRATION_COMPLETE.md` - Dataset integration

---

## 🚀 **LET'S CREATE AERONAV!**

**Start by asking the AI Assistant on the right:**

```
Generate the AeroNav Fleet Command project structure and core navigation module
```

**The AI will guide you through creating the entire project!** 🌟✨

**READY TO START? Type in the AI chat now!** 🚀


