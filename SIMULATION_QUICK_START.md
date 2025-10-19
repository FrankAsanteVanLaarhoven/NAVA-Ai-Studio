# 🚀 NAVΛ Simulation - Quick Start

## ⚡ 60-Second Setup

### 1. Start Backend (Terminal 1)
```bash
./start-simulation.sh
```
Wait for: `🌐 Starting simulation API server on http://localhost:3030`

### 2. Start Frontend (Terminal 2)
```bash
npm run dev
```
Open: http://localhost:5173

### 3. Open Simulation
Click: **🎮 Simulation** icon (5th in left sidebar)

---

## 🎮 Basic Usage

### Add Your First Robot
1. Click **"🌍 World Manager"** tab
2. Enter name: `"MyRobot"`
3. Select: **Differential Drive**
4. Click **"➕ Add Robot"**

### Control the Robot
1. Click **"🎮 Simulation"** tab
2. Click **"▶ Start"** button
3. Click robot to select it
4. Switch to **"🎛️ Robot Control"** tab
5. Click **"⬆️ Forward"**
6. Watch it move!

### Add Obstacles
1. Go to **"🌍 World Manager"** tab
2. Scroll to **"📦 Add Obstacle"**
3. Enter name, select shape, set position
4. Click **"➕ Add Obstacle"**

---

## 🎯 Quick Commands

### Simulation Controls
| Button | Action |
|--------|--------|
| ▶ Start | Begin simulation |
| ⏸ Pause | Pause simulation |
| 🔄 Reset | Clear all & restart |

### Robot Movement
| Button | Effect |
|--------|--------|
| ⬆️ Forward | Move forward at 1 m/s |
| ⬇️ Backward | Move backward at 1 m/s |
| ⬅️ Turn Left | Rotate left at 1 rad/s |
| ➡️ Turn Right | Rotate right at 1 rad/s |
| 🛑 Stop | Emergency stop |

### Camera Views
| View | Angle |
|------|-------|
| 3D View | Isometric |
| Top View | Overhead |
| Side View | Profile |

---

## 🔧 Troubleshooting

### "🔴 Disconnected" Error
```bash
# Check if backend is running
curl http://localhost:3030/api/state

# If not, restart:
./start-simulation.sh
```

### Robot Won't Move
1. ✅ Check simulation is started (green "Running")
2. ✅ Check robot is selected (green border)
3. ✅ Check velocity is not zero
4. ✅ Click "Apply Control" button

### Build Errors
```bash
# Update Rust
rustup update

# Clean and rebuild
cd simulation-platform
cargo clean
cargo build --release
```

---

## 📖 Full Documentation
See `SIMULATION_GUIDE.md` for complete manual

---

## 🎉 That's It!
You're ready to simulate! 🤖✨

