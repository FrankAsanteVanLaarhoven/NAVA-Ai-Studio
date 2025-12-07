# 🚀 Quick Start - ROBOTIS-SYSTEMIC Integration

## TL;DR - Start Everything Now!

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
./start-robotis-system.sh
```

Your browser will automatically open to: **http://localhost:5175/workspace.html**

Click the **Univarm** (🔷) icon in the dock to access the robotics control interface!

---

## What Just Happened?

You've started a **fully integrated robotics development platform** combining:

### 🔷 ROBOTIS-SYSTEMIC (Univarm)
- **robotd Backend** (Rust) - Robot control, path planning, trust system
- **Univarm Web** (Next.js) - WebXR interface, command palette, live telemetry
- **Digital Twin** - Kobuki (TurtleBot2) robot simulation

### 💻 NAVΛ Studio IDE
- **Development Environment** - Code editor, terminal, workspace
- **Seamless Integration** - Univarm embedded in the IDE
- **Unified Interface** - Single access point for all tools

---

## Access Points

Once running, access the platform at:

| What | Where | How |
|------|-------|-----|
| **Main Workspace** | http://localhost:5175/workspace.html | Opens automatically |
| **Univarm Console** | Click 🔷 in dock | Embedded in IDE |
| **XR Bench** | Univarm → XR Bench tab | Robot control interface |
| **Command Palette** | Press ⌘K or Ctrl+K | Quick actions |
| **Direct Univarm** | http://localhost:3000 | Standalone access |
| **API** | http://localhost:8080/api | REST endpoints |
| **SSE Stream** | http://localhost:8080/api/rt/subscribe?token=demo-token-123 | Real-time updates |

---

## First Steps

### 1. Open the Workspace
- Browser opens automatically to http://localhost:5175/workspace.html
- You'll see the NAVΛ Studio desktop with app icons and dock

### 2. Launch Univarm
- Click the **Univarm** (🔷) icon in the bottom dock
- Full-screen overlay opens showing the Univarm interface

### 3. Try the XR Bench
- Click **XR Bench** tab in Univarm
- Click on the floor to set a navigation goal
- Click **Plan** then **Execute** to move the robot

### 4. Open Command Palette
- Press **⌘K** (Mac) or **Ctrl+K** (Windows/Linux)
- See server-suggested actions
- Try "Generate Evidence Pack" or "Toggle collision heatmap"

### 5. Load a Robot Preset
- Go to **Presets** tab
- Select **Kobuki (TurtleBot2) — Lab**
- Watch the digital twin load

---

## Stop Everything

When you're done:

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
./stop-robotis-system.sh
```

This cleanly shuts down all services and cleans up ports.

---

## Troubleshooting

### Problem: Port already in use

**Solution:**
```bash
./stop-robotis-system.sh  # Clean shutdown
./start-robotis-system.sh # Restart
```

### Problem: Browser didn't open

**Solution:**
Manually open: http://localhost:5175/workspace.html

### Problem: Univarm shows blank screen

**Solutions:**
1. Wait a moment for services to fully start
2. Check logs: `tail -f logs/univarm-web.log`
3. Restart: `./stop-robotis-system.sh && ./start-robotis-system.sh`

### Problem: Need more help

**Resources:**
- Run verification: `./verify-integration.sh`
- Full guide: [ROBOTIS_INTEGRATION_COMPLETE.md](./ROBOTIS_INTEGRATION_COMPLETE.md)
- Status: [INTEGRATION_STATUS.md](./INTEGRATION_STATUS.md)

---

## What Can You Do?

### Robot Control
- ✅ Control virtual robot with click-to-navigate
- ✅ Real-time telemetry (speed, position, metrics)
- ✅ Path planning with collision avoidance
- ✅ Digital twin of Kobuki/TurtleBot2

### WebXR Interface
- ✅ Virtual joystick with haptic feedback
- ✅ Interactive minimap
- ✅ 3D robot visualization
- ✅ Real-time DMR/AJ/TTP metrics

### Trust & Security
- ✅ Generate cryptographic evidence packs
- ✅ Ed25519 signed audit trails
- ✅ SBOM (Software Bill of Materials)
- ✅ Supply chain attestations (cosign, SLSA)

### Development
- ✅ Edit code in NAVΛ Studio IDE
- ✅ Hot reload for frontend and backend
- ✅ Integrated terminal
- ✅ Real-time logging

### Action Queue
- ✅ Server-suggested actions via ⌘K palette
- ✅ Role-based access control (Operator/Admin)
- ✅ Audit trail with cryptographic signatures
- ✅ Retry/cancel actions

---

## Service Details

### robotd (Backend)
- **Port**: 8080
- **Language**: Rust
- **Features**: REST API, SSE, robot control, trust system
- **Logs**: `logs/robotd.log`

### Univarm Web (Frontend)
- **Port**: 3000
- **Framework**: Next.js + React
- **Features**: XR Bench, presets, charts, command palette
- **Logs**: `logs/univarm-web.log`

### NAVΛ Studio IDE
- **Port**: 5175
- **Framework**: Vite + React
- **Features**: Workspace, proxy, integration layer
- **Logs**: `logs/nava-studio.log`

---

## Learn More

### Documentation
- **[Complete Integration Guide](./ROBOTIS_INTEGRATION_COMPLETE.md)** - Full documentation
- **[Integration Status](./INTEGRATION_STATUS.md)** - Current status and checklist
- **[ROBOTIS-SYSTEMIC README](../ROBOTIS-SYSTEMIC/README.md)** - Univarm overview
- **[ROBOTIS-SYSTEMIC RUNME](../ROBOTIS-SYSTEMIC/RUNME.md)** - Detailed instructions

### Architecture
- **[Integration Architecture](./ROBOTIS_INTEGRATION_COMPLETE.md#integration-architecture)** - System design
- **[Service Architecture](./INTEGRATION_STATUS.md#service-architecture)** - Component diagram

### Advanced Topics
- **JWT Authentication** - See ROBOTIS_INTEGRATION_COMPLETE.md
- **Ed25519 Audit Signatures** - See ROBOTIS_INTEGRATION_COMPLETE.md
- **RBAC Configuration** - See ROBOTIS_INTEGRATION_COMPLETE.md
- **Cloud Deployment** - See ROBOTIS-SYSTEMIC/RUNME.md

---

## Next Steps

1. ✅ **Explore the XR Bench** - Try robot control
2. ✅ **Generate Evidence Pack** - Press ⌘K → "Generate Evidence Pack"
3. ✅ **Monitor Real-Time Data** - Watch SSE stream in browser DevTools
4. ✅ **Load Robot Presets** - Try different robot configurations
5. ✅ **Read the Docs** - Check out the complete integration guide

---

**🎉 Welcome to the integrated ROBOTIS-SYSTEMIC + NAVΛ Studio platform!**

You now have a Fortune-500/IPO-grade robotics development environment at your fingertips.

**Start coding, controlling, and creating!** 🤖🚀💻

