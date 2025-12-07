# 🚀 ROBOTIS-SYSTEMIC Full Integration Guide

## Executive Summary

**ROBOTIS-SYSTEMIC** (Univarm Platform) is now **FULLY INTEGRATED** with **NAVΛ Studio IDE**, creating a unified robotics development and control platform with enterprise-grade capabilities.

## What is ROBOTIS-SYSTEMIC?

**Univarm** is a Fortune-500/IPO-grade control and trust platform featuring:

- ✅ **Real-time Guarantees** - DMR/AJ/TTP with live enforcement
- ✅ **Trust & Security** - SBOM, cosign, SLSA, Rekor, signed attestations
- ✅ **VR/WebXR Control** - Command & control with haptics, minimap, command palette
- ✅ **Enterprise Features** - RBAC, signed audit (Ed25519), OIDC-ready, multi-tenant
- ✅ **Observability** - Prometheus `/metrics`, OpenTelemetry hooks
- ✅ **Cloud Native** - Kubernetes Helm charts, Gatekeeper policies, hardened containers
- ✅ **Digital Twin** - Kobuki (TurtleBot2) robot simulation and control
- ✅ **CI/CD Security** - CodeQL, cargo-audit, trivy, Renovate

## Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  NAVΛ STUDIO IDE (Port 5175)                │
├─────────────────────────────────────────────────────────────┤
│  Workspace Desktop                                          │
│  ├── Dock Integration (Univarm Icon)                       │
│  ├── Vite Proxy: /univarm → localhost:3000                 │
│  ├── Vite Proxy: /api → localhost:8080                     │
│  └── UnivarmApp React Component                            │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│              Univarm Web (Next.js - Port 3000)              │
├─────────────────────────────────────────────────────────────┤
│  Frontend Applications                                      │
│  ├── XR Bench (WebXR Control Interface)                    │
│  ├── Presets Management                                     │
│  ├── Actions Dashboard                                      │
│  ├── Command Palette (⌘K)                                   │
│  ├── Live Charts & Telemetry                               │
│  └── Trust Dashboard                                        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                robotd Backend (Rust - Port 8080)            │
├─────────────────────────────────────────────────────────────┤
│  Backend Services                                           │
│  ├── RESTful API                                            │
│  ├── Server-Sent Events (SSE) for Real-time Updates        │
│  ├── Robot Control & Path Planning                         │
│  ├── Evidence Pack Generation                              │
│  ├── Trust & Attestation                                    │
│  ├── RBAC & Audit Trail (Ed25519 signatures)               │
│  └── Prometheus Metrics                                     │
└─────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
/Users/frankvanlaarhoven/Downloads/
├── NAVΛ STUDIO IDE/                    # Main IDE Platform
│   ├── start-robotis-system.sh         # ⭐ START ALL SERVICES
│   ├── stop-robotis-system.sh          # Stop all services
│   ├── verify-integration.sh           # Verify integration status
│   ├── .env.robotis                    # Integration config
│   ├── public/univarm/                 # Univarm static assets
│   │   ├── univarm-overlay.js
│   │   ├── univarm-overlay.css
│   │   ├── register-univarm.js
│   │   ├── apps.json
│   │   └── icon.svg
│   ├── src/
│   │   ├── apps/UnivarmApp.tsx         # React integration component
│   │   └── components/Workspace/
│   │       └── OSDesktop.tsx           # Desktop with Univarm dock icon
│   ├── vite.config.ts                  # Proxy configuration
│   ├── logs/                           # Service logs (auto-created)
│   └── UNIVARM_INTEGRATION_README.md   # Integration docs
│
└── ROBOTIS-SYSTEMIC/                   # Univarm Platform
    ├── services/robotd/                # Rust backend
    │   ├── Cargo.toml
    │   ├── src/
    │   │   ├── main.rs
    │   │   ├── rest_api.rs
    │   │   ├── drive_sse.rs
    │   │   ├── ddrive.rs
    │   │   └── path_follow.rs
    │   └── README.md
    ├── apps/web/                       # Next.js frontend
    │   ├── package.json
    │   ├── .env.local                  # Auto-generated config
    │   ├── app/
    │   │   ├── xr-bench/              # WebXR interface
    │   │   ├── presets/               # Robot presets
    │   │   └── actions/               # Action queue
    │   ├── components/
    │   └── store/
    ├── assets/urdf/kobuki/            # Kobuki robot model
    ├── deploy/                        # Kubernetes/Helm configs
    ├── docs/                          # Documentation
    └── README.md
```

## Quick Start

### 1. Start All Services (One Command!)

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
chmod +x start-robotis-system.sh
./start-robotis-system.sh
```

This will:
1. ✅ Start **robotd** (Rust backend) on port 8080
2. ✅ Start **Univarm Web** (Next.js) on port 3000
3. ✅ Start **NAVΛ Studio IDE** (Vite) on port 5175
4. ✅ Open your browser to the IDE automatically

### 2. Access the Integrated Platform

**Main Access:**
- 🌐 **NAVΛ Studio IDE**: http://localhost:5175/workspace.html
- 🔷 **Univarm in IDE**: Click the "Univarm" (🔷) icon in the dock
- 📊 **Univarm Direct**: http://localhost:3000

**API Endpoints:**
- 📡 **SSE Stream**: http://localhost:8080/api/rt/subscribe?token=demo-token-123
- 🔗 **REST API**: http://localhost:8080/api
- 📊 **Metrics**: http://localhost:8080/metrics
- 🛡️ **Trust Summary**: http://localhost:8080/api/trust/summary

### 3. Stop All Services

```bash
./stop-robotis-system.sh
```

## Verification

Check that everything is properly integrated:

```bash
chmod +x verify-integration.sh
./verify-integration.sh
```

This will verify:
- ✅ Directory structure
- ✅ Integration files
- ✅ Configuration files
- ✅ Dependencies (Node.js, Rust, npm, cargo)
- ✅ Service status
- ✅ API endpoints

## Key Features

### 1. Seamless IDE Integration

- **Dock Icon**: Univarm appears as a native app icon in the NAVΛ Studio dock
- **Embedded View**: Opens as a full-screen overlay within the IDE
- **Token Handoff**: SSE authentication token automatically passed
- **Unified Origin**: All services proxied through the IDE (no CORS issues)

### 2. WebXR Robot Control

Access the **XR Bench** at http://localhost:3000/xr-bench or via the integrated view:
- 🎮 **Virtual joystick** with haptic feedback
- 🗺️ **Interactive minimap** showing robot position
- 📊 **Real-time telemetry** (DMR, AJ, TTP metrics)
- ⌨️ **Command Palette** (⌘K / Ctrl+K)
- 🎯 **Click-to-navigate** path planning

### 3. Digital Twin - Kobuki Robot

- **URDF Model**: `assets/urdf/kobuki/kobuki.urdf`
- **Preset**: Load "Kobuki (TurtleBot2) — Lab" from Presets menu
- **Diff-Drive Kinematics**: Realistic wheel-based navigation
- **ROS 2 Compatible**: Maps to `/cmd_vel` and `/odom` topics

### 4. Trust & Security Dashboard

- **Evidence Packs**: Generate cryptographic proof bundles
- **Ed25519 Signatures**: Client-verifiable audit trails
- **SBOM**: Software Bill of Materials included
- **Cosign**: Container image attestations
- **SLSA Provenance**: Supply chain security

### 5. Action Queue & RBAC

- **Server-suggested actions** via Command Palette
- **Role-based access**: Operator and Admin roles
- **Signed audit log**: Every action cryptographically signed
- **Retry/Cancel controls**: Admin-only cancellation

## Configuration

### Environment Variables

**NAVΛ Studio IDE** (`.env.robotis`):
```bash
ROBOTIS_DIR=/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC
NAVA_DIR=/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE
ROBOTD_PORT=8080
UNIVARM_WEB_PORT=3000
NAVA_IDE_PORT=5175
SSE_TOKEN=demo-token-123
```

**Univarm Web** (`apps/web/.env.local`):
```bash
NEXT_PUBLIC_API_BASE=http://localhost:8080
API_BASE=http://localhost:8080
NEXT_PUBLIC_SSE_TOKEN=demo-token-123
```

**robotd Backend** (environment):
```bash
SSE_TOKEN=demo-token-123
RUST_LOG=robotd=info,tower_http=debug
```

### Advanced Configuration

**JWT Authentication** (optional):
```bash
export SSE_JWT_HS256="your-shared-secret"
# Issue JWTs with {"role": "operator"|"admin", "exp": ...}
```

**Ed25519 Audit Signatures**:
```bash
export AUDIT_ED25519_SK=$(openssl rand -base64 32)
# Retrieve public key: curl http://localhost:8080/api/audit/pubkey
```

**RBAC Tokens**:
```bash
export ACTION_OPERATOR_TOKENS="op1,op2,op3"
export ACTION_ADMIN_TOKENS="admin1,admin2"
```

## Usage Examples

### 1. Control Robot from IDE

1. Open NAVΛ Studio: http://localhost:5175/workspace.html
2. Click **Univarm** (🔷) icon in dock
3. Navigate to **XR Bench** tab
4. Load preset: **Kobuki (TurtleBot2) — Lab**
5. Click on floor to set navigation goal
6. Click **Plan** → **Execute**

### 2. Monitor Real-Time Metrics

```bash
# SSE stream (real-time updates)
curl -N http://localhost:8080/api/rt/subscribe?token=demo-token-123

# Prometheus metrics
curl http://localhost:8080/metrics

# Trust summary
curl http://localhost:8080/api/trust/summary | jq
```

### 3. Generate Evidence Pack

Via UI:
- Press **⌘K** (or Ctrl+K)
- Select "Generate Evidence Pack"

Via API:
```bash
curl -OJ http://localhost:8080/api/rt/evidence
# Downloads: evidence-<timestamp>.zip
```

### 4. Verify Audit Trail

```bash
# Install dependencies
pip install pynacl

# Get public key
curl -s http://localhost:8080/api/audit/pubkey > pubkey.json

# Verify evidence pack
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC
python3 scripts/verify_audit.py \
  --pubkey-url http://localhost:8080/api/audit/pubkey \
  --evidence evidence.zip
```

## Development Workflow

### Working on robotd (Backend)

```bash
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/services/robotd

# Run with hot reload
cargo watch -x run

# Run tests
cargo test

# Build release
cargo build --release
```

### Working on Univarm Web (Frontend)

```bash
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/apps/web

# Development mode
npm run dev

# Build for production
npm run build
npm start

# Lint
npm run lint
```

### Working on NAVΛ Studio IDE

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"

# Development mode (Vite)
npm run dev:vite

# Development mode (Next.js)
npm run dev

# Build
npm run build

# Desktop app
npm run tauri:dev
```

## Troubleshooting

### Port Already in Use

```bash
# Kill process on specific port
lsof -ti:8080 | xargs kill -9   # robotd
lsof -ti:3000 | xargs kill -9   # Univarm Web
lsof -ti:5175 | xargs kill -9   # NAVΛ IDE
```

### Services Won't Start

1. Check dependencies:
   ```bash
   ./verify-integration.sh
   ```

2. Check logs:
   ```bash
   tail -f logs/robotd.log
   tail -f logs/univarm-web.log
   tail -f logs/nava-studio.log
   ```

3. Manual start (for debugging):
   ```bash
   # Terminal 1: robotd
   cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/services/robotd
   SSE_TOKEN=demo-token-123 cargo run
   
   # Terminal 2: Univarm Web
   cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/apps/web
   npm run dev
   
   # Terminal 3: NAVΛ IDE
   cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
   npm run dev:vite
   ```

### CORS Issues

The Vite proxy should handle CORS, but if you encounter issues:

1. Ensure robotd is running with CORS enabled (it should be by default)
2. Check `vite.config.ts` has proxy configured
3. Use the unified NGINX config (see `UNIVARM_INTEGRATION_README.md`)

### SSE Connection Fails

1. Check token is correct: `demo-token-123`
2. Verify robotd is running: `curl http://localhost:8080/api/rt/subscribe?token=demo-token-123`
3. Check browser console for errors

## Production Deployment

See `ROBOTIS-SYSTEMIC/RUNME.md` for:
- Docker Compose deployment
- Kubernetes Helm charts
- Cloudflare Tunnel setup
- mTLS configuration
- Supply chain security

## Documentation

### ROBOTIS-SYSTEMIC Documentation

- **Quick Start**: `ROBOTIS-SYSTEMIC/RUNME.md`
- **Main README**: `ROBOTIS-SYSTEMIC/README.md`
- **Deployment**: `ROBOTIS-SYSTEMIC/docs/IMPLEMENTATION-GUIDE.md`
- **API Reference**: `ROBOTIS-SYSTEMIC/openapi/openapi.yml`
- **Docusaurus Site**: `ROBOTIS-SYSTEMIC/sites/docs/`

### NAVΛ Studio IDE Documentation

- **Main README**: `README.md`
- **Quick Start**: `START_HERE.md`
- **Integration Guide**: `UNIVARM_INTEGRATION_README.md`
- **Build Instructions**: `BUILD_INSTRUCTIONS.md`

## Architecture Highlights

### Backend (robotd)

- **Language**: Rust
- **Framework**: Axum (async web framework)
- **Database**: PostgreSQL (via migrations)
- **Real-time**: Server-Sent Events (SSE)
- **Metrics**: Prometheus
- **Security**: Ed25519 signatures, HMAC, JWT

### Frontend (Univarm Web)

- **Framework**: Next.js 14
- **UI**: React 18, Tailwind CSS
- **3D**: React Three Fiber, Three.js
- **XR**: @react-three/xr
- **Charts**: Recharts
- **State**: Zustand

### Integration Layer

- **Proxy**: Vite development server
- **Protocol**: HTTP/HTTPS, WebSocket, SSE
- **Authentication**: Token-based, JWT optional
- **CORS**: Handled by proxy

## What's Next?

### Immediate Next Steps

1. ✅ **Explore the XR Bench** - Try the WebXR interface
2. ✅ **Generate Evidence Pack** - Test the trust system
3. ✅ **Load Kobuki Preset** - Run the digital twin
4. ✅ **Monitor SSE Stream** - Watch real-time metrics

### Advanced Features to Try

1. **Command Palette Actions** - Press ⌘K and explore server-suggested actions
2. **Trust Dashboard** - Check `/api/trust/summary` endpoint
3. **Action Queue** - Queue actions and see audit trail
4. **Presets** - Create custom robot configurations
5. **ROS 2 Integration** - Connect to real Kobuki/TurtleBot2

### Future Enhancements

- [ ] Multi-robot fleet management
- [ ] Cloud deployment templates
- [ ] Advanced path planning algorithms
- [ ] Integration with SLAM (gmapping, cartographer)
- [ ] Camera feed integration
- [ ] Sensor fusion (LiDAR, IMU, odometry)

## Support & Community

- **Issues**: Report at ROBOTIS-SYSTEMIC or NAVΛ Studio IDE repos
- **Documentation**: See `docs/` directories in both projects
- **Email**: support@navlambda.studio

## License

Both projects are dual-licensed:
- **MIT License**
- **Apache License 2.0**

## Credits

**ROBOTIS-SYSTEMIC (Univarm)** - Fortune-500/IPO-grade robotics control platform
**NAVΛ Studio IDE** - Van Laarhoven Navigation Calculus IDE by Frank Van Laarhoven

---

**🎉 Integration Complete! ROBOTIS-SYSTEMIC is now fully integrated with NAVΛ Studio IDE.**

Start exploring: `./start-robotis-system.sh` and open http://localhost:5175/workspace.html

