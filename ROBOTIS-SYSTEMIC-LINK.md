# 🔗 ROBOTIS-SYSTEMIC Integration Link

This file serves as a reference link to the integrated ROBOTIS-SYSTEMIC platform.

## Location

**ROBOTIS-SYSTEMIC** is located at:
```
/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC
```

## What is ROBOTIS-SYSTEMIC?

**Univarm** - A Fortune-500/IPO-grade robotics control and trust platform featuring:
- Real-time robot control with WebXR interface
- Trust and attestation system
- Digital twin support (Kobuki/TurtleBot2)
- Enterprise security (RBAC, audit trails, Ed25519 signatures)
- Cloud-native deployment (Kubernetes, Docker)

## Quick Access

### Start Integrated System
```bash
./start-robotis-system.sh
```

### Access Points
- **Integrated View**: Click "Univarm" (🔷) in the dock at http://localhost:5175/workspace.html
- **Direct Access**: http://localhost:3000 (when services are running)
- **Backend API**: http://localhost:8080/api

### Documentation
- [Complete Integration Guide](./ROBOTIS_INTEGRATION_COMPLETE.md)
- [Integration Status](./INTEGRATION_STATUS.md)
- [Original Integration Docs](./UNIVARM_INTEGRATION_README.md)
- [ROBOTIS-SYSTEMIC README](/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/README.md)
- [ROBOTIS-SYSTEMIC RUNME](/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/RUNME.md)

## Key Components

### Backend Service (robotd)
- **Path**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/services/robotd`
- **Language**: Rust
- **Port**: 8080
- **Features**: REST API, SSE, robot control, trust system

### Frontend App (Univarm Web)
- **Path**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/apps/web`
- **Framework**: Next.js
- **Port**: 3000
- **Features**: XR Bench, presets, actions, live charts

### Robot Assets
- **URDF Models**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/assets/urdf/`
- **Presets**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/apps/web/public/presets/`
- **Scenes**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/data/scenes/`

### Deployment Configs
- **Docker**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/docker-compose.*.yml`
- **Kubernetes**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/deploy/helm/`
- **Terraform**: `/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/infra/terraform/`

## Integration Architecture

```
NAVΛ STUDIO IDE (this workspace)
├── Vite Dev Server (5175) ──┐
│                             │
├── Proxy: /univarm ──────────┼──→ Univarm Web (3000)
├── Proxy: /api ──────────────┼──→ robotd (8080)
│                             │
└── OSDesktop.tsx ────────────┘
    └── Univarm Dock Icon
        └── Opens UnivarmApp.tsx
            └── iframe to /univarm
```

## Maintenance

### Update ROBOTIS-SYSTEMIC
```bash
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC
git pull origin main  # If using git
```

### Rebuild Services
```bash
# Backend
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/services/robotd
cargo build --release

# Frontend
cd /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/apps/web
npm install
npm run build
```

## Support

For integration issues, see:
- [Integration Troubleshooting](./ROBOTIS_INTEGRATION_COMPLETE.md#troubleshooting)
- [Verification Script](./verify-integration.sh)

For ROBOTIS-SYSTEMIC specific issues:
- [ROBOTIS Documentation](../ROBOTIS-SYSTEMIC/docs/)
- [Implementation Guide](../ROBOTIS-SYSTEMIC/docs/IMPLEMENTATION-GUIDE.md)

---

**This is an integrated system.** Both NAVΛ Studio IDE and ROBOTIS-SYSTEMIC work together to provide a complete robotics development platform.

