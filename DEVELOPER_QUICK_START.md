# NAVA Studio Developer Quick Start Guide

## Welcome to NAVA Studio! 🚀

This guide will get you up and running with the NAVA Studio robotics simulation platform in under 30 minutes.

## Prerequisites

### Required Software
- **Rust**: 1.75 or later
- **Node.js**: 20 or later
- **Git**: Latest version
- **Docker**: Latest version (for containerization)
- **CUDA**: 12.0+ (for GPU acceleration)

### Recommended Hardware
- **CPU**: 8+ cores
- **RAM**: 32GB minimum, 64GB recommended
- **GPU**: NVIDIA GPU with 8GB+ VRAM (RTX 3070 or better)
- **Storage**: 500GB+ SSD

## Quick Setup (5 Minutes)

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/nava-studio.git
cd nava-studio
```

### 2. Install Rust Dependencies
```bash
# Update Rust toolchain
rustup update

# Install required targets
rustup target add wasm32-unknown-unknown

# Check installation
rustc --version
cargo --version
```

### 3. Install Node Dependencies
```bash
# Install frontend dependencies
npm install

# Verify installation
node --version
npm --version
```

### 4. Build the Project
```bash
# Build Rust backend
cd src-tauri
cargo build --release
cd ..

# Build frontend
npm run build
```

### 5. Run Development Server
```bash
# Start Tauri development server
npm run tauri dev
```

**That's it!** The NAVA Studio IDE should now be running on your machine.

---

## Project Structure

```
nava-studio/
├── src/                          # Frontend React application
│   ├── components/               # React components
│   ├── hooks/                    # Custom React hooks
│   ├── pages/                    # Page components
│   └── utils/                    # Utility functions
│
├── src-tauri/                    # Rust backend (Tauri)
│   ├── src/
│   │   ├── main.rs              # Entry point
│   │   ├── data/                # Dataset integration
│   │   │   ├── mod.rs
│   │   │   └── dataset_integration.rs
│   │   ├── simulation/          # Simulation engine
│   │   │   ├── mod.rs
│   │   │   └── simulation_engine.rs
│   │   ├── middleware/          # ROS2 middleware
│   │   │   ├── mod.rs
│   │   │   └── ros_middleware.rs
│   │   ├── ai/                  # AI/ML models
│   │   │   ├── mod.rs
│   │   │   └── vla_models.rs
│   │   ├── hardware/            # Hardware-in-the-Loop
│   │   │   ├── mod.rs
│   │   │   └── hil_system.rs
│   │   ├── cloud/               # Cloud deployment
│   │   │   ├── mod.rs
│   │   │   └── deployment.rs
│   │   └── ui/                  # Web interface
│   │       ├── mod.rs
│   │       └── web_interface.rs
│   └── Cargo.toml               # Rust dependencies
│
├── docs/                         # Documentation
├── k8s/                          # Kubernetes manifests
├── infrastructure/               # Terraform IaC
├── tests/                        # Test suites
└── scripts/                      # Build and deployment scripts
```

---

## Core Modules Overview

### 1. Data Layer (`src-tauri/src/data/`)
**Purpose**: Integrate and manage robotics datasets (Waymo, KITTI, RT-X)

**Key Files**:
- `dataset_integration.rs`: Main dataset integration logic

**Usage**:
```rust
use crate::data::dataset_integration::DatasetIntegration;

let dataset = DatasetIntegration::new();
dataset.load_waymo_data("path/to/waymo").await?;
```

**Tauri Command**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

await invoke('initialize_dataset_integration');
```

### 2. Simulation Engine (`src-tauri/src/simulation/`)
**Purpose**: Real-time physics simulation with CARLA, Isaac Sim, Gazebo

**Key Files**:
- `simulation_engine.rs`: Simulation manager and engine

**Usage**:
```rust
use crate::simulation::simulation_engine::SimulationEngine;

let mut engine = SimulationEngine::new();
engine.start().await?;
```

**Tauri Commands**:
```typescript
await invoke('start_simulation');
await invoke('stop_simulation');
```

### 3. ROS2 Middleware (`src-tauri/src/middleware/`)
**Purpose**: ROS2 communication and message passing

**Key Files**:
- `ros_middleware.rs`: ROS2 node management

**Usage**:
```rust
use crate::middleware::ros_middleware::RosMiddleware;

let ros = RosMiddleware::new();
ros.initialize().await?;
```

**Tauri Command**:
```typescript
await invoke('initialize_ros_middleware');
```

### 4. AI/ML Pipeline (`src-tauri/src/ai/`)
**Purpose**: Vision-Language-Action model training and inference

**Key Files**:
- `vla_models.rs`: VLA model implementations

**Usage**:
```rust
use crate::ai::vla_models::VlaModels;

let ai = VlaModels::new();
ai.train_model("rt-1").await?;
```

**Tauri Commands**:
```typescript
await invoke('train_ai_model', { modelName: 'rt-1' });
await invoke('run_ai_inference', { input: data });
```

### 5. Hardware-in-the-Loop (`src-tauri/src/hardware/`)
**Purpose**: Physical robot integration and testing

**Key Files**:
- `hil_system.rs`: HIL system management

**Usage**:
```rust
use crate::hardware::hil_system::HilSystem;

let hil = HilSystem::new();
hil.initialize().await?;
```

**Tauri Command**:
```typescript
await invoke('initialize_hil_system');
```

### 6. Cloud Deployment (`src-tauri/src/cloud/`)
**Purpose**: Kubernetes orchestration and scaling

**Key Files**:
- `deployment.rs`: Cloud deployment logic

**Usage**:
```rust
use crate::cloud::deployment::CloudDeployment;

let cloud = CloudDeployment::new();
cloud.deploy().await?;
```

**Tauri Command**:
```typescript
await invoke('deploy_to_cloud');
```

### 7. Web Interface (`src-tauri/src/ui/`)
**Purpose**: React-based control dashboard

**Key Files**:
- `web_interface.rs`: UI rendering and state management

**Usage**:
```rust
use crate::ui::web_interface::WebInterface;

let ui = WebInterface::new();
ui.render().await?;
```

**Tauri Command**:
```typescript
await invoke('render_ui');
```

---

## Development Workflow

### Daily Development
```bash
# 1. Pull latest changes
git pull origin main

# 2. Start development server
npm run tauri dev

# 3. Make changes to code

# 4. Test changes (auto-reload enabled)

# 5. Run tests
cargo test
npm test

# 6. Commit changes
git add .
git commit -m "feat: add new feature"
git push origin feature-branch
```

### Adding a New Feature

#### Backend (Rust)
1. Create new module in appropriate directory
2. Add module to `mod.rs`
3. Implement functionality
4. Create Tauri command in `main.rs`
5. Write tests

**Example**:
```rust
// src-tauri/src/data/new_feature.rs
pub struct NewFeature;

impl NewFeature {
    pub fn new() -> Self {
        Self
    }
    
    pub async fn do_something(&self) -> Result<String, String> {
        Ok("Success".to_string())
    }
}

// src-tauri/src/main.rs
#[tauri::command]
async fn new_feature_command() -> Result<String, String> {
    let feature = NewFeature::new();
    feature.do_something().await
}

fn main() {
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            new_feature_command,
            // ... other commands
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
```

#### Frontend (React)
1. Create new component in `src/components/`
2. Add to appropriate page
3. Use Tauri invoke to call backend
4. Handle state and UI updates

**Example**:
```typescript
// src/components/NewFeature.tsx
import { invoke } from '@tauri-apps/api/tauri';
import { useState } from 'react';

export function NewFeature() {
  const [result, setResult] = useState<string>('');

  const handleClick = async () => {
    try {
      const response = await invoke<string>('new_feature_command');
      setResult(response);
    } catch (error) {
      console.error('Error:', error);
    }
  };

  return (
    <div>
      <button onClick={handleClick}>Execute Feature</button>
      <p>{result}</p>
    </div>
  );
}
```

---

## Testing

### Unit Tests (Rust)
```bash
# Run all tests
cargo test

# Run specific test
cargo test test_name

# Run with output
cargo test -- --nocapture
```

**Example Test**:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_dataset_integration() {
        let dataset = DatasetIntegration::new();
        let result = dataset.initialize().await;
        assert!(result.is_ok());
    }
}
```

### Integration Tests (Frontend)
```bash
# Run frontend tests
npm test

# Run with coverage
npm run test:coverage
```

### End-to-End Tests
```bash
# Run E2E tests
npm run test:e2e
```

---

## Debugging

### Rust Backend
```bash
# Enable debug logging
RUST_LOG=debug npm run tauri dev

# Use Rust debugger
rust-lldb target/debug/nava-studio
```

### Frontend
```bash
# Open browser DevTools
# Right-click in app -> Inspect Element

# View console logs
console.log('Debug message');
```

### Common Issues

#### Issue: Compilation Errors
```bash
# Clean build artifacts
cargo clean
rm -rf node_modules
npm install
cargo build
```

#### Issue: Tauri Commands Not Found
- Ensure command is registered in `main.rs`
- Check command name matches in frontend
- Rebuild application

#### Issue: GPU Not Detected
```bash
# Check CUDA installation
nvcc --version
nvidia-smi

# Verify PyTorch GPU support
python -c "import torch; print(torch.cuda.is_available())"
```

---

## Performance Optimization

### Rust Backend
```rust
// Use release builds for performance testing
cargo build --release

// Enable LTO (Link Time Optimization)
// Add to Cargo.toml:
[profile.release]
lto = true
codegen-units = 1
```

### Frontend
```bash
# Build optimized production bundle
npm run build

# Analyze bundle size
npm run analyze
```

---

## Deployment

### Local Development
```bash
npm run tauri dev
```

### Production Build
```bash
# Build for current platform
npm run tauri build

# Output: src-tauri/target/release/bundle/
```

### Docker Deployment
```bash
# Build Docker image
docker build -t nava-studio:latest -f Dockerfile.backend .

# Run container
docker run -p 8080:8080 nava-studio:latest
```

### Kubernetes Deployment
```bash
# Apply manifests
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/backend-deployment.yaml
kubectl apply -f k8s/frontend-deployment.yaml

# Check status
kubectl get pods -n nava-studio
```

---

## Useful Commands

### Cargo (Rust)
```bash
cargo build              # Build project
cargo build --release    # Build optimized
cargo test               # Run tests
cargo check              # Check compilation
cargo clippy             # Lint code
cargo fmt                # Format code
cargo doc --open         # Generate and open docs
```

### NPM (Frontend)
```bash
npm install              # Install dependencies
npm run dev              # Start dev server
npm run build            # Build for production
npm test                 # Run tests
npm run lint             # Lint code
npm run format           # Format code
```

### Tauri
```bash
npm run tauri dev        # Development mode
npm run tauri build      # Production build
npm run tauri info       # System information
```

### Docker
```bash
docker build -t name .   # Build image
docker run name          # Run container
docker ps                # List containers
docker logs <id>         # View logs
docker exec -it <id> sh  # Enter container
```

### Kubernetes
```bash
kubectl get pods         # List pods
kubectl logs <pod>       # View logs
kubectl describe <pod>   # Pod details
kubectl exec -it <pod>   # Enter pod
kubectl port-forward     # Forward ports
```

---

## Resources

### Documentation
- **Production Implementation Plan**: `PRODUCTION_IMPLEMENTATION_PLAN.md`
- **Dataset Integration Spec**: `DATASET_INTEGRATION_SPEC.md`
- **Simulation Engine Spec**: `SIMULATION_ENGINE_SPEC.md`
- **API Documentation**: `API_DOCUMENTATION.md`
- **Deployment Guide**: `PRODUCTION_DEPLOYMENT_GUIDE.md`
- **Project Summary**: `PRODUCTION_READY_SUMMARY.md`

### External Resources
- **Rust**: https://www.rust-lang.org/learn
- **Tauri**: https://tauri.app/v1/guides/
- **React**: https://react.dev/learn
- **ROS2**: https://docs.ros.org/en/humble/
- **CARLA**: https://carla.readthedocs.io/
- **Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/
- **PyTorch**: https://pytorch.org/docs/

### Community
- **GitHub Issues**: Report bugs and request features
- **Discussions**: Ask questions and share ideas
- **Discord**: Join our developer community
- **Stack Overflow**: Tag questions with `nava-studio`

---

## Getting Help

### Common Questions

**Q: How do I add a new dataset?**  
A: Implement a new adapter in `src-tauri/src/data/` following the existing patterns.

**Q: How do I integrate a new simulation engine?**  
A: Add integration in `src-tauri/src/simulation/simulation_engine.rs`.

**Q: How do I expose a new API endpoint?**  
A: Create a Tauri command in `src-tauri/src/main.rs` and invoke from frontend.

**Q: How do I deploy to production?**  
A: Follow the `PRODUCTION_DEPLOYMENT_GUIDE.md` for step-by-step instructions.

### Support Channels
1. Check documentation first
2. Search existing GitHub issues
3. Ask in Discord community
4. Create a new GitHub issue

---

## Contributing

We welcome contributions! Please see `CONTRIBUTING.md` for guidelines.

### Quick Contribution Steps
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Write tests
5. Submit a pull request

---

## License

NAVA Studio is dual-licensed under MIT OR Apache-2.0.

---

**Happy Coding! 🚀**

For questions or support, reach out to the team or check the documentation.

**Last Updated**: 2025  
**Version**: 1.0
