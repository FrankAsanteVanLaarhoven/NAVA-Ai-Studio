# NAVΛ Studio Built-in Plugins

This directory contains the official plugin ecosystem for NAVΛ Studio.

## Available Plugins

### 1. VNC Math Verification (`vnc-math-verification/`)
Mathematical theorem verification for VNC equations.

**Features**:
- Automatic theorem checking
- Proof verification
- Mathematical consistency validation

### 2. Navigation Debugger (`navigation-debugger/`)
Advanced debugging tools for navigation paths.

**Features**:
- Path tracing visualization
- Energy landscape analysis
- Breakpoint management
- Performance profiling

### 3. Cloud Deployer (`cloud-deployer/`)
One-click cloud deployment integration.

**Features**:
- AWS, GCP, Azure support
- Docker container generation
- Kubernetes manifest creation
- Automated CI/CD pipelines

### 4. Performance Profiler (`performance-profiler/`)
Comprehensive performance analysis tools.

**Features**:
- Real-time profiling
- Memory usage tracking
- CPU utilization monitoring
- Optimization suggestions

### 5. Git Integration (`git-integration/`)
Version control integration for NAVΛ projects.

**Features**:
- Git workflow integration
- Diff visualization
- Commit management
- Branch operations

## Installing Plugins

```bash
# Via NAVΛ Studio UI
Settings → Plugins → Install from Marketplace

# Via CLI
navλ plugin install <plugin-name>

# Manual installation
cp -r plugin-directory ~/.navlambda/plugins/
```

## Developing Plugins

See the [Plugin Development Guide](../docs/plugin-development.md) for detailed information on creating your own plugins.

## Plugin Structure

```
plugin-name/
├── Cargo.toml          # Rust manifest
├── package.json        # npm manifest
├── src/
│   ├── lib.rs         # Rust implementation
│   └── index.ts       # TypeScript implementation
├── README.md
└── LICENSE
```

## Contributing

We welcome plugin contributions! See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

