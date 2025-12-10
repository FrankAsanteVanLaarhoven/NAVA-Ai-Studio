# NAVΛ SDK - Ultimate Cross-Platform SDK

**The Expert-Led SDK for NAVΛ Platform - Runs Everywhere**

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS%20%7C%20Windows%20%7C%20Web-lightgrey)](https://github.com/nava-studio/nava-sdk)
[![Rust](https://img.shields.io/badge/Rust-1.75+-orange.svg)](https://www.rust-lang.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.0+-blue.svg)](https://www.typescriptlang.org/)

## 🚀 Overview

The **NAVΛ SDK** is a production-ready, expert-led software development kit that enables developers to integrate NAVΛ platform capabilities into any application, on any platform.

### ✨ Key Features

- **🌐 Universal Platform Support**: Native support for Linux, macOS, Windows, Web, and Browser Extensions
- **⚡ High Performance**: Rust core with WebAssembly for web, native binaries for desktop
- **🔧 Expert-Led Design**: Built by robotics and navigation calculus experts
- **📦 Easy Integration**: Simple APIs for all platforms
- **🎨 NAVΛ Branding**: Official logo (λΛ) with glowing green effect included in all packages
- **🌐 Web Extension**: Enterprise-grade browser extension (Chrome, Firefox, Edge, Safari)
- **📚 Comprehensive Docs**: Complete documentation with examples
- **🧪 Well Tested**: Extensive test suite and CI/CD

## 📦 Installation

### Native Platforms (Linux, macOS, Windows)

#### Rust/Cargo

```toml
[dependencies]
nava-sdk = { version = "0.1.0", path = "../sdk/native" }
```

#### Python

```bash
pip install nava-sdk
```

#### Node.js

```bash
npm install @nava/sdk
```

### Web Platform

```bash
npm install @nava/sdk-web
```

Or via CDN:

```html
<script src="https://cdn.nava.studio/sdk/v1/nava-sdk.js"></script>
```

### Browser Extension

Install the NAVΛ SDK browser extension:

- **Chrome**: Load unpacked from `sdk/web-extension/` or install from Chrome Web Store
- **Firefox**: Load temporary from `sdk/web-extension/` or install from Firefox Add-ons
- **Edge**: Load unpacked from `sdk/web-extension/` or install from Edge Add-ons

Build the extension:

```bash
cd sdk/web-extension
npm install
npm run build
```

See [web-extension/README.md](web-extension/README.md) for full documentation.

## 🎯 Quick Start

### Rust

```rust
use nava_sdk::*;

fn main() {
    let mut nav = NavigationField::new();
    nav.set_manifold(Manifold::euclidean(3));
    
    let path = nav.find_optimal_path(
        &[0.0, 0.0, 0.0],
        &[5.0, 5.0, 5.0],
        &NavigationConstraints::default()
    );
    
    println!("Optimal path: {:?}", path);
}
```

### TypeScript/JavaScript (Web)

```typescript
import { NavigationField, Manifold } from '@nava/sdk-web';

const nav = new NavigationField();
nav.setManifold(Manifold.euclidean(3));

const path = nav.findOptimalPath(
    [0, 0, 0],
    [5, 5, 5],
    NavigationConstraints.default()
);

console.log('Optimal path:', path);
```

### Python

```python
from nava_sdk import NavigationField, Manifold

nav = NavigationField()
nav.set_manifold(Manifold.euclidean(3))

path = nav.find_optimal_path(
    [0.0, 0.0, 0.0],
    [5.0, 5.0, 5.0],
    NavigationConstraints.default()
)

print(f"Optimal path: {path}")
```

## 🏗️ Architecture

```
sdk/
├── native/          # Rust core library
├── web/             # TypeScript/WebAssembly bindings
├── bindings/         # Language bindings (Python, Node.js, C/C++)
├── examples/         # Example projects
├── docs/             # Documentation
└── assets/           # Logo, icons, branding
```

## 📚 Documentation

- [API Reference](./docs/api-reference.md)
- [Platform Guides](./docs/platforms/)
- [Examples](./examples/)
- [Contributing](./CONTRIBUTING.md)

## 🛠️ Building from Source

### Prerequisites

- Rust 1.75+ (for native builds)
- Node.js 18+ (for web builds)
- Python 3.10+ (for Python bindings)

### Build All Platforms

```bash
./scripts/build-all.sh
```

### Build Specific Platform

```bash
# Native
cd native && cargo build --release

# Web
cd web && npm run build

# Python
cd bindings/python && python setup.py build
```

## 🧪 Testing

```bash
# Run all tests
./scripts/test-all.sh

# Test specific platform
cargo test          # Native
npm test            # Web
pytest              # Python
```

## 📄 License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](./CONTRIBUTING.md) for details.

## 🔗 Links

- [NAVΛ Studio IDE](https://github.com/nava-studio/nava-studio-ide)
- [Documentation](https://docs.nava.studio)
- [Discord Community](https://discord.gg/nava-studio)

---

**Built with ❤️ by the NAVΛ Team**

