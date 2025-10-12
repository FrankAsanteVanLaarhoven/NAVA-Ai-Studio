# Multi-Target Compilation Guide

NAVΛ Studio can compile your Van Laarhoven Navigation Calculus code to multiple target languages and platforms.

## Supported Targets

### 1. C++ (High Performance)

**Target**: `cpp`

Generates optimized C++17 code for maximum performance.

```bash
# Compile to C++
navλ compile --target cpp input.vnc -o output.cpp

# Build with compiler
g++ -std=c++17 -O3 -march=native output.cpp -o app
```

**Features**:
- Zero-overhead abstractions
- SIMD optimizations
- Template metaprogramming
- STL integration

**Use Cases**:
- High-performance applications
- Embedded systems
- Real-time systems
- Game engines

### 2. Python (Prototyping & Data Science)

**Target**: `python`

Generates Python 3.12+ code with NumPy integration.

```bash
# Compile to Python
navλ compile --target python input.vnc -o output.py

# Run
python output.py
```

**Features**:
- NumPy array operations
- SciPy integration
- Matplotlib visualization
- Jupyter notebook support

**Use Cases**:
- Data science
- Machine learning
- Rapid prototyping
- Research

### 3. WebAssembly (Browser & Web)

**Target**: `wasm`

Compiles to WebAssembly for browser execution.

```bash
# Compile to WASM
navλ compile --target wasm input.vnc

# Build with wasm-pack
wasm-pack build --target web
```

**Features**:
- Near-native performance
- Browser sandboxing
- JavaScript interop
- Memory safety

**Use Cases**:
- Web applications
- Interactive visualizations
- Browser-based tools
- Progressive web apps

### 4. GLSL (GPU Shaders)

**Target**: `glsl`

Generates GPU compute shaders for parallel processing.

```bash
# Compile to GLSL
navλ compile --target glsl input.vnc -o shader.glsl

# Compile shader
glslangValidator -V shader.glsl -o shader.spv
```

**Features**:
- Massively parallel
- GPU acceleration
- Real-time processing
- Vector operations

**Use Cases**:
- Real-time visualization
- Parallel navigation
- Image processing
- Scientific computing

### 5. Docker (Containerization)

**Target**: `docker`

Generates Dockerfile and container configuration.

```bash
# Generate Dockerfile
navλ compile --target docker input.vnc

# Build container
docker build -t navλ-app .

# Run
docker run -p 8080:8080 navλ-app
```

**Features**:
- Reproducible builds
- Isolated environment
- Easy deployment
- Version control

**Use Cases**:
- Microservices
- Cloud deployment
- CI/CD pipelines
- Development environments

### 6. Kubernetes (Orchestration)

**Target**: `kubernetes`

Generates Kubernetes manifests and Helm charts.

```bash
# Generate Kubernetes config
navλ compile --target kubernetes input.vnc

# Deploy to cluster
kubectl apply -f deployment.yaml

# Or use Helm
helm install navλ-app ./chart
```

**Features**:
- Auto-scaling
- Load balancing
- Health checks
- Rolling updates

**Use Cases**:
- Cloud-native apps
- Distributed systems
- Production deployment
- High availability

## Compilation Options

### Optimization Levels

```bash
# Debug build (fast compilation, slower runtime)
navλ compile --target cpp --opt debug input.vnc

# Release build (optimized for speed)
navλ compile --target cpp --opt release input.vnc

# Size-optimized build
navλ compile --target cpp --opt size input.vnc
```

### VNC-Specific Optimizations

```bash
# Enable VNC mathematical optimizations
navλ compile --vnc-optimize input.vnc

# Use quantum navigation algorithms
navλ compile --quantum input.vnc

# Enable consciousness integration
navλ compile --consciousness input.vnc
```

## Cross-Compilation

Compile for different platforms:

```bash
# Target specific architecture
navλ compile --target cpp --arch x86_64 input.vnc
navλ compile --target cpp --arch arm64 input.vnc
navλ compile --target cpp --arch wasm32 input.vnc

# Target specific OS
navλ compile --target cpp --os linux input.vnc
navλ compile --target cpp --os macos input.vnc
navλ compile --target cpp --os windows input.vnc
```

## Multi-Target Builds

Compile to multiple targets simultaneously:

```bash
# Compile to all targets
navλ compile --all-targets input.vnc

# Compile to specific targets
navλ compile --targets cpp,python,wasm input.vnc

# Output to directory
navλ compile --all-targets --output-dir ./build input.vnc
```

## Performance Comparison

| Target | Relative Speed | Memory Usage | Startup Time |
|--------|---------------|--------------|--------------|
| C++ | 10x (baseline) | Low | Fast |
| GLSL (GPU) | 100x | GPU Memory | Fast |
| WebAssembly | 8x | Low | Medium |
| Python | 1x | Medium | Slow |

## Example: Full-Stack Application

Compile different components for different targets:

```bash
# Backend (high performance)
navλ compile --target cpp backend.vnc -o backend

# Frontend (browser)
navλ compile --target wasm frontend.vnc

# GPU compute (parallel processing)
navλ compile --target glsl compute.vnc

# Data processing (Python)
navλ compile --target python analysis.vnc

# Deployment (Docker + Kubernetes)
navλ compile --target docker,kubernetes fullstack.vnc
```

## Integration with Build Systems

### CMake (C++)

```cmake
# CMakeLists.txt
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated.cpp
  COMMAND navλ compile --target cpp ${CMAKE_CURRENT_SOURCE_DIR}/nav.vnc
  DEPENDS nav.vnc
)

add_executable(app ${CMAKE_CURRENT_BINARY_DIR}/generated.cpp)
```

### Webpack (WebAssembly)

```javascript
// webpack.config.js
module.exports = {
  module: {
    rules: [
      {
        test: /\.vnc$/,
        use: ['navlambda-wasm-loader']
      }
    ]
  }
};
```

### Poetry (Python)

```toml
# pyproject.toml
[tool.poetry.scripts]
build-vnc = "scripts:compile_vnc"
```

## Debugging Compiled Code

### C++ Debugging

```bash
# Compile with debug symbols
navλ compile --target cpp --debug input.vnc -o output.cpp
g++ -g output.cpp -o app

# Debug with GDB
gdb ./app
```

### WebAssembly Debugging

```bash
# Enable DWARF debug info
navλ compile --target wasm --debug input.vnc

# Debug in Chrome DevTools
chrome --enable-features=WebAssemblyDebugging
```

### GLSL Debugging

```bash
# Validate shader
glslangValidator shader.glsl

# Profile with RenderDoc
renderdoc app
```

## Best Practices

1. **Choose the right target for your use case**
   - C++ for performance-critical code
   - Python for data analysis
   - WASM for web applications
   - GLSL for parallel processing

2. **Optimize after profiling**
   - Measure before optimizing
   - Focus on hot paths
   - Use target-specific optimizations

3. **Test on target platform**
   - CI/CD for multiple targets
   - Platform-specific testing
   - Performance benchmarks

4. **Keep source code portable**
   - Avoid target-specific features
   - Use VNC abstractions
   - Document platform differences

## Troubleshooting

### Common Issues

**Issue**: Compilation fails for specific target

**Solution**: Check target-specific dependencies
```bash
navλ doctor --target cpp
```

**Issue**: Performance not as expected

**Solution**: Enable optimizations
```bash
navλ compile --target cpp --opt release --vnc-optimize input.vnc
```

**Issue**: WASM module too large

**Solution**: Enable size optimizations
```bash
navλ compile --target wasm --opt size input.vnc
wasm-opt -Oz output.wasm -o optimized.wasm
```

## Advanced Topics

### Custom Target Plugins

Create your own compilation target:

```rust
use navlambda_compiler::*;

pub struct MyTargetCompiler;

impl TargetCompiler for MyTargetCompiler {
    fn compile(&self, ast: &Ast) -> Result<String> {
        // Your code generation logic
    }
}
```

### Linking with Native Libraries

```bash
# Link with external libraries
navλ compile --target cpp --link opengl,pthread input.vnc
```

### Profile-Guided Optimization

```bash
# Generate profile
navλ compile --target cpp --pgo-generate input.vnc
./app  # Run to generate profile data

# Use profile for optimization
navλ compile --target cpp --pgo-use input.vnc
```

---

**Multi-Target Compilation**: Write once, run anywhere 🚀⋋

