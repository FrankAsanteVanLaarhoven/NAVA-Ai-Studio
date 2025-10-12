# NAVΛ Studio Test Suite

Comprehensive testing for all components of NAVΛ Studio.

## Test Structure

```
tests/
├── integration/        # End-to-end integration tests
├── lsp/               # Language server tests
├── compiler/          # Compilation tests
├── frontend/          # Frontend component tests
└── performance/       # Performance benchmarks
```

## Running Tests

### All Tests
```bash
./scripts/test.sh
```

### Rust Backend Tests
```bash
cd src-tauri
cargo test --all-features
```

### Frontend Tests
```bash
npm run test
```

### Specific Test Suites
```bash
# LSP tests
cargo test --package navlambda-studio --lib lsp

# Compiler tests
cargo test --package navlambda-studio --lib compiler

# Frontend component tests
npm run test -- --grep "MonacoEditor"
```

## Test Coverage

Current coverage targets:
- Backend (Rust): >80%
- Frontend (TypeScript): >75%
- Integration: All critical paths
- Performance: All key operations

## Writing Tests

### Rust Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_navigation_parsing() {
        let parser = NavLambdaParser::new();
        let result = parser.parse("navigate_to⋋(start, goal)");
        assert!(result.is_ok());
    }
}
```

### TypeScript Tests
```typescript
import { describe, it, expect } from 'vitest';

describe('NavigationVisualizer', () => {
  it('should render paths', () => {
    const paths = [/* test data */];
    const { container } = render(<NavigationVisualizer paths={paths} />);
    expect(container).toBeTruthy();
  });
});
```

## Continuous Integration

Tests run automatically on:
- Every commit (pre-commit hook)
- Pull requests (GitHub Actions)
- Main branch merges
- Release builds

## Performance Benchmarks

Run benchmarks with:
```bash
cargo bench
```

Key metrics:
- LSP response time: <100ms
- Compilation speed: <5s for typical project
- 3D rendering: 60 FPS minimum
- Memory usage: <500MB typical

## Test Data

Example VNC code for testing is available in `assets/examples/`.

