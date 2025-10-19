# NAVΛ Studio Testing Framework

Comprehensive testing suite for NAVΛ Studio IDE with unit tests, integration tests, and end-to-end testing capabilities.

## 🚀 Test Structure

```
tests/
├── unit/                    # Unit tests for individual components
│   ├── components/            # Component-specific tests
│   │   ├── editor.test.ts       # Editor component tests
│   │   ├── terminal.test.ts     # Terminal component tests
│   │   ├── visualizer.test.ts   # 3D visualizer tests
│   │   ├── compiler.test.ts     # Compiler tests
│   │   └── lsp.test.ts          # Language Server tests
│   ├── utils/                 # Utility function tests
│   │   ├── parser.test.ts       # VNC parser tests
│   │   ├── validator.test.ts    # Code validator tests
│   │   ├── formatter.test.ts    # Code formatter tests
│   │   └── config.test.ts       # Configuration tests
│   ├── ros/                   # ROS integration tests
│   │   ├── commands.test.ts     # ROS command tests
│   │   ├── terminal.test.ts     # ROS terminal tests
│   │   └── simulation.test.ts   # Gazebo simulation tests
│   └── api/                   # API endpoint tests
│       ├── editor.test.ts       # Editor API tests
│       ├── terminal.test.ts     # Terminal API tests
│       └── compiler.test.ts     # Compiler API tests
├── integration/             # Integration tests
│   ├── editor-integration.test.ts
│   ├── ros-integration.test.ts
│   ├── compiler-integration.test.ts
│   └── full-workflow.test.ts
├── e2e/                     # End-to-end tests
│   ├── user-journey.test.ts     # Complete user workflows
│   ├── ros-learning.test.ts     # ROS learning center tests
│   ├── multi-target.test.ts     # Multi-compilation tests
│   └── performance.test.ts      # Performance benchmarks
├── fixtures/                # Test data and mock objects
│   ├── vnc-samples/           # Sample VNC code files
│   ├── ros-configs/           # ROS configuration files
│   └── mock-data/             # Mock data for testing
├── utils/                   # Testing utilities and helpers
│   ├── test-helpers.ts        # Common test utilities
│   ├── mock-factory.ts        # Mock object creation
│   ├── assertions.ts          # Custom assertions
│   └── setup.ts               # Test environment setup
└── performance/             # Performance and load tests
    ├── benchmarks.ts          # Performance benchmarks
    └── load-tests.ts          # Load testing scenarios
```

## 🧪 Testing Technologies

### Unit Testing
- **Jest**: JavaScript/TypeScript testing framework
- **React Testing Library**: Component testing utilities
- **Mock Service Worker**: API mocking
- **Sinon**: Standalone test spies, stubs, and mocks

### Integration Testing
- **Supertest**: HTTP assertion library
- **TestContainers**: Docker container management for tests
- **MongoDB Memory Server**: In-memory MongoDB for testing

### End-to-End Testing
- **Playwright**: Cross-browser automation
- **Cypress**: End-to-end testing framework
- **Selenium**: WebDriver automation

### Performance Testing
- **Artillery**: Load testing framework
- **Lighthouse**: Performance auditing
- **Chrome DevTools**: Performance profiling

## 🔧 Running Tests

### Run All Tests
```bash
npm test                    # Run all tests
npm run test:watch         # Run tests in watch mode
npm run test:coverage      # Run tests with coverage report
```

### Run Specific Test Suites
```bash
npm run test:unit          # Run unit tests only
npm run test:integration   # Run integration tests only
npm run test:e2e           # Run end-to-end tests only
npm run test:performance   # Run performance tests only
```

### Run Tests by Pattern
```bash
npm test -- --testNamePattern="Editor"     # Run tests with "Editor" in name
npm test -- --testPathPattern="components"  # Run tests in components directory
npm test -- --verbose                       # Run with detailed output
```

### Debug Tests
```bash
npm run test:debug         # Run tests in debug mode
npm run test:inspect       # Run with Node.js inspector
```

## 📋 Test Coverage

### Coverage Goals
- **Statements**: 90% minimum
- **Branches**: 85% minimum
- **Functions**: 95% minimum
- **Lines**: 90% minimum

### Coverage Reports
```bash
npm run test:coverage      # Generate coverage report
npm run test:coverage:html # Generate HTML coverage report
npm run test:coverage:lcov # Generate LCOV coverage report
```

### Coverage Badge
![Coverage Status](https://img.shields.io/badge/coverage-92%25-brightgreen)

## 🔗 Continuous Integration

Tests run automatically on:
- **Pull Request**: All test suites
- **Push to Main**: Full test suite with coverage
- **Nightly**: Extended test suite including performance tests
- **Release**: Complete validation including manual tests

### CI/CD Platforms
- **GitHub Actions**: Primary CI/CD platform
- **CircleCI**: Backup and specialized testing
- **Jenkins**: Internal testing infrastructure

## 💡 Best Practices

### Test Organization
- Group related tests in describe blocks
- Use meaningful test names that describe behavior
- Keep tests focused and independent
- Use setup and teardown functions appropriately

### Mocking Strategy
- Mock external dependencies
- Use realistic mock data
- Reset mocks between tests
- Avoid over-mocking

### Assertion Quality
- Use specific assertions instead of generic ones
- Assert on behavior, not implementation
- Include helpful error messages
- Test edge cases and error conditions

### Performance Considering
- Keep unit tests fast (< 100ms per test)
- Use parallel execution when possible
- Optimize test data setup
- Profile slow tests regularly

## 🎯 Test Types

### Unit Tests
Test individual functions and components in isolation:
```typescript
describe('EditorAPI', () => {
  test('should create editor with VNC support', () => {
    const editor = EditorAPI.createEditor(container, { vncMode: true });
    expect(editor).toBeDefined();
    expect(editor.getOption('vncMode')).toBe(true);
  });

  test('should register completion provider', () => {
    const provider = createMockCompletionProvider();
    const disposable = EditorAPI.registerCompletionProvider('navlambda', provider);
    expect(disposable).toBeDefined();
    expect(disposable.dispose).toBeInstanceOf(Function);
  });
});
```

### Integration Tests
Test interaction between components:
```typescript
describe('Editor and Terminal Integration', () => {
  test('should execute VNC code and display output', async () => {
    const editor = createEditorWithVNCCode('⋋(x) = x + 1');
    const terminal = createTerminalSession();

    await executeCode(editor, terminal);

    expect(terminal.getOutput()).toContain('Navigation function executed');
    expect(editor.getErrors()).toHaveLength(0);
  });
});
```

### End-to-End Tests
Test complete user workflows:
```typescript
test('complete navigation workflow', async ({ page }) => {
  await page.goto('http://localhost:3000');
  await page.click('text=New Project');
  await page.fill('[placeholder="Project name"]', 'Navigation Demo');
  await page.click('button:has-text("Create")');

  await page.fill('.monaco-editor', '⋋(start, end) = navigate(start, end)');
  await page.click('text=Compile');
  await page.click('text=Visualize');

  await expect(page.locator('canvas')).toBeVisible();
  await expect(page.locator('text=3D Visualization')).toBeVisible();
});
```

### Rust Backend Tests
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

    #[test]
    fn test_lsp_completion() {
        let lsp = LanguageServer::new();
        let completions = lsp.get_completions("⋋", 1);
        assert!(!completions.is_empty());
    }
}
```

### Rust Backend Test Commands
```bash
cd src-tauri
cargo test --all-features          # Run all Rust tests
cargo test --lib lsp               # Run LSP tests
cargo test --lib compiler          # Run compiler tests
cargo bench                        # Run performance benchmarks
```

## 🚨 Known Issues

### Common Test Failures
1. **Timing Issues**: Use proper async/await patterns
2. **Resource Cleanup**: Ensure proper cleanup in afterEach
3. **Mock Conflicts**: Reset mocks between test suites
4. **Environment Dependencies**: Use consistent test environments

### Flaky Tests
- Tests that depend on timing
- Tests with external dependencies
- Tests that modify global state
- Tests with race conditions

### Platform-Specific Issues
- Windows: Path separator differences
- macOS: Permission and security restrictions
- Linux: Font and rendering differences

## 📊 Performance Benchmarks

Key performance metrics:
- **LSP Response Time**: <100ms
- **Compilation Speed**: <5s for typical project
- **3D Rendering**: 60 FPS minimum
- **Memory Usage**: <500MB typical
- **Test Suite Runtime**: <5 minutes full suite

## 📞 Support

For test-related questions:
- **GitHub Issues**: Report bugs and request features
- **Discord**: Join our testing community
- **Stack Overflow**: Tag questions with `navlambda-testing`
- **Documentation**: Check our testing guides

## 📁 Test Data

Example VNC code for testing is available in:
- `tests/fixtures/vnc-samples/` - Sample VNC navigation code
- `tests/fixtures/ros-configs/` - ROS configuration files
- `tests/fixtures/mock-data/` - Mock data for various test scenarios

---

*This testing framework ensures NAVΛ Studio maintains the highest quality standards. Contribute by writing tests for new features and reporting any issues you encounter.*

