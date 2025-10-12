# Contributing to NAVΛ Studio

Thank you for your interest in contributing to NAVΛ Studio! This document provides guidelines for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Process](#development-process)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Pull Request Process](#pull-request-process)
- [Community](#community)

## Code of Conduct

### Our Pledge

We are committed to providing a welcoming and inclusive experience for everyone, regardless of:
- Age, body size, disability, ethnicity, gender identity and expression
- Level of experience, nationality, personal appearance, race, religion
- Sexual identity and orientation

### Our Standards

**Positive behaviors:**
- Using welcoming and inclusive language
- Being respectful of differing viewpoints
- Gracefully accepting constructive criticism
- Focusing on what is best for the community
- Showing empathy towards other community members

**Unacceptable behaviors:**
- Trolling, insulting/derogatory comments, and personal or political attacks
- Public or private harassment
- Publishing others' private information without permission
- Other conduct which could reasonably be considered inappropriate

## Getting Started

### Prerequisites

- Rust 1.75+
- Node.js 20+
- Git
- Basic knowledge of Rust, TypeScript, and React

### Setup Development Environment

```bash
# Clone the repository
git clone https://github.com/frankvanlaarhoven/navlambda-studio.git
cd navlambda-studio

# Run setup script
chmod +x scripts/setup-dev-environment.sh
./scripts/setup-dev-environment.sh

# Start development server
npm run tauri:dev
```

## Development Process

### 1. Find or Create an Issue

- Check existing issues for something you'd like to work on
- For new features, create an issue first to discuss
- Comment on the issue to let others know you're working on it

### 2. Fork and Branch

```bash
# Fork the repository on GitHub
# Clone your fork
git clone https://github.com/YOUR_USERNAME/navlambda-studio.git

# Create a feature branch
git checkout -b feature/your-feature-name
```

### 3. Make Your Changes

- Write clear, documented code
- Follow the coding standards below
- Add tests for new functionality
- Update documentation as needed

### 4. Test Your Changes

```bash
# Run Rust tests
cd src-tauri
cargo test

# Run frontend tests
npm run test

# Run linter
npm run lint
```

### 5. Commit Your Changes

```bash
# Stage your changes
git add .

# Commit with a clear message
git commit -m "Add feature: your feature description"
```

**Commit Message Format:**
```
<type>: <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks

**Example:**
```
feat: Add quantum navigation operator

Implements the quantum evolution operator (ℰ) for
Van Laarhoven Navigation Calculus, enabling quantum-
inspired path optimization.

Closes #123
```

### 6. Push and Create Pull Request

```bash
# Push to your fork
git push origin feature/your-feature-name

# Create a Pull Request on GitHub
```

## Coding Standards

### Rust Code

```rust
// Use descriptive names
pub struct NavigationPath {
    waypoints: Vec<Point3D>,
    energy: f64,
}

// Document public APIs
/// Navigate from start to goal using VNC optimization.
/// 
/// # Arguments
/// * `start` - Starting position
/// * `goal` - Goal position
/// 
/// # Returns
/// Optimized navigation path
pub fn navigate_to_vnc(start: Point3D, goal: Point3D) -> NavigationPath {
    // Implementation
}

// Use Result for error handling
pub fn parse_code(code: &str) -> Result<Ast, ParseError> {
    // Implementation
}

// Follow Rust conventions
// - Snake case for functions and variables
// - PascalCase for types
// - SCREAMING_SNAKE_CASE for constants
```

### TypeScript/React Code

```typescript
// Use TypeScript types
interface NavigationPath {
  waypoints: [number, number, number][];
  energy: number;
}

// Functional components with hooks
export const NavigationVisualizer: React.FC<Props> = ({ paths }) => {
  const [isAnimating, setIsAnimating] = useState(false);
  
  useEffect(() => {
    // Effect logic
  }, [paths]);
  
  return (
    <div className="visualizer">
      {/* Component JSX */}
    </div>
  );
};

// Use async/await for promises
async function compileCode(code: string): Promise<CompiledOutput> {
  const result = await compilerService.compile(code);
  return result;
}
```

### Code Style

- **Indentation**: 2 spaces for TypeScript, 4 spaces for Rust
- **Line Length**: Maximum 100 characters
- **Comments**: Write clear, concise comments for complex logic
- **Naming**: Use descriptive, meaningful names

### Documentation

- Document all public APIs
- Include examples in documentation
- Update README.md for significant features
- Add JSDoc/rustdoc comments

## Testing

### Unit Tests

```rust
// Rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_navigation_parsing() {
        let code = "navigate_to⋋(start, goal)";
        let result = parse_code(code);
        assert!(result.is_ok());
    }
}
```

```typescript
// TypeScript
import { describe, it, expect } from 'vitest';

describe('NavigationVisualizer', () => {
  it('should render paths correctly', () => {
    const paths = [/* test data */];
    const { container } = render(<NavigationVisualizer paths={paths} />);
    expect(container).toBeTruthy();
  });
});
```

### Integration Tests

- Test complete workflows
- Test cross-component interactions
- Test compilation to various targets

### Manual Testing

- Test in development mode
- Test production build
- Test on different platforms (Windows, macOS, Linux)

## Pull Request Process

### Before Submitting

- [ ] Code follows style guidelines
- [ ] Tests pass locally
- [ ] Documentation is updated
- [ ] Commit messages are clear
- [ ] Branch is up to date with main

### PR Description Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
How has this been tested?

## Screenshots (if applicable)
Add screenshots for UI changes

## Checklist
- [ ] My code follows the style guidelines
- [ ] I have performed a self-review
- [ ] I have commented my code where needed
- [ ] I have updated the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix/feature works
- [ ] New and existing tests pass locally
```

### Review Process

1. **Automated Checks**: CI/CD runs tests and linters
2. **Code Review**: Maintainers review your code
3. **Discussion**: Address feedback and make changes
4. **Approval**: Once approved, your PR will be merged

### After Merge

- Delete your feature branch
- Update your fork
- Close related issues

## Community

### Communication Channels

- **GitHub Discussions**: For questions and ideas
- **Discord**: Real-time chat (https://discord.gg/navlambda)
- **Twitter**: Updates and announcements (@navlambda_studio)
- **Email**: support@navlambda.studio

### Getting Help

- Check existing documentation
- Search for similar issues
- Ask in Discord or GitHub Discussions
- Be patient and respectful

### Recognition

Contributors are recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation

## Areas Where We Need Help

### High Priority

- [ ] Documentation improvements
- [ ] Bug fixes
- [ ] Test coverage
- [ ] Performance optimizations

### Feature Development

- [ ] Additional compilation targets
- [ ] Plugin ecosystem
- [ ] Cloud integrations
- [ ] Mobile support

### Community

- [ ] Writing tutorials
- [ ] Creating examples
- [ ] Translating documentation
- [ ] Answering questions

## License

By contributing to NAVΛ Studio, you agree that your contributions will be licensed under the project's MIT OR Apache-2.0 license.

## Questions?

Don't hesitate to ask! We're here to help:
- Open an issue with the "question" label
- Ask in Discord
- Email support@navlambda.studio

---

**Thank you for contributing to NAVΛ Studio!** 🙏⋋💻

Together we're building the future of Van Laarhoven Navigation Calculus programming!

