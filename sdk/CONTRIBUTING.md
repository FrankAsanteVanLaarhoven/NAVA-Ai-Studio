# Contributing to NAVΛ SDK

Thank you for your interest in contributing to the NAVΛ SDK!

## Development Setup

### Prerequisites

- Rust 1.75+ (for native SDK)
- Node.js 18+ (for web SDK)
- Python 3.10+ (for Python bindings)
- Git

### Getting Started

1. Clone the repository:
```bash
git clone https://github.com/nava-studio/nava-sdk.git
cd nava-sdk
```

2. Build all platforms:
```bash
./scripts/build-all.sh
```

3. Run tests:
```bash
./scripts/test-all.sh
```

## Code Style

### Rust

Follow the official Rust style guide. Run `cargo fmt` before committing.

### TypeScript

Use ESLint and Prettier. Run `npm run lint` and `npm run format`.

### Python

Follow PEP 8. Use `black` for formatting.

## Testing

All code must include tests:

- **Rust**: Unit tests in `#[cfg(test)]` modules
- **TypeScript**: Jest tests in `*.test.ts` files
- **Python**: pytest tests in `test_*.py` files

## Pull Requests

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Ensure all tests pass
6. Submit a pull request

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (MIT OR Apache-2.0).

