# VNC Symbol Fonts

Fonts with complete support for Van Laarhoven Navigation Calculus mathematical symbols.

## Recommended Fonts

### 1. Latin Modern Math
Best overall support for mathematical symbols including ⋋.
- **Download**: [GUST Fonts](http://www.gust.org.pl/projects/e-foundry/lm-math)
- **License**: GUST Font License

### 2. STIX Two Math
Comprehensive mathematical font with excellent rendering.
- **Download**: [STIX Fonts](https://www.stixfonts.org/)
- **License**: OFL (Open Font License)

### 3. Cambria Math
Included with Microsoft Office, good Windows support.
- **Availability**: Pre-installed on Windows
- **License**: Proprietary (Microsoft)

## Installation

### macOS
```bash
cp *.otf ~/Library/Fonts/
```

### Linux
```bash
cp *.otf ~/.local/share/fonts/
fc-cache -f -v
```

### Windows
Right-click font files and select "Install"

## VNC Symbol Coverage

All recommended fonts support:
- ⋋ (U+22CB) - Lambda Navigation
- ⊗ (U+2297) - Tensor Product
- ⊕ (U+2295) - Direct Sum
- ∪ (U+222A) - Union
- ∩ (U+2229) - Intersection
- ↑ (U+2191) - Upward Arrow
- ↓ (U+2193) - Downward Arrow
- → (U+2192) - Rightward Arrow
- ← (U+2190) - Leftward Arrow
- 𝒩 (U+1D4A9) - Script N
- ℐ (U+2110) - Script I
- ℰ (U+2130) - Script E

## Fallback Strategy

NAVΛ Studio uses the following font fallback chain:
1. Latin Modern Math
2. STIX Two Math
3. Cambria Math
4. System default math font

## Custom Font Configuration

Edit your user settings:
```json
{
  "editor.fontFamily": "Latin Modern Math, JetBrains Mono, monospace"
}
```

