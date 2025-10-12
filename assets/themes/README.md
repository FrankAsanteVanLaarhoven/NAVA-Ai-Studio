# NAVΛ Studio Editor Themes

Beautiful themes optimized for VNC mathematical programming.

## Built-in Themes

### 1. VNC Dark (Default)
Classic dark theme with vibrant VNC symbol highlighting.

**Colors**:
- Background: `#1a1a1a`
- Foreground: `#ffffff`
- ⋋ Symbol: `#00ff00` (Bright Green)
- Operators: `#00ccff` (Cyan)
- Master Operator: `#ff0099` (Magenta)

### 2. VNC Light
Light theme for daytime coding.

**Colors**:
- Background: `#f5f5f5`
- Foreground: `#2a2a2a`
- ⋋ Symbol: `#00aa00` (Dark Green)
- Operators: `#0088cc` (Blue)

### 3. Mathematical Midnight
Deep dark theme inspired by mathematical notation.

**Colors**:
- Background: `#0d0d0d`
- Foreground: `#e0e0e0`
- ⋋ Symbol: `#00ff66` (Bright Teal)
- Operators: `#66ccff` (Sky Blue)

### 4. Quantum Navigation
Inspired by quantum computing interfaces.

**Colors**:
- Background: `#13141a`
- Foreground: `#d4d4ff`
- ⋋ Symbol: `#00ffff` (Cyan)
- Operators: `#ff00ff` (Magenta)

## Theme Structure

```json
{
  "name": "Theme Name",
  "type": "dark",
  "colors": {
    "editor.background": "#1a1a1a",
    "editor.foreground": "#ffffff",
    ...
  },
  "tokenColors": [
    {
      "scope": "vnc.lambda.nav",
      "settings": {
        "foreground": "#00ff00",
        "fontStyle": "bold"
      }
    }
  ]
}
```

## Installing Custom Themes

1. Create theme file in `~/.navlambda/themes/`
2. Restart NAVΛ Studio
3. Select theme from Settings → Appearance

## Creating Themes

See theme template in `theme-template.json`

Theme guidelines:
- Ensure good contrast (WCAG AA minimum)
- Make VNC symbols distinctive
- Test with color blindness simulators
- Provide both dark and light variants

