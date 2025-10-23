# 🎨 NAVΛ Studio Brand Identity Guide

## Official Brand Colors

### Primary Brand Color
**Lime Green** - Our signature color that represents innovation, growth, and energy.

```css
--navlambda-primary: #7FD957
--navlambda-primary-hover: #8FE967
--navlambda-primary-active: #6FC947
```

**Usage:**
- Primary action buttons (Run, Execute, Submit)
- Active states and highlights
- Success messages and confirmations
- Brand logo and accents
- Focus indicators
- Links and interactive elements

**Do's:**
- Use for primary CTAs (Call-to-Actions)
- Use for success states
- Use for active/selected states
- Use sparingly for maximum impact

**Don'ts:**
- Don't use for large background areas
- Don't use for body text
- Don't combine with similar greens
- Don't overuse - maintain visual hierarchy

---

## Dark Theme Backgrounds

### Background Hierarchy
Our dark theme creates depth through subtle variations:

```css
/* Darkest - Main editor/canvas */
--navlambda-bg-primary: #1e1e1e

/* Dark - Sidebar/panels */
--navlambda-bg-secondary: #252526

/* Medium - Cards/elevated surfaces */
--navlambda-bg-tertiary: #2d2d2d

/* Lightest - Hover states/inputs */
--navlambda-bg-elevated: #333333
```

**Visual Hierarchy:**
```
#1e1e1e (Darkest) → #252526 → #2d2d2d → #333333 (Lightest)
```

---

## Text Colors

### Text Hierarchy
```css
/* Primary - Headings, important text */
--navlambda-text-primary: #ffffff

/* Secondary - Body text, labels */
--navlambda-text-secondary: #cccccc

/* Tertiary - Supporting text */
--navlambda-text-tertiary: #999999

/* Muted - Placeholders, hints */
--navlambda-text-muted: #6e6e6e

/* Disabled - Inactive elements */
--navlambda-text-disabled: #4d4d4d
```

**Contrast Ratios:**
- Primary text on dark bg: 15.8:1 (AAA)
- Secondary text on dark bg: 10.5:1 (AAA)
- Tertiary text on dark bg: 5.2:1 (AA)

---

## Borders & Dividers

```css
/* Primary borders - Default state */
--navlambda-border-primary: #3e3e3e

/* Secondary borders - Subtle dividers */
--navlambda-border-secondary: #2d2d2d

/* Focus borders - Interactive elements */
--navlambda-border-focus: #7FD957

/* Hover borders - Hover states */
--navlambda-border-hover: #4d4d4d
```

---

## Button Styles

### Primary Button (Green)
**Use for:** Main actions, confirmations, submissions

```css
Background: #7FD957
Text: #1e1e1e (dark text for contrast)
Hover: #8FE967 + glow effect
Active: #6FC947
```

**Example:**
```html
<button class="navlambda-btn navlambda-btn-primary">
  Run Code
</button>
```

### Secondary Button (Dark)
**Use for:** Alternative actions, cancel, back

```css
Background: #3e3e3e
Text: #ffffff
Hover: #4d4d4d
Border: 1px solid #3e3e3e
```

### Tertiary Button (Transparent)
**Use for:** Tertiary actions, links, subtle interactions

```css
Background: transparent
Text: #7FD957
Border: 1px solid #3e3e3e
Hover: rgba(127, 217, 87, 0.1) background
```

### Danger Button (Red)
**Use for:** Destructive actions, delete, remove

```css
Background: #f44336
Text: #ffffff
Hover: #ff5544
```

---

## Status Colors

### Success (Green)
```css
Color: #7FD957
Background: rgba(127, 217, 87, 0.1)
Border: rgba(127, 217, 87, 0.3)
```
**Use for:** Success messages, completed tasks, positive feedback

### Error (Red)
```css
Color: #f44336
Background: rgba(244, 67, 54, 0.1)
Border: rgba(244, 67, 54, 0.3)
```
**Use for:** Error messages, failed operations, critical warnings

### Warning (Orange)
```css
Color: #ff9800
Background: rgba(255, 152, 0, 0.1)
Border: rgba(255, 152, 0, 0.3)
```
**Use for:** Warnings, cautions, important notices

### Info (Blue)
```css
Color: #2196f3
Background: rgba(33, 150, 243, 0.1)
Border: rgba(33, 150, 243, 0.3)
```
**Use for:** Informational messages, tips, neutral notifications

---

## Typography

### Font Families
```css
/* UI Text */
--navlambda-font-family: -apple-system, BlinkMacSystemFont, 
  'Segoe UI', 'Roboto', 'Helvetica Neue', sans-serif

/* Code/Monospace */
--navlambda-font-mono: 'Fira Code', 'Consolas', 
  'Monaco', 'Courier New', monospace
```

### Font Sizes
```css
--navlambda-font-size-xs: 11px   /* Small labels, captions */
--navlambda-font-size-sm: 12px   /* Body text, inputs */
--navlambda-font-size-md: 13px   /* Default size */
--navlambda-font-size-lg: 14px   /* Emphasized text */
--navlambda-font-size-xl: 16px   /* Headings */
--navlambda-font-size-2xl: 20px  /* Large headings */
--navlambda-font-size-3xl: 24px  /* Hero text */
```

### Font Weights
```css
--navlambda-font-weight-normal: 400    /* Body text */
--navlambda-font-weight-medium: 500    /* Labels */
--navlambda-font-weight-semibold: 600  /* Headings */
--navlambda-font-weight-bold: 700      /* Emphasis */
```

---

## Spacing System

### Consistent Spacing Scale
```css
--navlambda-spacing-xs: 4px    /* Tight spacing */
--navlambda-spacing-sm: 8px    /* Small gaps */
--navlambda-spacing-md: 16px   /* Default spacing */
--navlambda-spacing-lg: 24px   /* Section spacing */
--navlambda-spacing-xl: 32px   /* Large sections */
--navlambda-spacing-2xl: 48px  /* Major sections */
```

**Usage Guidelines:**
- Use `xs` (4px) for icon gaps, tight layouts
- Use `sm` (8px) for form field gaps, list items
- Use `md` (16px) for card padding, default gaps
- Use `lg` (24px) for section spacing
- Use `xl` (32px) for major component spacing
- Use `2xl` (48px) for page-level spacing

---

## Border Radius

```css
--navlambda-radius-sm: 4px    /* Small elements */
--navlambda-radius-md: 6px    /* Buttons, inputs */
--navlambda-radius-lg: 8px    /* Cards, panels */
--navlambda-radius-xl: 12px   /* Modals, large cards */
--navlambda-radius-full: 9999px /* Pills, badges */
```

---

## Shadows & Effects

### Box Shadows
```css
--navlambda-shadow-sm: 0 2px 4px rgba(0, 0, 0, 0.3)
--navlambda-shadow-md: 0 4px 8px rgba(0, 0, 0, 0.4)
--navlambda-shadow-lg: 0 8px 16px rgba(0, 0, 0, 0.5)
--navlambda-shadow-xl: 0 12px 24px rgba(0, 0, 0, 0.6)
```

### Glow Effects
```css
/* Primary glow - for green elements */
--navlambda-glow-primary: 0 0 20px rgba(127, 217, 87, 0.3)

/* Focus glow - for focused inputs */
--navlambda-glow-focus: 0 0 0 3px rgba(127, 217, 87, 0.2)
```

**Usage:**
- Use glow effects on hover for primary buttons
- Use focus glow for keyboard navigation
- Use subtle shadows for depth and hierarchy

---

## Animations & Transitions

### Transition Speeds
```css
--navlambda-transition-fast: 150ms ease
--navlambda-transition-normal: 250ms ease
--navlambda-transition-slow: 350ms ease
```

### Standard Animations
```css
/* Fade In */
@keyframes navlambda-fade-in {
  from { opacity: 0; }
  to { opacity: 1; }
}

/* Slide Up */
@keyframes navlambda-slide-up {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Pulse */
@keyframes navlambda-pulse {
  0%, 100% {
    opacity: 1;
    transform: scale(1);
  }
  50% {
    opacity: 0.8;
    transform: scale(1.05);
  }
}

/* Glow */
@keyframes navlambda-glow {
  0%, 100% {
    box-shadow: 0 0 10px rgba(127, 217, 87, 0.3);
  }
  50% {
    box-shadow: 0 0 20px rgba(127, 217, 87, 0.5);
  }
}
```

---

## Component Examples

### Input Field
```html
<input 
  class="navlambda-input" 
  type="text" 
  placeholder="Enter text..."
/>
```

**Styles:**
- Background: `#333333`
- Border: `1px solid #3e3e3e`
- Focus: Border `#7FD957` + glow
- Hover: Border `#4d4d4d`

### Card/Panel
```html
<div class="navlambda-card">
  <h3>Card Title</h3>
  <p>Card content goes here...</p>
</div>
```

**Styles:**
- Background: `#2d2d2d`
- Border: `1px solid #3e3e3e`
- Border-radius: `8px`
- Padding: `24px`
- Shadow: `0 4px 8px rgba(0, 0, 0, 0.4)`

### Modal
```html
<div class="navlambda-modal-backdrop">
  <div class="navlambda-modal">
    <h2>Modal Title</h2>
    <p>Modal content...</p>
  </div>
</div>
```

**Styles:**
- Backdrop: `rgba(0, 0, 0, 0.85)` + blur
- Modal bg: `#2d2d2d`
- Border: `1px solid #3e3e3e`
- Shadow: `0 12px 24px rgba(0, 0, 0, 0.6)`

---

## Accessibility Guidelines

### Color Contrast
- **AAA Standard:** Primary text (#ffffff) on dark backgrounds
- **AA Standard:** Secondary text (#cccccc) on dark backgrounds
- **Focus Indicators:** Always visible with 2px outline

### Keyboard Navigation
- All interactive elements must be keyboard accessible
- Focus states must be clearly visible
- Tab order must be logical

### Screen Readers
- Use semantic HTML
- Provide alt text for images
- Use ARIA labels when needed

### Reduced Motion
```css
@media (prefers-reduced-motion: reduce) {
  * {
    animation-duration: 0.01ms !important;
    transition-duration: 0.01ms !important;
  }
}
```

---

## Usage Examples

### Authentication Modal
```html
<div class="auth-modal">
  <div class="auth-card">
    <div class="auth-header">
      <div class="auth-logo">NAVΛ Studio</div>
      <h2 class="auth-title">Sign In</h2>
    </div>
    
    <form class="auth-form">
      <div class="auth-input-group">
        <label class="auth-label">Username</label>
        <input class="auth-input" type="text" />
      </div>
      
      <button class="auth-button auth-button-primary">
        Sign In
      </button>
    </form>
  </div>
</div>
```

### Status Indicator
```html
<div class="auth-status-indicator">
  <div class="auth-status-dot signed-in"></div>
  <span class="auth-status-text">
    Signed In: <span class="auth-status-user">admin</span>
  </span>
</div>
```

### Notification
```html
<div class="navlambda-notification navlambda-notification-success">
  ✓ Operation completed successfully!
</div>
```

---

## Implementation Checklist

### For New Components
- [ ] Use CSS variables from `brand-identity.css`
- [ ] Follow spacing system (4px, 8px, 16px, 24px, 32px, 48px)
- [ ] Use standard border radius (4px, 6px, 8px, 12px)
- [ ] Apply consistent shadows
- [ ] Use transition speeds (150ms, 250ms, 350ms)
- [ ] Ensure AAA contrast for text
- [ ] Add focus states for keyboard navigation
- [ ] Test with reduced motion preference
- [ ] Verify responsive behavior

### For Existing Components
- [ ] Replace hardcoded colors with CSS variables
- [ ] Update button styles to match brand
- [ ] Standardize spacing and sizing
- [ ] Add hover and focus states
- [ ] Implement consistent animations
- [ ] Test accessibility
- [ ] Verify dark theme compatibility

---

## File Structure

```
src/
├── styles/
│   └── brand-identity.css          # Central brand system
├── components/
│   ├── Auth/
│   │   ├── Login.css               # Brand-compliant auth
│   │   ├── AuthStatusIndicator.css # Status indicators
│   │   └── SecuritySettings.css    # Security UI
│   └── ...
```

---

## Quick Reference

### Most Common Colors
```css
/* Backgrounds */
#1e1e1e  /* Main editor */
#2d2d2d  /* Cards/modals */
#333333  /* Inputs/hover */

/* Text */
#ffffff  /* Primary */
#cccccc  /* Secondary */
#999999  /* Tertiary */

/* Brand */
#7FD957  /* Primary green */

/* Status */
#7FD957  /* Success */
#f44336  /* Error */
#ff9800  /* Warning */
#2196f3  /* Info */

/* Borders */
#3e3e3e  /* Default */
#7FD957  /* Focus */
```

### Most Common Spacing
```
4px   8px   16px   24px   32px   48px
xs    sm    md     lg     xl     2xl
```

### Most Common Transitions
```css
transition: all 150ms ease;  /* Fast - hover states */
transition: all 250ms ease;  /* Normal - most animations */
transition: all 350ms ease;  /* Slow - complex animations */
```

---

## Support & Questions

For questions about brand usage or design decisions:
- Review this guide first
- Check `src/styles/brand-identity.css` for implementation
- Refer to existing components for examples
- Maintain consistency across all UI elements

**Remember:** Consistency is key to a premium brand experience!

---

## Version History

- **v1.0** - Initial brand identity system
  - Established primary green (#7FD957)
  - Defined dark theme palette
  - Created component library
  - Set accessibility standards

---

*Last Updated: 2024*
*NAVΛ Studio - Premium Robotics Development Platform*
