# Component-Specific Styles

This directory contains CSS modules for individual components.

## Organization

Each major component should have its own CSS file:

- `editor.module.css` - Editor component styles
- `visualizer.module.css` - 3D visualizer styles
- `toolbar.module.css` - Toolbar component styles
- `statusbar.module.css` - Status bar styles
- etc.

## CSS Modules

We use CSS modules for component-scoped styling:

```tsx
import styles from './component.module.css';

export const Component = () => {
  return <div className={styles.container}>...</div>;
};
```

## Naming Conventions

- Use camelCase for class names
- Prefix utility classes with `u-`
- Prefix layout classes with `l-`
- Prefix component classes with `c-`

## Example

```css
/* editor.module.css */
.container {
  display: flex;
  height: 100%;
}

.codeArea {
  flex: 1;
  font-family: var(--font-mono);
}

.symbolPalette {
  width: 200px;
  border-left: 1px solid var(--vnc-border);
}
```

