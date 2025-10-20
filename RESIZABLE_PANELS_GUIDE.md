# Resizable Panels System Guide

## Overview

NAVΛ Studio IDE now features a comprehensive resizable panel system similar to VSCode and Cursor, allowing users to customize their workspace layout with draggable dividers between all panels, columns, and sections.

## Components

### 1. ResizablePanel

The `ResizablePanel` component provides horizontal (left/right) and vertical (top/bottom) resizing capabilities.

#### Features:
- **Horizontal Resizing**: For left and right sidebar panels
- **Vertical Resizing**: For top and bottom panels (e.g., notebook/terminal panel)
- **LocalStorage Persistence**: Panel sizes are automatically saved and restored
- **Collapse/Expand**: Panels can be collapsed with smooth animations
- **Visual Feedback**: Resize handles glow on hover with the signature green color
- **Smooth Dragging**: Cursor changes appropriately during resize operations

#### Usage:

```tsx
<ResizablePanel
  side="left"              // 'left' | 'right' | 'top' | 'bottom'
  defaultWidth={300}       // Default width in pixels (for horizontal)
  minWidth={200}           // Minimum width
  maxWidth={600}           // Maximum width
  defaultHeight={300}      // Default height in pixels (for vertical)
  minHeight={150}          // Minimum height
  maxHeight={600}          // Maximum height
  isCollapsed={!showPanel} // Collapse state
  onToggleCollapse={handleToggle}
  title="PANEL TITLE"
  storageKey="unique-storage-key"  // For localStorage persistence
>
  {/* Panel content */}
</ResizablePanel>
```

### 2. SplitPanel

The `SplitPanel` component divides a container into two resizable sections with a draggable divider.

#### Features:
- **Horizontal & Vertical Splits**: Split panels horizontally or vertically
- **Percentage-Based Sizing**: Uses percentage splits for responsive layouts
- **Nested Splits**: Can be nested to create complex layouts
- **LocalStorage Persistence**: Split positions are saved automatically
- **Collapsible Sections**: Individual sections can be collapsed
- **Section Titles**: Optional headers for each section
- **Min Size Constraints**: Prevents sections from becoming too small

#### Usage:

```tsx
<SplitPanel
  direction="vertical"     // 'horizontal' | 'vertical'
  defaultSplit={60}        // Percentage (0-100)
  minSize={100}            // Minimum size in pixels
  storageKey="unique-split-key"
  titles={['SECTION 1', 'SECTION 2']}  // Optional section headers
  collapsible={true}       // Enable collapse buttons
>
  {/* First section content */}
  <ComponentA />
  {/* Second section content */}
  <ComponentB />
</SplitPanel>
```

## Implementation in NAVΛ Studio IDE

### Current Layout Structure:

```
┌──────────────────────────────────────────────────────┐
│                      Toolbar                         │
├──────┬───────────────────────────────────┬──────────┤
│      │                                   │          │
│  A   │                                   │    R     │
│  C   │           Editor Area             │    i     │
│  T   │                                   │    g     │
│  I   │                                   │    h     │
│  V   ├───────────────────────────────────┤    t     │
│  I   │                                   │          │
│  T   │         Bottom Panel              │    P     │
│  Y   │         (Notebook)                │    a     │
│      │                                   │    n     │
│  B   │                                   │    e     │
│  A   │                                   │    l     │
│  R   │                                   │          │
│      │                                   │          │
└──────┴───────────────────────────────────┴──────────┘
```

### Panels with Resizable Dividers:

1. **Left Sidebar** (Activity Panel)
   - Resizable horizontally
   - Storage key: `nav-studio-left-panel-width`
   - Contains nested SplitPanels for Explorer activity:
     - File Explorer
     - Outline & Timeline (split vertically)

2. **Bottom Panel** (Notebook)
   - Resizable vertically
   - Storage key: `nav-studio-bottom-panel-height`
   - Collapsible

3. **Right Panel** (Visualizer/AI/Collaboration)
   - Resizable horizontally
   - Storage key: `nav-studio-right-panel-width`
   - Collapsible

4. **Explorer Split Panels** (Nested in Left Sidebar)
   - First split: Explorer vs (Outline + Timeline)
     - Storage key: `nav-studio-explorer-split`
   - Second split: Outline vs Timeline
     - Storage key: `nav-studio-outline-timeline-split`

### Example: Explorer Activity with Nested Splits

```tsx
<SplitPanel
  direction="vertical"
  defaultSplit={60}
  minSize={100}
  storageKey="nav-studio-explorer-split"
  titles={['EXPLORER', 'OUTLINE & TIMELINE']}
  collapsible={true}
>
  <FileExplorer />
  <SplitPanel
    direction="vertical"
    defaultSplit={50}
    minSize={80}
    storageKey="nav-studio-outline-timeline-split"
    titles={['OUTLINE', 'TIMELINE']}
    collapsible={true}
  >
    <Outline hideHeader={true} />
    <Timeline hideHeader={true} />
  </SplitPanel>
</SplitPanel>
```

## Visual Styling

### Resize Handles

- **Horizontal Handles** (for vertical resizing):
  - Height: 4px
  - Cursor: `row-resize`
  - Color: Green (#00ff00) with glow effect on hover

- **Vertical Handles** (for horizontal resizing):
  - Width: 4px
  - Cursor: `col-resize`
  - Color: Green (#00ff00) with glow effect on hover

### Animations

- Smooth transitions when collapsing/expanding panels
- Handle bars fade in/out on hover
- Cursor changes globally during drag operations
- Glowing effect on active drag

## User Interactions

### Resizing Panels:

1. Hover over any panel divider
2. The resize handle will glow green
3. Click and drag to resize
4. Release to lock the new size
5. Size is automatically saved to localStorage

### Collapsing Panels:

1. Click the collapse button (◀ ▶ ▲ ▼) in the panel header
2. Panel smoothly collapses
3. An expand button appears to restore the panel

### Keyboard Shortcuts:

- `Ctrl+B`: Toggle left sidebar
- `Ctrl+J`: Toggle bottom panel
- `Ctrl+\`: Toggle right panel

## Best Practices

### For Developers:

1. **Always provide unique storage keys** for persistence
2. **Set appropriate min/max sizes** to prevent UI breaking
3. **Use meaningful panel titles** for user clarity
4. **Test nested layouts** thoroughly for resize conflicts
5. **Consider mobile/responsive** behavior

### For Users:

1. **Drag slowly** for precise sizing
2. **Double-click dividers** to reset to default (feature to be implemented)
3. **Use collapse buttons** to temporarily hide panels
4. **Experiment with layouts** - settings are saved automatically

## Technical Details

### LocalStorage Keys:

- `nav-studio-left-panel-width`: Left sidebar width
- `nav-studio-right-panel-width`: Right panel width
- `nav-studio-bottom-panel-height`: Bottom panel height
- `nav-studio-explorer-split`: Explorer/Outline split position
- `nav-studio-outline-timeline-split`: Outline/Timeline split position

### CSS Classes:

- `.resizable-panel`: Main panel container
- `.resize-handle`: Draggable handle element
- `.split-panel`: Split container
- `.split-divider`: Divider between split sections
- `.dragging`: Active drag state

### Browser Compatibility:

- Chrome/Edge: ✅ Full support
- Firefox: ✅ Full support
- Safari: ✅ Full support
- Mobile browsers: ⚠️ Limited (touch events may need adjustment)

## Future Enhancements

1. **Double-click to reset**: Reset panels to default sizes
2. **Layout presets**: Save and load custom layout configurations
3. **Keyboard resize**: Use arrow keys to resize panels
4. **Panel snapshots**: Save workspace layouts per project
5. **Touch gestures**: Better mobile/tablet support
6. **Panel animations**: More fluid transitions

## Troubleshooting

### Panel won't resize:
- Check if min/max constraints are too restrictive
- Ensure the panel is not collapsed
- Verify the resize handle is not hidden by another element

### Size not persisting:
- Check browser localStorage is enabled
- Verify unique storage keys are provided
- Clear browser cache if necessary

### Nested panels conflicting:
- Ensure parent containers have proper overflow handling
- Check z-index values for resize handles
- Verify event propagation is not blocked

## Component Files

- **ResizablePanel**: `src/components/Common/ResizablePanel.tsx`
- **SplitPanel**: `src/components/Common/SplitPanel.tsx`
- **Styles**: 
  - `src/components/Common/ResizablePanel.css`
  - `src/components/Common/SplitPanel.css`
- **Usage**: `src/App.tsx`

---

**Last Updated**: October 19, 2025  
**Version**: 1.0.0  
**Status**: ✅ Production Ready

