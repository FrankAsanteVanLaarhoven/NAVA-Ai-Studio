import { describe, test, expect, beforeEach, afterEach, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { NavLambdaMonacoEditor } from '../../../src/components/Editor/MonacoEditor';

// Mock Monaco Editor
const mockEditor = {
  getValue: vi.fn(),
  setValue: vi.fn(),
  dispose: vi.fn(),
  onDidChangeModelContent: vi.fn(),
  onDidChangeCursorPosition: vi.fn(),
  addCommand: vi.fn(),
  trigger: vi.fn(),
  getModel: vi.fn(),
  updateOptions: vi.fn(),
};

const mockMonaco = {
  editor: {
    create: vi.fn(() => mockEditor),
  },
  KeyMod: {
    Alt: 1,
    Shift: 2,
    CtrlCmd: 3,
  },
  KeyCode: {
    KeyL: 4,
    KeyT: 5,
    KeyS: 6,
    KeyM: 7,
    KeyE: 8,
    KeyS: 9,
  },
  Position: vi.fn(),
};

// Mock the monaco module
vi.mock('monaco-editor', () => mockMonaco);

// Mock the NAVΛ language registration
vi.mock('../../../src/services/navlambda-language', () => ({
  registerNavLambdaLanguage: vi.fn(),
}));

describe('NavLambdaMonacoEditor', () => {
  const mockOnCodeChange = vi.fn();
  const mockOnCursorPositionChange = vi.fn();
  const initialCode = 'nav test() {\n  ⋋\n}';
  const filePath = '/test/main.navλ';

  beforeEach(() => {
    vi.clearAllMocks();

    // Reset mock editor methods
    mockEditor.getValue.mockReturnValue(initialCode);
    mockEditor.setValue.mockImplementation(() => {});
    mockEditor.dispose.mockImplementation(() => {});
    mockEditor.onDidChangeModelContent.mockImplementation(() => ({
      dispose: vi.fn(),
    }));
    mockEditor.onDidChangeCursorPosition.mockImplementation(() => ({
      dispose: vi.fn(),
    }));
    mockEditor.addCommand.mockImplementation(() => {});
    mockEditor.trigger.mockImplementation(() => {});
    mockEditor.getModel.mockReturnValue({});
    mockEditor.updateOptions.mockImplementation(() => {});

    // Mock window event dispatching
    const mockDispatchEvent = vi.fn();
    Object.defineProperty(window, 'dispatchEvent', {
      value: mockDispatchEvent,
      writable: true,
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  test('renders editor container', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
        filePath={filePath}
      />
    );

    expect(screen.getByText('main.navλ')).toBeInTheDocument();
    expect(mockMonaco.editor.create).toHaveBeenCalledWith(
      expect.any(HTMLDivElement),
      expect.objectContaining({
        value: initialCode,
        language: 'navlambda',
        theme: 'vnc-theme',
        fontSize: 14,
      })
    );
  });

  test('registers NAVΛ language on first mount', () => {
    const { rerender } = render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    // Should register language on first render
    expect(vi.mocked(await import('../../../src/services/navlambda-language')).registerNavLambdaLanguage).toHaveBeenCalledTimes(1);

    // Should not register again on re-render
    rerender(
      <NavLambdaMonacoEditor
        initialCode="new code"
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(vi.mocked(await import('../../../src/services/navlambda-language')).registerNavLambdaLanguage).toHaveBeenCalledTimes(1);
  });

  test('calls onCodeChange when content changes', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    // Simulate content change
    const contentChangeCallback = mockEditor.onDidChangeModelContent.mock.calls[0][0];
    mockEditor.getValue.mockReturnValue('updated code');

    contentChangeCallback();

    expect(mockOnCodeChange).toHaveBeenCalledWith('updated code');
  });

  test('calls onCursorPositionChange when cursor moves', () => {
    const mockPosition = { lineNumber: 1, column: 5 };

    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
        onCursorPositionChange={mockOnCursorPositionChange}
      />
    );

    // Simulate cursor position change
    const cursorChangeCallback = mockEditor.onDidChangeCursorPosition.mock.calls[0][0];

    cursorChangeCallback({ position: mockPosition });

    expect(mockOnCursorPositionChange).toHaveBeenCalledWith(mockPosition);
  });

  test('does not set up cursor position listener when callback not provided', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(mockEditor.onDidChangeCursorPosition).not.toHaveBeenCalled();
  });

  test('updates editor value when initialCode changes', () => {
    const { rerender } = render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    // Initial value should be set
    expect(mockEditor.setValue).not.toHaveBeenCalled();

    // Change initialCode
    const newCode = 'new nav code';
    rerender(
      <NavLambdaMonacoEditor
        initialCode={newCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(mockEditor.getValue).toHaveBeenCalled();
    expect(mockEditor.setValue).toHaveBeenCalledWith(newCode);
  });

  test('does not update value if it matches current editor value', () => {
    mockEditor.getValue.mockReturnValue(initialCode);

    const { rerender } = render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    rerender(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(mockEditor.setValue).not.toHaveBeenCalled();
  });

  test('disposes editor on unmount', () => {
    const { unmount } = render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    unmount();

    expect(mockEditor.dispose).toHaveBeenCalled();
  });

  test('renders symbol palette buttons', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
        filePath={filePath}
      />
    );

    expect(screen.getByTitle('Lambda Navigation (Alt+L)')).toBeInTheDocument();
    expect(screen.getByTitle('Tensor Product (Alt+Shift+T)')).toBeInTheDocument();
    expect(screen.getByTitle('Navigation Sum (Alt+Shift+S)')).toBeInTheDocument();
    expect(screen.getByTitle('Master Operator (Alt+Shift+M)')).toBeInTheDocument();
    expect(screen.getByTitle('Evolution (Alt+Shift+E)')).toBeInTheDocument();
  });

  test('inserts symbol when palette button is clicked', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    const lambdaButton = screen.getByTitle('Lambda Navigation (Alt+L)');
    fireEvent.click(lambdaButton);

    expect(mockEditor.trigger).toHaveBeenCalledWith('keyboard', 'type', { text: '⋋' });
  });

  test('sets up keyboard shortcuts for symbols', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    // Check that addCommand was called for each shortcut
    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.Alt | mockMonaco.KeyCode.KeyL,
      expect.any(Function)
    );

    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.Alt | mockMonaco.KeyMod.Shift | mockMonaco.KeyCode.KeyT,
      expect.any(Function)
    );

    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.Alt | mockMonaco.KeyMod.Shift | mockMonaco.KeyCode.KeyS,
      expect.any(Function)
    );

    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.Alt | mockMonaco.KeyMod.Shift | mockMonaco.KeyCode.KeyM,
      expect.any(Function)
    );

    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.Alt | mockMonaco.KeyMod.Shift | mockMonaco.KeyCode.KeyE,
      expect.any(Function)
    );
  });

  test('sets up save keyboard shortcut', () => {
    // Mock window.dispatchEvent
    const mockDispatchEvent = vi.fn();
    Object.defineProperty(window, 'dispatchEvent', {
      value: mockDispatchEvent,
      writable: true,
    });

    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(mockEditor.addCommand).toHaveBeenCalledWith(
      mockMonaco.KeyMod.CtrlCmd | mockMonaco.KeyCode.KeyS,
      expect.any(Function)
    );

    // Find the save command callback and execute it
    const saveCommandCall = mockEditor.addCommand.mock.calls.find(
      call => call[0] === (mockMonaco.KeyMod.CtrlCmd | mockMonaco.KeyCode.KeyS)
    );

    if (saveCommandCall) {
      const saveCallback = saveCommandCall[1];
      mockEditor.getValue.mockReturnValue('code to save');
      saveCallback();

      expect(mockDispatchEvent).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'editor-save',
          detail: { code: 'code to save' },
        })
      );
    }
  });

  test('displays file name when filePath is provided', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
        filePath="/deep/nested/path/file.navλ"
      />
    );

    expect(screen.getByText('file.navλ')).toBeInTheDocument();
  });

  test('does not display file info when filePath is not provided', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(screen.queryByText('main.navλ')).not.toBeInTheDocument();
  });

  test('configures editor with correct options', () => {
    render(
      <NavLambdaMonacoEditor
        initialCode={initialCode}
        onCodeChange={mockOnCodeChange}
      />
    );

    expect(mockMonaco.editor.create).toHaveBeenCalledWith(
      expect.any(HTMLDivElement),
      expect.objectContaining({
        language: 'navlambda',
        theme: 'vnc-theme',
        fontSize: 14,
        fontFamily: 'JetBrains Mono, Consolas, monospace',
        lineNumbers: 'on',
        minimap: { enabled: true },
        tabSize: 2,
        insertSpaces: true,
        wordWrap: 'on',
        folding: true,
        bracketPairColorization: { enabled: true },
      })
    );
  });
});