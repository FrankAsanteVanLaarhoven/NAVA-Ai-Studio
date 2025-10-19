export function createMockContainer(): HTMLElement {
  const container = document.createElement('div');
  container.id = 'mock-container';
  document.body.appendChild(container);
  return container;
}

export function createMockMonacoEditor() {
  return {
    getContainer: vi.fn(),
    getOptions: vi.fn(),
    dispose: vi.fn(),
    onDidChangeModelContent: vi.fn(),
    onDidChangeCursorPosition: vi.fn(),
    getValue: vi.fn(),
    setValue: vi.fn(),
    updateOptions: vi.fn(),
    getOption: vi.fn(),
    setOption: vi.fn(),
    trigger: vi.fn()
  };
}