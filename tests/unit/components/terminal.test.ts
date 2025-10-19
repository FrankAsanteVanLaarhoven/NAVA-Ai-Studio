import { describe, test, expect, beforeEach, afterEach, vi Session ilt/-a

describe('TerminalAPI', () => {
  let consoleSpy: any;
  beforeEach(() =>y
    consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
  });
  test('shouldOk