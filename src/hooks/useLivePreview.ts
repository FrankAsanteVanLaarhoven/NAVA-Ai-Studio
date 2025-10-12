import { useState, useEffect, useCallback } from 'react';
import { lspClient } from '../services/lsp-client';

export function useLivePreview(code: string, autoRun: boolean = false) {
  const [output, setOutput] = useState<string>('');
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const runPreview = useCallback(async () => {
    if (!code.trim()) return;

    setIsRunning(true);
    setError(null);

    try {
      const result = await lspClient.runLivePreview(code);
      setOutput(result);
    } catch (err) {
      setError(String(err));
    } finally {
      setIsRunning(false);
    }
  }, [code]);

  useEffect(() => {
    if (autoRun && code.trim()) {
      const timer = setTimeout(() => {
        runPreview();
      }, 500); // Debounce

      return () => clearTimeout(timer);
    }
  }, [code, autoRun, runPreview]);

  return {
    output,
    isRunning,
    error,
    runPreview,
  };
}

