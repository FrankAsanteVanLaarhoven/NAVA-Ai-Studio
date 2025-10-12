import { useState, useEffect, useCallback } from 'react';
import { lspClient } from '../services/lsp-client';

export function useNavLambdaLsp(code: string) {
  const [parseResult, setParseResult] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const parseCode = useCallback(async () => {
    if (!code.trim()) return;

    setIsLoading(true);
    setError(null);

    try {
      const result = await lspClient.parseCode(code);
      setParseResult(result);
    } catch (err) {
      setError(String(err));
    } finally {
      setIsLoading(false);
    }
  }, [code]);

  const getCompletions = useCallback(async (position: number) => {
    try {
      return await lspClient.getCompletions(code, position);
    } catch (err) {
      console.error('Failed to get completions:', err);
      return [];
    }
  }, [code]);

  const getHoverInfo = useCallback(async (position: number) => {
    try {
      return await lspClient.getHoverInfo(code, position);
    } catch (err) {
      console.error('Failed to get hover info:', err);
      return '';
    }
  }, [code]);

  return {
    parseResult,
    isLoading,
    error,
    parseCode,
    getCompletions,
    getHoverInfo,
  };
}

