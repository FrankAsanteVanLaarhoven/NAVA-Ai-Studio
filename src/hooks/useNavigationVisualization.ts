import { useState, useCallback } from 'react';
import { NavigationVisualization } from '../types';
import { lspClient } from '../services/lsp-client';

export function useNavigationVisualization() {
  const [visualization, setVisualization] = useState<NavigationVisualization | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const visualize = useCallback(async (code: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const result = await lspClient.visualizeNavigationPath(code);
      const vizData = JSON.parse(result) as NavigationVisualization;
      setVisualization(vizData);
    } catch (err) {
      setError(String(err));
    } finally {
      setIsLoading(false);
    }
  }, []);

  const clearVisualization = useCallback(() => {
    setVisualization(null);
    setError(null);
  }, []);

  return {
    visualization,
    isLoading,
    error,
    visualize,
    clearVisualization,
  };
}

